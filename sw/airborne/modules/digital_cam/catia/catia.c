/**
 * @file catia.c
 * @brief Main CATIA daemon coordinating flight-controller triggers and camera backends.
 * @details The event loop receives validated UART frames, queues bounded capture jobs,
 * dispatches backend work on detached workers, records metadata, and returns status.
 * It deliberately keeps serial I/O separate from slow camera processes so one capture
 * cannot block command reception. Backend failure is isolated per camera wherever
 * possible; systemd owns whole-daemon restart for irrecoverable transport failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include "version.h"
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "ai_cam_pipe.h"
#include "vehicle_detect_pipe.h"
#include "home_vector_pipe.h"
#include "camera_speedtest.h"
#include "ear_cam_pipe.h"
#include "lwir_cam_pipe.h"
#include "serial.h"
#include "serial_tx.h"
#include "chdk_pipe.h"
#include "image_exif.h"
#include "image_mock_transform.h"
#include "local_pipe.h"
#include "std.h"
#include "protocol.h"
#include "pose_log.h"
#include "socket.h"
#include "path_utils.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 8
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
// Search&Rescue Onboard Detection Application
#define SODA "./soda"
#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define DEBUG_HEARTBEAT_SECONDS 5
#define SERIAL_READ_BUFFER_SIZE 256
#define EVENT_LOOP_TIMEOUT_MS 1000
#define SODA_SHUTDOWN_GRACE_MS 2000
#define SODA_WAIT_INTERVAL_US 10000
#ifndef LWIR_RETRY_COOLDOWN_US
#define LWIR_RETRY_COOLDOWN_US 30000000U
#endif
#define LWIR_RETRY_POLL_US 100000

#ifndef CATIA_SERIAL_DEVICE
#define CATIA_SERIAL_DEVICE "/dev/ttySAC0"
#endif

#ifndef CATIA_LOCAL_SIM_DEVICE
#define CATIA_LOCAL_SIM_DEVICE "/tmp/catia-sim"
#endif

#ifndef CATIA_LOCAL_APP_DEVICE
#define CATIA_LOCAL_APP_DEVICE "/tmp/catia-app"
#endif

#ifndef CATIA_LOCAL_LOCK_FILE
#define CATIA_LOCAL_LOCK_FILE "/tmp/catia-local.lock"
#endif

#ifndef CATIA_MOCK_IMAGE
#define CATIA_MOCK_IMAGE "mock-camera.jpg"
#endif

#ifndef CATIA_CHDK_PHOTO_DIR
#define CATIA_CHDK_PHOTO_DIR "photos"
#endif

#ifndef CATIA_AI_CAM_PHOTO_DIR
#define CATIA_AI_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_LWIR_CAM_PHOTO_DIR
#define CATIA_LWIR_CAM_PHOTO_DIR "photos"
#endif

#ifndef CATIA_SODA
#define CATIA_SODA "soda"
#endif

enum camera_backend_type {
  CAMERA_BACKEND_UNSELECTED,
  CAMERA_BACKEND_CHDK,
  CAMERA_BACKEND_LOCAL,
  CAMERA_BACKEND_AI_CAM,
  CAMERA_BACKEND_AI_CAM_DETECT,
  CAMERA_BACKEND_AI_CAM_HOME,
  CAMERA_BACKEND_LWIR_CAM
};

struct camera_backend {
  const char *name;
  int (*init)(const char *source_image);
  int (*shoot)(char *filename, size_t filename_size, int image_number);
  void (*deinit)(void);
  const char *soda_application;
};

struct capture_job {
  union dc_shot_union shot;
  uint8_t camera_mask;
  uint64_t ticket;
};

/** A captured file waiting for EXIF and SODA, with what they need from capture time. */
struct captured_image {
  char filename[MAX_FILENAME];
  int camera_id;
  double capture_delay;
  struct capture_timing capture_times;
};

static void *handle_msg_shoot(void *ptr);
static bool capture_image(const union dc_shot_union *shoot, struct captured_image *image,
                          int camera_id, const struct camera_backend *backend);
static void finish_image(const union dc_shot_union *shoot, struct captured_image *image);
static void handle_received_message(void);
static void start_shoot_worker(const union dc_shot_union *shoot, uint8_t camera_mask);
static void handle_targeted_stop(int32_t camera_id);
static bool ensure_earcam_active(void);
static void send_msg_ear_result(const struct ear_loudest_spot *spot);
static void send_msg_home_vector_result(void);
static int run_soda(const char *filename, const union dc_shot_union *shoot, int camera_id);
static void release_worker_slot(void);
static inline void send_msg_image_buffer(void);
static inline void send_msg_status(void);
static void print_usage(const char *program);
static int lock_local_instance(void);
static pid_t start_local_serial_bridge(void);
static int wait_for_path(const char *path);
static void stop_local_serial_bridge(void);
static void handle_signal(int signal_number);
static void notify_systemd_ready(void);
static int move_file(const char *source, const char *destination);
static int camera_backend_select(enum camera_backend_type type, bool test_mode);
static int camera_prepare(int camera_id);
static bool lwir_ready_or_start_recovery(void);
static void lwir_mark_backoff(void);
static void cameras_deinit(void);
static int local_backend_shoot(char *filename, size_t filename_size, int image_number);
static int chdk_backend_init(const char *source_image);
static int chdk_backend_shoot(char *filename, size_t filename_size, int image_number);

static volatile int is_shooting, image_idx, image_count, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t lwir_camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t camera_available = PTHREAD_COND_INITIALIZER;
static pthread_cond_t workers_finished = PTHREAD_COND_INITIALIZER;
static uint64_t next_job_ticket, next_capture_ticket;
static bool local_mode;
static enum camera_backend_type requested_camera_backend = CAMERA_BACKEND_UNSELECTED;
static struct camera_backend camera;
static bool camera_initialized[4];
static bool camera_unavailable[4];
static bool camera_warned[4];
/** Tiny1-C server startup can take tens of seconds. Recovery runs outside the ordered
 * optical capture lane and is rate-limited so a broken USB link cannot stall AIcam. */
static uint64_t lwir_retry_after_us;
static bool lwir_recovery_in_progress;
static bool lwir_async_recovery_enabled = true;
static uint64_t lwir_busy_skipped_count;
static bool local_capture_only;
/** True when --aicam-detect was selected: camera id CATIA_CAMERA_AICAM still means "the
 * AI camera" on the wire (unchanged for the flight controller/NPS), but camera_prepare()
 * and cameras_deinit() route it to the vehicle-detection backend instead of plain
 * ai_cam_pipe capture. Plain --aicam behavior is completely unaffected. */
static bool aicam_detect_enabled;
/** True when --aicam-home was selected: same wire-compatible routing trick as
 * aicam_detect_enabled above, but for the visual-homing backend. Mutually exclusive with
 * aicam_detect_enabled (only one camera backend is selected per CATIA process). */
static bool aicam_home_enabled;
static const char *camera_source_image;
static bool local_bridge_requested;
static bool local_bridge_owned;
static volatile sig_atomic_t keep_running = 1;
static pid_t socat_pid = -1;
static int local_lock_fd = -1;
static const char *mock_image = CATIA_MOCK_IMAGE;
static bool mock_transform_enabled;
static bool debug_enabled;
static bool test_capture_enabled;
static bool earcam_requested;
static bool earcam_active;
static bool earcam_unavailable;
static bool earcam_simulated;
static int optical_camera_id = CATIA_CAMERA_ALL;
static uint64_t serial_receive_monotonic_us;
static struct clock_alignment fc_clock;
static bool clock_probes_enabled;
#ifndef CATIA_POSE_LOG_DIR
#define CATIA_POSE_LOG_DIR NULL
#endif
static const char *pose_log_dir = CATIA_POSE_LOG_DIR;
static bool motion_compensation_enabled;
static const char *chdk_photo_directory = CATIA_CHDK_PHOTO_DIR;
static uint64_t next_clock_probe_us, clock_probe_count, clock_reply_count, clock_rejected_count;

/** @brief Read CATIA's monotonic clock for local latency and event ordering.
 * @return Microseconds since an arbitrary monotonic origin, or zero on clock failure. */
static uint64_t monotonic_time_us(void)
{
  struct timespec now;
  return clock_gettime(CLOCK_MONOTONIC, &now) == 0
      ? (uint64_t)now.tv_sec * 1000000U + (uint64_t)now.tv_nsec / 1000U : 0;
}

/** @brief Opportunistically send a low-priority flight-controller clock probe.
 * @details Probes are rate-limited to one per second and sent only on an idle UART so
 * alignment evidence cannot delay capture/status traffic. */
static void send_clock_probe(uint64_t now_us)
{
  if (!clock_probes_enabled || now_us == 0 || now_us < next_clock_probe_us
      || now_us > UINT64_MAX - 1000000) return;
  next_clock_probe_us = now_us + 1000000;
  union catia_clock_request_union token;
  if (getrandom(token.bin, sizeof(token.bin), GRND_NONBLOCK) != (ssize_t)sizeof(token.bin)) return;
  const uint64_t sent_us = monotonic_time_us();
  if (!clock_alignment_request(&fc_clock, token, sent_us)) return;
  if (serial_tx_send_if_idle(CATIA_CLOCK_REQUEST, token.bin, sizeof(token.bin)) != 0) {
    fc_clock.pending = false;
    return;
  }
  ++clock_probe_count;
}

/** @brief Validate and enqueue a token-bound pose sample with clock evidence.
 * @details Mapping failure does not discard the pose: the log records it as unmapped,
 * preserving raw diagnostic evidence without claiming false time precision. */
static void record_clocked_pose(void)
{
  if (catia_protocol.payload_len != CATIA_POSE_CLOCKED_MSG_SIZE) {
    pose_log_record(NULL, 0, serial_receive_monotonic_us);
    return;
  }
  union catia_pose_clocked_union message;
  for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = catia_protocol.payload[index];
  struct pose_clock_evidence evidence = {.token = message.data.request};
  struct clock_interval begin, end;
  if (clock_alignment_map(&fc_clock, message.data.request, message.data.sample.data.sample_begin_us,
                           serial_receive_monotonic_us, &begin)
      && clock_alignment_map(&fc_clock, message.data.request, message.data.sample.data.sample_end_us,
                              serial_receive_monotonic_us, &end)) {
    evidence.sample_time = (struct clock_interval){begin.earliest_us, end.latest_us};
    evidence.probe_sent_us = fc_clock.mapped_sent_us;
    evidence.probe_received_us = fc_clock.received_us;
    evidence.probe_receive_fc_us = fc_clock.receive_fc_us;
    evidence.probe_transmit_fc_us = fc_clock.anchor_fc_us;
    evidence.mapped = true;
  }
  pose_log_record_clocked(message.data.sample.bin, sizeof(message.data.sample.bin),
                          serial_receive_monotonic_us, &evidence);
}

extern char **environ;

/** @brief Configure CATIA and run its multiplexed UART/local-UDP event loop.
 * @return Process status suitable for systemd restart policy.
 * @details After argument validation, startup claims the fixed local payload endpoint
 * before touching shared UART or camera resources. This rejects a second process early
 * when the systemd service is already active. CATIA then validates selected backends,
 * starts optional persistent services, initializes non-blocking transport, and drains
 * serial bytes into the protocol parser. Shutdown wakes and drains detached workers
 * before releasing backend resources. */
int main(int argc, char *argv[])
{
  const char *serial_device = CATIA_SERIAL_DEVICE;
  bool serial_was_selected = false;
  bool mock_image_was_selected = false;
  bool test_mode = false;
  bool speedtest_enabled = false;
  bool earcam_option_selected = false;
  bool pose_log_selected = false;
  bool clock_align_selected = false;
  bool lwir_raw_selected = false;
  bool lwir_calibration_selected = false;
  bool motion_compensation_selected = false;
  uint64_t serial_byte_count = 0;
  uint64_t valid_frame_count = 0;
  uint64_t rejected_frame_count = 0;
  time_t next_debug_heartbeat = 0;
  unsigned char serial_buffer[SERIAL_READ_BUFFER_SIZE];
  int option;

  static const struct option long_options[] = {
    {"serial", required_argument, NULL, 's'},
    {"local", no_argument, NULL, 'l'},
    {"chdk", no_argument, NULL, 'c'},
    {"aicam", no_argument, NULL, 'a'},
    {"aicam-detect", no_argument, NULL, 1002},
    {"aicam-home", no_argument, NULL, 1003},
    {"lwircam", no_argument, NULL, 'w'},
    {"earcam", no_argument, NULL, 'e'},
    {"earcam-sim", required_argument, NULL, 'E'},
    {"earcam-band", required_argument, NULL, 'B'},
    {"test", no_argument, NULL, 't'},
    {"mocktransform", no_argument, NULL, 'f'},
    {"debug", no_argument, NULL, 'd'},
    {"mock-image", required_argument, NULL, 'i'},
    {"pose-log", required_argument, NULL, 'p'},
    {"clock-align", no_argument, NULL, 'k'},
    {"lwir-raw", no_argument, NULL, 'r'},
    {"lwir-calibration", required_argument, NULL, 'y'},
    {"lwir-motion-compensation", no_argument, NULL, 'm'},
    {"help", no_argument, NULL, 'h'},
    {"version", no_argument, NULL, 1000},
    {"speedtest", no_argument, NULL, 1001},
    {NULL, 0, NULL, 0}
  };

  while ((option = getopt_long(argc, argv, "s:lcaweE:B:tfdi:p:kry:mh", long_options, NULL)) != -1) {
    switch (option) {
      case 's':
        serial_device = optarg;
        serial_was_selected = true;
        break;
      case 'l':
        local_mode = true;
        local_bridge_requested = true;
        break;
      case 'c':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_CHDK;
        break;
      case 'a':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_AI_CAM;
        break;
      case 1002:
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_AI_CAM_DETECT;
        break;
      case 1003:
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_AI_CAM_HOME;
        break;
      case 'w':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_LWIR_CAM;
        break;
      case 'e':
        earcam_requested = true;
        earcam_option_selected = true;
        break;
      case 'E': {
        double sim_lat = 0.0;
        double sim_lon = 0.0;
        double sim_level = 95.0;
        int fields = sscanf(optarg, "%lf,%lf,%lf", &sim_lat, &sim_lon, &sim_level);
        if (fields < 2 || sim_lat < -90.0 || sim_lat > 90.0 || sim_lon < -180.0 || sim_lon > 180.0) {
          fprintf(stderr, "CATIA:\t--earcam-sim expects LAT,LON[,LEVEL_DB_AT_1M]\n");
          return 2;
        }
        ear_cam_pipe_set_simulated_source(sim_lat, sim_lon, sim_level);
        earcam_requested = true;
        earcam_simulated = true;
        earcam_option_selected = true;
        break;
      }
      case 'B': {
        double low_hz = 0.0;
        double high_hz = 0.0;
        if (sscanf(optarg, "%lf,%lf", &low_hz, &high_hz) != 2 || low_hz < 100.0 || high_hz <= low_hz
            || high_hz > 20000.0) {
          fprintf(stderr, "CATIA:\t--earcam-band expects LOW_HZ,HIGH_HZ\n");
          return 2;
        }
        ear_cam_pipe_set_band(low_hz, high_hz);
        earcam_option_selected = true;
        break;
      }
      case 't':
        test_mode = true;
        break;
      case 'f':
        mock_transform_enabled = true;
        break;
      case 'd':
        debug_enabled = true;
        break;
      case 'i':
        mock_image = optarg;
        mock_image_was_selected = true;
        break;
      case 'p':
        if (optarg[0] == '\0') {
          fprintf(stderr, "CATIA:\t--pose-log expects a directory\n");
          return 2;
        }
        pose_log_dir = optarg;
        pose_log_selected = true;
        break;
      case 'k':
        clock_probes_enabled = true;
        clock_align_selected = true;
        break;
      case 'r':
        lwir_cam_pipe_set_native_raw(1);
        lwir_raw_selected = true;
        break;
      case 'y':
        if (lwir_cam_pipe_set_calibration(optarg) != 0) {
          fprintf(stderr, "CATIA:\t--lwir-calibration expects a camera YAML file\n");
          return 2;
        }
        lwir_calibration_selected = true;
        break;
      case 'm':
        motion_compensation_enabled = true;
        motion_compensation_selected = true;
        break;
      case 1000:
        puts(CATIA_BUILD_VERSION);
        return 0;
      case 1001:
        speedtest_enabled = true;
        break;
      case 'h':
        print_usage(argv[0]);
        return 0;
      default:
        print_usage(argv[0]);
        return 2;
    }
  }

  if (optind != argc) {
    print_usage(argv[0]);
    return 2;
  }

  if (mock_image_was_selected && !test_mode
      && (!local_mode || requested_camera_backend != CAMERA_BACKEND_UNSELECTED)) {
    fprintf(stderr, "CATIA:\t--mock-image requires --test or --local without a camera backend\n");
    return 2;
  }
  if (mock_transform_enabled && !test_mode) {
    fprintf(stderr, "CATIA:\t--mocktransform requires --test\n");
    return 2;
  }
  if (speedtest_enabled) {
    if (requested_camera_backend == CAMERA_BACKEND_UNSELECTED) {
      fprintf(stderr, "CATIA:\t--speedtest requires --chdk, --aicam, or --lwircam\n");
      return 2;
    }
    if (serial_was_selected || local_mode || test_mode || mock_image_was_selected
      || mock_transform_enabled || debug_enabled || earcam_option_selected || pose_log_selected
        || clock_align_selected || lwir_raw_selected || lwir_calibration_selected
        || motion_compensation_selected) {
      fprintf(stderr, "CATIA:\t--speedtest cannot be combined with daemon, mock, EARcam, pose, or LWIR processing options\n");
      return 2;
    }
  }

  if (debug_enabled) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IOLBF, 0);
  }

  /* The fixed loopback payload endpoint is also the physical-mode instance
   * claim. Acquire it before opening UART or camera resources so a foreground
   * invocation cannot compete with the systemd-managed CATIA service. */
  if (socket_init(1) != 0) {
    return 1;
  }

  if (!local_mode && !serial_was_selected && access(CATIA_SERIAL_DEVICE, R_OK | W_OK) != 0
      && access(CATIA_LOCAL_SIM_DEVICE, F_OK) == 0 && access(CATIA_LOCAL_APP_DEVICE, F_OK) == 0) {
    local_mode = true;
    serial_device = CATIA_LOCAL_APP_DEVICE;
    printf("CATIA:\tauto-detected existing local serial bridge\n");
  }

  if (local_mode) {
    if (serial_was_selected) {
      fprintf(stderr, "CATIA:\t--serial cannot be combined with --local\n");
      return 2;
    }
    if (local_bridge_requested) {
      if (lock_local_instance() != 0) {
        return 1;
      }
      socat_pid = start_local_serial_bridge();
      if (socat_pid < 0) {
        return 1;
      }
      local_bridge_owned = true;
    }
    serial_device = CATIA_LOCAL_APP_DEVICE;
  }

  enum camera_backend_type selected_camera_backend = requested_camera_backend;
  if (selected_camera_backend == CAMERA_BACKEND_UNSELECTED) {
    selected_camera_backend = local_mode ? CAMERA_BACKEND_LOCAL : CAMERA_BACKEND_CHDK;
  }
  if (camera_backend_select(selected_camera_backend, test_mode) != 0) {
    stop_local_serial_bridge();
    return 2;
  }
  test_capture_enabled = test_mode;

  switch (selected_camera_backend) {
    case CAMERA_BACKEND_CHDK: optical_camera_id = CATIA_CAMERA_CHDK; break;
    case CAMERA_BACKEND_AI_CAM: optical_camera_id = CATIA_CAMERA_AICAM; break;
    case CAMERA_BACKEND_AI_CAM_DETECT: optical_camera_id = CATIA_CAMERA_AICAM; break;
    case CAMERA_BACKEND_AI_CAM_HOME: optical_camera_id = CATIA_CAMERA_AICAM; break;
    case CAMERA_BACKEND_LWIR_CAM: optical_camera_id = CATIA_CAMERA_LWIRCAM; break;
    default: optical_camera_id = CATIA_CAMERA_ALL; break;
  }
  aicam_detect_enabled = selected_camera_backend == CAMERA_BACKEND_AI_CAM_DETECT;
  aicam_home_enabled = selected_camera_backend == CAMERA_BACKEND_AI_CAM_HOME;

  if (speedtest_enabled) {
    const char *photo_root = NULL;
    char filename_prefix = '\0';
    switch (selected_camera_backend) {
      case CAMERA_BACKEND_CHDK:
        photo_root = CATIA_CHDK_PHOTO_DIR;
        filename_prefix = 'c';
        break;
      case CAMERA_BACKEND_AI_CAM:
      case CAMERA_BACKEND_AI_CAM_DETECT:
      case CAMERA_BACKEND_AI_CAM_HOME:
        photo_root = CATIA_AI_CAM_PHOTO_DIR;
        filename_prefix = 'a';
        break;
      case CAMERA_BACKEND_LWIR_CAM:
        photo_root = CATIA_LWIR_CAM_PHOTO_DIR;
        filename_prefix = 'l';
        break;
      default:
        return 2;
    }
    char speedtest_directory[PATH_MAX];
    if (camera_speedtest_create_output_directory(photo_root, speedtest_directory,
                                                 sizeof(speedtest_directory)) != 0) {
      fprintf(stderr, "CATIA SPEEDTEST:\tunable to create isolated output directory: %s\n",
              strerror(errno));
      return 1;
    }
    chdk_photo_directory = speedtest_directory;
    ai_cam_pipe_set_photo_directory(speedtest_directory);
    lwir_cam_pipe_set_photo_directory(speedtest_directory);
    struct camera_speedtest_config speedtest = {
      .backend_name = camera.name,
      .photo_directory = speedtest_directory,
      .filename_prefix = filename_prefix,
      .shoot = camera.shoot,
      .keep_running = &keep_running
    };
    int first_image;
    if (camera_speedtest_find_image_block(&speedtest, &first_image) != 0) {
      fprintf(stderr, "CATIA SPEEDTEST:\tunable to reserve an unused image-number block: %s\n",
              strerror(errno));
      return 1;
    }
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    printf("CATIA:\tStarting Camera Speed Test\n");
    printf("CATIA:\tcamera backend: %s\n", camera.name);
    uint64_t initialization_started_us = monotonic_time_us();
    int prepare_result = camera_prepare(optical_camera_id);
    uint64_t initialization_completed_us = monotonic_time_us();
    if (prepare_result != 0 || initialization_started_us == 0
        || initialization_completed_us < initialization_started_us) {
      fprintf(stderr, "CATIA SPEEDTEST:\tcamera initialization failed\n");
      cameras_deinit();
      return 1;
    }
    printf("CATIA SPEEDTEST:\tinitialization: %.3f ms (excluded from capture rate)\n",
           (initialization_completed_us - initialization_started_us) / 1000.0);
    int speedtest_result = camera_speedtest_run(&speedtest, first_image);
    cameras_deinit();
    printf("CATIA:\tSpeed test shutdown\n");
    return speedtest_result == 0 ? 0 : 1;
  }

  // Initialization
  printf("CATIA:\tStarting Camera Application Triggering Image Analysis\n");
  printf("CATIA:\tserial device: %s\n", serial_device);
  printf("CATIA:\tcamera backend: %s\n", camera.name);
  if (debug_enabled) {
    printf("CATIA DEBUG:\tenabled\n");
  }
  if (test_mode) {
    if (mock_image_was_selected) {
      printf("CATIA:\tcamera test mode: mock image %s\n", mock_image);
    } else {
      printf("CATIA:\tcamera test mode: random image from testphotos when available\n");
    }
    printf("CATIA:\tmock attitude transform: %s\n", mock_transform_enabled ? "enabled" : "disabled");
  }
  if (local_mode) {
    printf("CATIA:\tlocal simulator device: %s\n", CATIA_LOCAL_SIM_DEVICE);
  }
  camera_source_image = NULL;
  if (selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode) {
    printf("CATIA:\tmock camera image: %s\n", mock_image);
    camera_source_image = mock_image;
  } else if (test_mode) {
    camera_source_image = mock_image_was_selected ? mock_image : NULL;
  }
  signal(SIGPIPE, SIG_IGN);
  if ((test_mode || selected_camera_backend == CAMERA_BACKEND_LOCAL)
      && camera.init(camera_source_image) != 0) {
    stop_local_serial_bridge();
    return 1;
  }
  local_capture_only = selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode;
  camera_initialized[optical_camera_id] = test_mode || local_capture_only;
  if (earcam_requested) {
    ensure_earcam_active();
  }
  int ret = serial_init(serial_device);
  if (ret < 0) {
    if (earcam_active) {
      ear_cam_pipe_deinit();
    }
    cameras_deinit();
    stop_local_serial_bridge();
    return -1;
  }
  if (serial_tx_start(fd) != 0) {
    fprintf(stderr, "CATIA:\tunable to initialize nonblocking UART output: %s\n", strerror(errno));
    close(fd);
    if (earcam_active) ear_cam_pipe_deinit();
    cameras_deinit();
    stop_local_serial_bridge();
    return 1;
  }
  fc_clock = (struct clock_alignment){0};
  next_clock_probe_us = clock_probe_count = clock_reply_count = clock_rejected_count = 0;
  if (pose_log_start(pose_log_dir) != 0) {
    fprintf(stderr, "CATIA POSE:\tlog unavailable: %s; capture continues\n", strerror(errno));
  }
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  // Initial settings
  is_shooting = 0;
  catia_protocol.status = 0;
  image_idx = 0;
  image_count = 0;
  shooting_count = 0;
  shooting_thread_count = 0;

  puts(CATIA_BUILD_VERSION);
  printf("Started OK\n");
  notify_systemd_ready();
  if (!test_mode && !local_mode && selected_camera_backend == CAMERA_BACKEND_LWIR_CAM) {
    lwir_ready_or_start_recovery();
  }
  if (debug_enabled) {
    next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
    printf("CATIA DEBUG:\twaiting for CATIA camera messages on %s\n", serial_device);
  }

  struct pollfd event_sources[2] = {
    {.fd = fd, .events = POLLIN, .revents = 0},
    {.fd = socket_get_fd(), .events = POLLIN, .revents = 0}
  };

  // MAIN loop
  while (keep_running) {
    send_clock_probe(monotonic_time_us());
    event_sources[0].events = POLLIN | (serial_tx_pending() ? POLLOUT : 0);
    int poll_result;
    do {
      poll_result = poll(event_sources, 2, EVENT_LOOP_TIMEOUT_MS);
    } while (poll_result < 0 && errno == EINTR && keep_running);
    if (poll_result < 0) {
      fprintf(stderr, "CATIA:\tevent poll failed: %s\n", strerror(errno));
      break;
    }
    if ((event_sources[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      fprintf(stderr, "CATIA:\tserial device disconnected\n");
      break;
    }
    if ((event_sources[0].revents & POLLOUT) != 0 && serial_tx_flush() != 0) {
      fprintf(stderr, "CATIA:\tUART output failed: %s\n", strerror(errno));
      break;
    }

    // Drain available serial data in one syscall and process frames in order.
    ssize_t bytes_read = 0;
    if ((event_sources[0].revents & POLLIN) != 0) {
      bytes_read = read(fd, serial_buffer, sizeof(serial_buffer));
        serial_receive_monotonic_us = monotonic_time_us();
    }
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; index++) {
        serial_byte_count++;
        uint8_t parser_errors_before = catia_protocol.error;
        if (debug_enabled && serial_buffer[index] == STX) {
          printf("CATIA DEBUG:\treceived CATIA frame start at serial byte %llu\n",
                 (unsigned long long)serial_byte_count);
        }
        parse_catia(&catia_protocol, serial_buffer[index]);
        if (catia_protocol.error != parser_errors_before) {
          rejected_frame_count++;
          if (debug_enabled) {
            printf("CATIA DEBUG:\trejected CATIA frame at serial byte %llu (total rejected: %llu)\n",
                   (unsigned long long)serial_byte_count,
                   (unsigned long long)rejected_frame_count);
          }
        }
        if (catia_protocol.msg_received) {
          valid_frame_count++;
          if (debug_enabled) {
            printf("CATIA DEBUG:\taccepted CATIA message id %u with %u payload bytes "
                   "(frame %llu)\n",
                   catia_protocol.msg_id, catia_protocol.payload_len,
                   (unsigned long long)valid_frame_count);
          }
          handle_received_message();
        }
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      fprintf(stderr, "CATIA:\tserial read failed: %s\n", strerror(errno));
    }

    if (debug_enabled && time(NULL) >= next_debug_heartbeat) {
      printf("CATIA DEBUG:\twaiting for CATIA data: %llu bytes, %llu valid frames, "
             "%llu rejected frames\n",
             (unsigned long long)serial_byte_count,
             (unsigned long long)valid_frame_count,
             (unsigned long long)rejected_frame_count);
          struct pose_log_stats pose_status = pose_log_status();
          printf("CATIA POSE:\taccepted=%" PRIu64 " synced=%" PRIu64 " dropped=%" PRIu64
             " rejected=%" PRIu64 " error=%d\n", pose_status.accepted, pose_status.synced,
             pose_status.dropped, pose_status.rejected, pose_status.error);
      next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
      if (clock_probes_enabled) {
        printf("CATIA CLOCK:\tprobes=%" PRIu64 " accepted=%" PRIu64 " rejected=%" PRIu64 "\n",
               clock_probe_count, clock_reply_count, clock_rejected_count);
      }
    }

    // Read the socket
    if ((event_sources[1].revents & POLLIN) != 0
      && socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if (image_count < MAX_IMAGE_BUFFERS) {
        image_count++;
      }
    }

  }

  // Close
  serial_tx_stop();
  close(fd);
  pthread_mutex_lock(&mut);
  pthread_cond_broadcast(&camera_available);
  while (shooting_thread_count > 0 || lwir_recovery_in_progress) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  if (earcam_active) {
    ear_cam_pipe_deinit();
  }
  cameras_deinit();
  stop_local_serial_bridge();

    if (pose_log_stop() != 0) fprintf(stderr, "CATIA POSE:\tlog incomplete: %s\n", strerror(errno));
    struct pose_log_stats pose_status = pose_log_status();
    fprintf(stderr, "CATIA POSE:\tfinal accepted=%" PRIu64 " synced=%" PRIu64
      " dropped=%" PRIu64 " rejected=%" PRIu64 " error=%d\n",
      pose_status.accepted, pose_status.synced, pose_status.dropped, pose_status.rejected, pose_status.error);
  printf("CATIA:\tShutdown\n");
  return 0;
}

/** @brief Send READY=1 to systemd when launched with a notify socket.
 * @details Failure is diagnostic only: CATIA also supports direct/manual execution. */
static void notify_systemd_ready(void)
{
  const char *socket_path = getenv("NOTIFY_SOCKET");
  if (socket_path == NULL || socket_path[0] == '\0') {
    return;
  }

  struct sockaddr_un address = {.sun_family = AF_UNIX};
  size_t path_length = strlen(socket_path);
  if (path_length >= sizeof(address.sun_path)) {
    fprintf(stderr, "CATIA:\tsystemd notification socket path is too long\n");
    return;
  }

  for (size_t index = 0; index <= path_length; index++) {
    address.sun_path[index] = socket_path[index];
  }
  bool abstract_socket = address.sun_path[0] == '@';
  if (abstract_socket) {
    address.sun_path[0] = '\0';
  }

  int notify_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (notify_fd < 0) {
    fprintf(stderr, "CATIA:\tfailed to create systemd notification socket: %s\n",
            strerror(errno));
    return;
  }

  static const char ready_message[] = "READY=1\nSTATUS=MORA serial ready; camera selected on shoot command";
  socklen_t address_length = (socklen_t)(offsetof(struct sockaddr_un, sun_path)
                                        + path_length + (abstract_socket ? 0 : 1));
  if (sendto(notify_fd, ready_message, sizeof(ready_message) - 1, MSG_NOSIGNAL,
             (const struct sockaddr *)&address, address_length) < 0) {
    fprintf(stderr, "CATIA:\tfailed to notify systemd that startup completed: %s\n",
            strerror(errno));
  }
  close(notify_fd);
}

static void *handle_msg_shoot(void *ptr)
{
  struct capture_job *job = ptr;

  pthread_mutex_lock(&mut);
  while (job->ticket != next_capture_ticket && keep_running) {
    pthread_cond_wait(&camera_available, &mut);
  }
  if (!keep_running) {
    pthread_mutex_unlock(&mut);
    release_worker_slot();
    free(job);
    return NULL;
  }

  is_shooting = 1;
  shooting_count++;
  pthread_mutex_unlock(&mut);

  /* Only the captures hold the camera slot. EXIF and SODA for optical images run
   * after it is released, so the next shot can capture meanwhile. LWIR is finished
   * inside the slot because its geolocation reads the backend's last-capture state;
   * --aicam-detect has the same hazard (vehicle_detect_pipe's last-detection state is
   * likewise a single static slot, not part of `image`), so it is finished inside too.
   * --aicam-home has the identical hazard (home_vector_pipe's last-result state), and
   * additionally must reach send_msg_home_vector_result() before the next shot can
   * overwrite that slot, so it is finished inside the capture slot as well. */
  const bool separate_lwir = lwir_async_recovery_enabled
      && !test_capture_enabled && !local_capture_only;
  struct captured_image images[CATIA_CAMERA_LWIRCAM];
  int image_count_ready = 0;
  const int ordered_camera_max = separate_lwir ? CATIA_CAMERA_AICAM : CATIA_CAMERA_LWIRCAM;
  for (int camera_id = CATIA_CAMERA_CHDK; camera_id <= ordered_camera_max && keep_running; ++camera_id) {
    if ((job->camera_mask & (1U << (camera_id - 1))) != 0 && camera_prepare(camera_id) == 0) {
      struct captured_image *image = &images[image_count_ready];
      if (!capture_image(&job->shot, image, camera_id, &camera)) continue;
      bool finish_in_slot = image->camera_id == CATIA_CAMERA_LWIRCAM
        || (image->camera_id == CATIA_CAMERA_AICAM && (aicam_detect_enabled || aicam_home_enabled));
      if (finish_in_slot) finish_image(&job->shot, image);
      else ++image_count_ready;
    }
  }

  /** Reserve LWIR before advancing the ordered ticket. A later trigger can then only
   * skip a busy lane, never overtake this trigger and steal its thermal capture. */
  bool lwir_lane_reserved = false;
  if (separate_lwir && keep_running
      && (job->camera_mask & CATIA_CAMERA_MASK_LWIR) != 0) {
    lwir_lane_reserved = pthread_mutex_trylock(&lwir_camera_mutex) == 0;
    if (!lwir_lane_reserved) {
      pthread_mutex_lock(&mut);
      ++lwir_busy_skipped_count;
      pthread_cond_broadcast(&workers_finished);
      pthread_mutex_unlock(&mut);
      if (debug_enabled) {
        printf("CATIA-%d DEBUG:\tprevious LWIR operation still active; shot skipped\n",
               job->shot.data.nr);
      }
    }
  }

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  ++next_capture_ticket;
  pthread_cond_broadcast(&camera_available);
  pthread_mutex_unlock(&mut);
  for (int index = 0; index < image_count_ready; ++index) {
    finish_image(&job->shot, &images[index]);
  }
  if (lwir_lane_reserved) {
    if (lwir_ready_or_start_recovery()) {
      const struct camera_backend lwir_backend = {
        "lwircam", lwir_cam_pipe_init, lwir_cam_pipe_shoot,
        lwir_cam_pipe_deinit, CATIA_SODA
      };
      struct captured_image lwir_image;
      if (capture_image(&job->shot, &lwir_image, CATIA_CAMERA_LWIRCAM, &lwir_backend)) {
        finish_image(&job->shot, &lwir_image);
      }
      lwir_ready_or_start_recovery();
    } else if (debug_enabled) {
      printf("CATIA-%d DEBUG:\tLWIR unavailable or recovery in progress; shot skipped\n",
             job->shot.data.nr);
    }
    pthread_mutex_unlock(&lwir_camera_mutex);
  }
  release_worker_slot();
  free(job);
  return NULL;
}

/** Capture into `image`; true when a file is ready for finish_image(). */
static bool capture_image(const union dc_shot_union *shoot, struct captured_image *image,
                          int camera_id, const struct camera_backend *backend)
{
  char *filename = image->filename;
  filename[0] = '\0';
  image->camera_id = camera_id;
  printf("CATIA-%d:\tShooting: start\n", shoot->data.nr);
  if (debug_enabled) {
    if (test_capture_enabled) {
      printf("CATIA-%d DEBUG:\trequesting test image for selected %s backend\n",
             shoot->data.nr, backend->name);
    } else {
      printf("CATIA-%d DEBUG:\trequesting image from %s backend\n", shoot->data.nr, backend->name);
    }
  }
  if (backend->shoot(filename, sizeof(image->filename), shoot->data.nr) != 0) {
    fprintf(stderr, "CATIA-%d:\t%s camera capture failed\n", shoot->data.nr, backend->name);
    backend->deinit();
    if (camera_id == CATIA_CAMERA_LWIRCAM) lwir_mark_backoff();
    else camera_initialized[camera_id] = false;
    filename[0] = '\0';
  }
  printf("CATIA-%d:\tShooting: got image %s\n", shoot->data.nr, filename);
  if (filename[0] != '\0' && mock_transform_enabled) {
    int transform_result = image_mock_transform(filename, shoot);
    if (transform_result < 0) {
      fprintf(stderr, "CATIA-%d:\tfailed to apply mock attitude transform to %s\n",
              shoot->data.nr, filename);
      filename[0] = '\0';
    } else if (transform_result == IMAGE_MOCK_TRANSFORM_SKIPPED) {
      printf("CATIA-%d:\tShooting: mock attitude transform skipped; source image retained\n",
             shoot->data.nr);
    } else {
      printf("CATIA-%d:\tShooting: mock attitude transform applied\n", shoot->data.nr);
    }
  }
  if (filename[0] != '\0' && test_capture_enabled
      && camera_id == CATIA_CAMERA_LWIRCAM
      && lwir_cam_pipe_process_mock(filename) != 0) {
    fprintf(stderr, "CATIA-%d:\tLWIR mock processing failed\n", shoot->data.nr);
    filename[0] = '\0';
  }
  const bool live_lwir = image->camera_id == CATIA_CAMERA_LWIRCAM && !test_capture_enabled;
  image->capture_delay = live_lwir ? lwir_cam_pipe_capture_delay() : -1;
  image->capture_times = live_lwir ? lwir_cam_pipe_capture_timing() : (struct capture_timing){0};
  return filename[0] != '\0';
}

/** EXIF, LWIR hotspot geolocation and SODA for a captured image. */
static void finish_image(const union dc_shot_union *shoot, struct captured_image *image)
{
  char *filename = image->filename;
  bool image_ready = false;
  {
    if (debug_enabled) {
      printf("CATIA-%d DEBUG:\twriting flight metadata to captured JPEG\n", shoot->data.nr);
    }
    const int compensate = motion_compensation_enabled;
    if (image_exif_write_capture(filename, shoot, image->capture_delay, compensate,
                                 &image->capture_times) == 0) {
      image_ready = true;
      printf("CATIA-%d:\tShooting: EXIF metadata added\n", shoot->data.nr);
      if (image->camera_id == CATIA_CAMERA_LWIRCAM) {
        if (mock_transform_enabled) {
          if (image_exif_write_hotspots(filename, "status=unsupported_mock_transform; coordinates_omitted=true") != 0) {
            fprintf(stderr, "CATIA-%d:\tfailed to record unsupported thermal transform\n", shoot->data.nr);
          }
        } else if (lwir_cam_pipe_geolocate(filename) != 0) {
          fprintf(stderr, "CATIA-%d:\tLWIR hotspot geolocation failed; photo retained\n", shoot->data.nr);
          lwir_mark_backoff();
          if (image_exif_write_hotspots(filename, "status=analysis_failed; coordinates_omitted=true") != 0) {
            fprintf(stderr, "CATIA-%d:\tfailed to record hotspot analysis failure\n", shoot->data.nr);
          }
        }
      } else if (image->camera_id == CATIA_CAMERA_AICAM && aicam_detect_enabled) {
        /** The detection result (if any) was already produced by vehicle_detect_pipe_shoot()
         * as part of this same capture, unlike LWIR's separate post-capture geolocate() pass;
         * just format and record it, mirroring image_exif_write_hotspots()'s convention. */
        char detection_summary[256];
        vehicle_detect_pipe_last_detection_summary(detection_summary, sizeof(detection_summary));
        if (image_exif_write_vehicle_detections(filename, detection_summary) != 0) {
          fprintf(stderr, "CATIA-%d:\tfailed to record vehicle detection result\n", shoot->data.nr);
        } else if (debug_enabled) {
          printf("CATIA-%d DEBUG:\tvehicle detection: %s\n", shoot->data.nr, detection_summary);
        }
        /** Copy after the EXIF write above (not from vehicle_detect_pipe_shoot()) so the
         * duplicate in detections/ carries the same flight + detection metadata as the
         * original, not just the raw frame. No-op when nothing was detected. */
        vehicle_detect_pipe_save_detection_copy(filename);
      } else if (image->camera_id == CATIA_CAMERA_AICAM && aicam_home_enabled) {
        send_msg_home_vector_result();
      }
      printf("Photo take %d\n", shoot->data.nr);
    } else {
      fprintf(stderr, "CATIA-%d:\tfailed to add EXIF metadata to %s\n", shoot->data.nr, filename);
    }
  }

  if (image_ready) {
    int soda_result = run_soda(filename, shoot, image->camera_id);
    printf("CATIA-%d:\tShooting: soda return %d of image %s\n",
           shoot->data.nr, soda_result, filename);
  }

}

static void handle_received_message(void)
{
  catia_protocol.msg_received = false;

  if (catia_protocol.msg_id == CATIA_CLOCK_REPLY) {
    if (clock_probes_enabled) {
      if (clock_alignment_reply(&fc_clock, catia_protocol.payload, catia_protocol.payload_len,
                                  serial_receive_monotonic_us)) ++clock_reply_count;
      else ++clock_rejected_count;
    }
    return;
  }
  if (catia_protocol.msg_id == CATIA_POSE_CLOCKED) {
    record_clocked_pose();
    return;
  }
  if (catia_protocol.msg_id == CATIA_POSE_SAMPLE) {
    fc_clock.valid = false;
    pose_log_record(catia_protocol.payload, catia_protocol.payload_len, serial_receive_monotonic_us);
    return;
  }
  if (catia_protocol.msg_id == CATIA_SHOOT || catia_protocol.msg_id == CATIA_SHOOT_TARGETED
      || catia_protocol.msg_id == CATIA_SHOOT_MASK) {
    bool targeted = catia_protocol.msg_id == CATIA_SHOOT_TARGETED;
    bool masked = catia_protocol.msg_id == CATIA_SHOOT_MASK;
    uint8_t expected = masked ? CATIA_SHOOT_MASK_MSG_SIZE
                             : targeted ? CATIA_SHOOT_TARGETED_MSG_SIZE : CATIA_SHOOT_MSG_SIZE;
    if (debug_enabled) {
      printf("CATIA DEBUG:\tphoto trigger received; decoding shot payload\n");
    }
    if (catia_protocol.payload_len != expected) {
      fprintf(stderr, "CATIA:\tinvalid CATIA shot payload length %u\n", catia_protocol.payload_len);
      return;
    }

    union dc_shot_union shot;
    union dc_shot_union *shoot = &shot;
    for (size_t index = 0; index < CATIA_SHOOT_MSG_SIZE; index++) {
      shoot->bin[index] = catia_protocol.payload[index];
    }
    int32_t camera_id = CATIA_CAMERA_ALL;
    uint32_t camera_mask = CATIA_CAMERA_MASK_ALL;
    if (targeted) {
      union dc_shot_targeted_union targeted_msg;
      for (size_t index = 0; index < CATIA_SHOOT_TARGETED_MSG_SIZE; index++) {
        targeted_msg.bin[index] = catia_protocol.payload[index];
      }
      camera_id = targeted_msg.data.camera_id;
      if (camera_id < 0 || camera_id > 8) {
        fprintf(stderr, "CATIA:\tinvalid camera id %d\n", camera_id);
        return;
      }
      camera_mask = camera_id == CATIA_CAMERA_ALL ? CATIA_CAMERA_MASK_ALL : 1U << (camera_id - 1);
    }
    if (masked) {
      union dc_shot_mask_union message;
      for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = catia_protocol.payload[index];
      camera_mask = message.data.camera_mask;
      if (camera_mask > CATIA_CAMERA_MASK_ALL) {
        fprintf(stderr, "CATIA:\tinvalid camera mask %" PRIu32 "\n", camera_mask);
        return;
      }
    }
    if (camera_mask == CATIA_CAMERA_MASK_NONE) return;
    if ((camera_mask & ~CATIA_CAMERA_MASK_SUPPORTED) != 0) {
      fprintf(stderr, "CATIA:\tunsupported camera bits 0x%02" PRIx32 "; supported cameras continue\n",
              camera_mask & ~CATIA_CAMERA_MASK_SUPPORTED);
    }
    camera_mask &= CATIA_CAMERA_MASK_SUPPORTED;
    printf("CATIA:\tSHOT %d mask 0x%02" PRIx32 " | lat %.7f lon %.7f | MSL %.1f m AGL %.1f m | "
           "roll %.1f pitch %.1f yaw %.1f deg | speed %.1f m/s course %.1f deg\n",
           shoot->data.nr, camera_mask,
           shoot->data.lat / 1e7,
           shoot->data.lon / 1e7,
           shoot->data.alt / 1000.0,
           shoot->data.groundalt / POSITION_BFP_SCALE,
           shoot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.vground / SPEED_BFP_SCALE,
           shoot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG);

    bool wants_ear = (camera_mask & CATIA_CAMERA_MASK_EAR) != 0;
    if (wants_ear) ensure_earcam_active();
    if (wants_ear && earcam_active) {
      // Geotagging is a memory copy; do it inline so no worker slot is consumed.
      if (ear_cam_pipe_record(shoot) == 0 && debug_enabled) {
        printf("CATIA-%d DEBUG:\tearcam sample recorded\n", shoot->data.nr);
      }
    } else if (wants_ear && debug_enabled) {
      printf("CATIA-%d DEBUG:\tearcam requested but not active\n", shoot->data.nr);
    }
    if ((camera_mask & (CATIA_CAMERA_MASK_CHDK | CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR)) != 0) {
      start_shoot_worker(shoot, (uint8_t)camera_mask);
    }
  } else if (catia_protocol.msg_id == CATIA_STOP_TARGETED) {
    if (catia_protocol.payload_len != CATIA_STOP_TARGETED_MSG_SIZE) {
      fprintf(stderr, "CATIA:\tinvalid CATIA stop payload length %u\n", catia_protocol.payload_len);
      return;
    }
    int32_t camera_id = 0;
    for (size_t index = 0; index < CATIA_STOP_TARGETED_MSG_SIZE; index++) {
      camera_id |= (int32_t)catia_protocol.payload[index] << (8 * index);
    }
    handle_targeted_stop(camera_id);
  } else if (catia_protocol.msg_id == CATIA_BUFFER_EMPTY) {
    send_msg_image_buffer();
  }
}

/** Start EARcam when available without making an optional microphone a CATIA dependency. */
static bool ensure_earcam_active(void)
{
  if (earcam_active) return true;
  if (earcam_unavailable) return false;
  if (ear_cam_pipe_init(NULL) != 0) {
    earcam_unavailable = true;
    fprintf(stderr, "CATIA:\tEARcam not available; AIcam and LWIRcam remain active\n");
    return false;
  }
  earcam_active = true;
  printf("CATIA:\tacoustic backend: earcam\n");
  return true;
}

static int camera_prepare(int camera_id)
{
  enum camera_backend_type type;
  switch (camera_id) {
    case CATIA_CAMERA_ALL: type = CAMERA_BACKEND_UNSELECTED; break;
    case CATIA_CAMERA_CHDK: type = CAMERA_BACKEND_CHDK; break;
    case CATIA_CAMERA_AICAM: type = CAMERA_BACKEND_AI_CAM; break;
    case CATIA_CAMERA_LWIRCAM: type = CAMERA_BACKEND_LWIR_CAM; break;
    default:
      fprintf(stderr, "CATIA:\tinvalid camera id %d\n", camera_id);
      return -1;
  }
  if (!keep_running) return -1;
  if (camera_id == CATIA_CAMERA_AICAM && aicam_detect_enabled) type = CAMERA_BACKEND_AI_CAM_DETECT;
  if (camera_id == CATIA_CAMERA_AICAM && aicam_home_enabled) type = CAMERA_BACKEND_AI_CAM_HOME;
  if (local_capture_only) type = CAMERA_BACKEND_LOCAL;
  if (camera_backend_select(type, test_capture_enabled) != 0) return -1;
  optical_camera_id = camera_id;
  if (camera_initialized[camera_id]) return 0;
  if (camera_id == CATIA_CAMERA_LWIRCAM && camera_unavailable[camera_id]) {
    uint64_t now_us = monotonic_time_us();
    if (now_us == 0 || now_us < lwir_retry_after_us) return -1;
    camera_unavailable[camera_id] = false;
    printf("CATIA:\tretrying LWIR camera after unavailable cooldown\n");
  }
  if (camera.init(camera_source_image) != 0) {
    camera.deinit();
    if (camera_id == CATIA_CAMERA_LWIRCAM) lwir_mark_backoff();
    if (!camera_warned[camera_id]) {
      camera_warned[camera_id] = true;
      fprintf(stderr, "CATIA:\t%s (camera id %d) not available; continuing without it\n",
              camera.name, camera_id);
    } else if (debug_enabled) {
      printf("CATIA DEBUG:\t%s (camera id %d) still not available\n", camera.name, camera_id);
    }
    return -1;
  }
  camera_warned[camera_id] = false;
  if (camera_id == CATIA_CAMERA_LWIRCAM) {
    camera_unavailable[camera_id] = false;
    lwir_retry_after_us = 0;
  }
  camera_initialized[camera_id] = true;
  printf("CATIA:\tactive camera backend: %s (id %d)\n", camera.name, optical_camera_id);
  return 0;
}

static void lwir_mark_backoff(void)
{
  uint64_t now_us = monotonic_time_us();
  pthread_mutex_lock(&mut);
  camera_initialized[CATIA_CAMERA_LWIRCAM] = false;
  camera_unavailable[CATIA_CAMERA_LWIRCAM] = true;
  lwir_retry_after_us = now_us == 0 || now_us > UINT64_MAX - LWIR_RETRY_COOLDOWN_US
                        ? UINT64_MAX : now_us + LWIR_RETRY_COOLDOWN_US;
  pthread_mutex_unlock(&mut);
}

static void *recover_lwir_camera(void *unused)
{
  (void)unused;
  bool retrying = false;
  while (keep_running) {
    pthread_mutex_lock(&mut);
    uint64_t now_us = monotonic_time_us();
    bool cooling_down = camera_unavailable[CATIA_CAMERA_LWIRCAM]
                        && (now_us == 0 || now_us < lwir_retry_after_us);
    pthread_mutex_unlock(&mut);
    if (cooling_down) {
      usleep(LWIR_RETRY_POLL_US);
      continue;
    }
    if (retrying) printf("CATIA:\tretrying LWIR camera after unavailable cooldown\n");

    pthread_mutex_lock(&lwir_camera_mutex);
    if (!keep_running) {
      pthread_mutex_unlock(&lwir_camera_mutex);
      break;
    }
    int result = lwir_cam_pipe_init_cancelable(NULL, &keep_running);
    if (result == 0 && !keep_running) {
      lwir_cam_pipe_deinit();
      result = -1;
    }
    pthread_mutex_unlock(&lwir_camera_mutex);

    pthread_mutex_lock(&mut);
    if (result == 0) {
      camera_initialized[CATIA_CAMERA_LWIRCAM] = true;
      camera_unavailable[CATIA_CAMERA_LWIRCAM] = false;
      camera_warned[CATIA_CAMERA_LWIRCAM] = false;
      lwir_retry_after_us = 0;
      lwir_recovery_in_progress = false;
      printf("CATIA:\tactive camera backend: lwircam (id %d)\n", CATIA_CAMERA_LWIRCAM);
      pthread_cond_broadcast(&workers_finished);
      pthread_mutex_unlock(&mut);
      return NULL;
    }
    camera_initialized[CATIA_CAMERA_LWIRCAM] = false;
    camera_unavailable[CATIA_CAMERA_LWIRCAM] = true;
    now_us = monotonic_time_us();
    lwir_retry_after_us = now_us == 0 || now_us > UINT64_MAX - LWIR_RETRY_COOLDOWN_US
                          ? UINT64_MAX : now_us + LWIR_RETRY_COOLDOWN_US;
    if (!camera_warned[CATIA_CAMERA_LWIRCAM]) {
      camera_warned[CATIA_CAMERA_LWIRCAM] = true;
      fprintf(stderr, "CATIA:\tlwircam (camera id %d) not available; AIcam remains active\n",
              CATIA_CAMERA_LWIRCAM);
    }
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    retrying = true;
  }

  pthread_mutex_lock(&mut);
  lwir_recovery_in_progress = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  return NULL;
}

static bool lwir_ready_or_start_recovery(void)
{
  pthread_mutex_lock(&mut);
  if (camera_initialized[CATIA_CAMERA_LWIRCAM]) {
    pthread_mutex_unlock(&mut);
    return true;
  }
  if (!keep_running || lwir_recovery_in_progress) {
    pthread_mutex_unlock(&mut);
    return false;
  }
  lwir_recovery_in_progress = true;
  pthread_mutex_unlock(&mut);

  pthread_t recovery_thread;
  int thread_result = pthread_create(&recovery_thread, NULL, recover_lwir_camera, NULL);
  if (thread_result != 0) {
    pthread_mutex_lock(&mut);
    lwir_recovery_in_progress = false;
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "CATIA:\tfailed to start LWIR recovery: %s\n", strerror(thread_result));
    lwir_mark_backoff();
    return false;
  }
  pthread_detach(recovery_thread);
  return false;
}

static void cameras_deinit(void)
{
  for (int camera_id = 0; camera_id <= CATIA_CAMERA_LWIRCAM; ++camera_id) {
    if (!camera_initialized[camera_id]) continue;
    enum camera_backend_type type = CAMERA_BACKEND_LOCAL;
    if (!local_capture_only) {
      switch (camera_id) {
        case CATIA_CAMERA_CHDK: type = CAMERA_BACKEND_CHDK; break;
        case CATIA_CAMERA_AICAM: type = CAMERA_BACKEND_AI_CAM; break;
        case CATIA_CAMERA_LWIRCAM: type = CAMERA_BACKEND_LWIR_CAM; break;
      }
      if (camera_id == CATIA_CAMERA_AICAM && aicam_detect_enabled) type = CAMERA_BACKEND_AI_CAM_DETECT;
      if (camera_id == CATIA_CAMERA_AICAM && aicam_home_enabled) type = CAMERA_BACKEND_AI_CAM_HOME;
    }
    if (camera_backend_select(type, test_capture_enabled) == 0) camera.deinit();
    camera_initialized[camera_id] = false;
  }
}

static void start_shoot_worker(const union dc_shot_union *shoot, uint8_t camera_mask)
{
  struct capture_job *job = malloc(sizeof(*job));
  if (job == NULL) {
    fprintf(stderr, "CATIA:\tfailed to allocate capture job\n");
    return;
  }
  job->shot = *shoot;
  job->camera_mask = camera_mask;
  pthread_mutex_lock(&mut);
  if (shooting_thread_count >= MAX_PROCESSING_THREADS) {
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "CATIA-%d:\tprocessing queue is full\n", shoot->data.nr);
    free(job);
    return;
  }
  shooting_thread_count++;
  job->ticket = next_job_ticket;

  pthread_t shooting_thread;
  int thread_result = pthread_create(&shooting_thread, NULL, handle_msg_shoot, job);
  if (thread_result != 0) {
    shooting_thread_count--;
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "CATIA-%d:\tfailed to start shooting thread: %s\n",
            shoot->data.nr, strerror(thread_result));
    free(job);
    return;
  }
  ++next_job_ticket;
  pthread_mutex_unlock(&mut);
  pthread_detach(shooting_thread);
  send_msg_status();
}

static void handle_targeted_stop(int32_t camera_id)
{
  bool keep = (camera_id & CATIA_STOP_FLAG_KEEP) != 0;
  camera_id &= 0xFF;
  printf("CATIA:\tSTOP cam %d%s\n", camera_id, keep ? " (interim)" : "");
  if (camera_id != CATIA_CAMERA_ALL && camera_id != CATIA_CAMERA_EARCAM) {
    return;
  }
  if (!earcam_active) {
    if (camera_id == CATIA_CAMERA_EARCAM) {
      struct ear_loudest_spot none = {0};
      send_msg_ear_result(&none);
    }
    return;
  }
  struct ear_loudest_spot spot;
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);
  if (keep) {
    ear_cam_pipe_solve(&spot);
  } else {
    // Final stop: render the acoustic "photo" from the whole session before it is cleared.
    ear_cam_pipe_solve(&spot);
    char sound_picture[MAX_FILENAME];
    if (ear_cam_pipe_render(&spot, sound_picture, sizeof(sound_picture), earcam_simulated) == 0) {
      union dc_shot_union picture_shot;
      memset(&picture_shot, 0, sizeof(picture_shot));
      picture_shot.data.nr = ear_cam_pipe_last_shot_nr();
      if (spot.valid) {
        picture_shot.data.lat = (int32_t)llround(spot.lat_deg * 1e7);
        picture_shot.data.lon = (int32_t)llround(spot.lon_deg * 1e7);
        picture_shot.data.alt = (int32_t)llround(spot.alt_m * 1000.0);
        picture_shot.data.groundalt = (int32_t)llround(spot.agl_m * POSITION_BFP_SCALE);
      }
      if (image_exif_write(sound_picture, &picture_shot) == 0) {
        printf("CATIA:\tEAR sound picture %s\n", sound_picture);
        printf("Photo take %d\n", picture_shot.data.nr);
        int soda_result = run_soda(sound_picture, &picture_shot, CATIA_CAMERA_EARCAM);
        printf("CATIA-%d:\tShooting: soda return %d of image %s\n",
               picture_shot.data.nr, soda_result, sound_picture);
      } else {
        fprintf(stderr, "CATIA:\tfailed to add EXIF metadata to %s\n", sound_picture);
      }
    }
    ear_cam_pipe_finish(&spot);
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  double elapsed_ms = (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) / 1e6;
  if (spot.valid) {
    printf("CATIA:\tEAR loudest spot lat %.7f lon %.7f agl %.1f m alt %.1f m | %.1f dB "
           "conf %.2f | %u/%u samples in %.2f ms\n",
           spot.lat_deg, spot.lon_deg, spot.agl_m, spot.alt_m, spot.level_db,
           spot.confidence, spot.used_count, spot.sample_count, elapsed_ms);
  } else {
    printf("CATIA:\tEAR loudest spot not determined (%u samples, %.2f ms)\n",
           spot.sample_count, elapsed_ms);
  }
  send_msg_ear_result(&spot);
}

static void send_msg_ear_result(const struct ear_loudest_spot *spot)
{
  union catia_ear_result_union result;
  memset(&result, 0, sizeof(result));
  result.data.status = spot->valid ? CATIA_EAR_RESULT_VALID : CATIA_EAR_RESULT_INVALID;
  if (spot->valid) {
    result.data.lat = (int32_t)llround(spot->lat_deg * 1e7);
    result.data.lon = (int32_t)llround(spot->lon_deg * 1e7);
    result.data.agl_mm = (int32_t)llround(spot->agl_m * 1000.0);
    result.data.alt_mm = (int32_t)llround(spot->alt_m * 1000.0);
    result.data.level_cdb = (int32_t)llround(spot->level_db * 100.0);
    result.data.confidence = (int32_t)llround(spot->confidence * 1000.0);
  }
  result.data.sample_count = (int32_t)spot->sample_count;

  if (serial_tx_send(CATIA_EAR_RESULT, result.bin, sizeof(result.bin)) != 0) {
    fprintf(stderr, "CATIA:\tEAR result UART send failed: %s\n", strerror(errno));
  }
}

/** Unlike send_msg_ear_result() (sent once per explicit solve), this runs after every
 * --aicam-home shot: the visual-homing CNN predicts a fresh body-frame direction on every
 * frame, and the flight controller's HOME waypoint should track it continuously. */
static void send_msg_home_vector_result(void)
{
  float dx = 0.f, dy = 0.f, dist = 0.f;
  bool valid = home_vector_pipe_last_result(&dx, &dy, &dist);

  union catia_home_vector_result_union result;
  memset(&result, 0, sizeof(result));
  result.data.status = valid ? CATIA_HOME_VECTOR_VALID : CATIA_HOME_VECTOR_INVALID;
  if (valid) {
    result.data.dx_scaled = (int32_t)llround((double)dx * 10000.0);
    result.data.dy_scaled = (int32_t)llround((double)dy * 10000.0);
    result.data.dist_mm = (int32_t)llround((double)dist * 1000.0);
  }

  if (serial_tx_send(CATIA_HOME_VECTOR_RESULT, result.bin, sizeof(result.bin)) != 0) {
    fprintf(stderr, "CATIA:\thome vector result UART send failed: %s\n", strerror(errno));
  }
}

static int run_soda(const char *filename, const union dc_shot_union *shoot, int camera_id)
{
  const char *soda_application = camera_id == CATIA_CAMERA_CHDK && !test_capture_enabled ? SODA : CATIA_SODA;
  const char *camera_option = NULL;
  switch (camera_id) {
    case CATIA_CAMERA_ALL: break;
    case CATIA_CAMERA_CHDK: camera_option = "--chdkcam"; break;
    case CATIA_CAMERA_AICAM: camera_option = "--aicam"; break;
    case CATIA_CAMERA_LWIRCAM: camera_option = "--lwircam"; break;
    case CATIA_CAMERA_EARCAM: camera_option = "--earcam"; break;
    default:
      fprintf(stderr, "CATIA:\tinvalid SODA camera id %d\n", camera_id);
      return -1;
  }
  char values[10][16];
  const int32_t fields[10] = {
    shoot->data.nr, shoot->data.lat, shoot->data.lon, shoot->data.alt,
    shoot->data.phi, shoot->data.theta, shoot->data.psi, shoot->data.vground,
    shoot->data.course, shoot->data.groundalt
  };
  for (size_t index = 0; index < 10; index++) {
    int length = snprintf(values[index], sizeof(values[index]), "%d", fields[index]);
    if (length < 0 || (size_t)length >= sizeof(values[index])) {
      return -1;
    }
  }

  char *arguments[] = {
    (char *)soda_application, (char *)filename,
    values[0], values[1], values[2], values[3], values[4],
    values[5], values[6], values[7], values[8], values[9], NULL, NULL, NULL
  };
  size_t argument_count = 12;
  if (camera_option != NULL) arguments[argument_count++] = (char *)camera_option;
  if (local_mode) arguments[argument_count++] = (char *)"--local";
  arguments[argument_count] = NULL;
  if (debug_enabled) {
    printf("CATIA-%d DEBUG:\tstarting SODA directly: %s\n",
           shoot->data.nr, soda_application);
  }

  pid_t soda_pid;
  int spawn_result = posix_spawnp(&soda_pid, soda_application, NULL, NULL,
                                  arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "CATIA-%d:\tfailed to start SODA: %s\n",
            shoot->data.nr, strerror(spawn_result));
    return -1;
  }

  int status = 0;
  int shutdown_wait_ms = 0;
  for (;;) {
    pid_t wait_result = waitpid(soda_pid, &status, WNOHANG);
    if (wait_result == soda_pid) {
      break;
    }
    if (wait_result < 0 && errno != EINTR) {
      return -1;
    }
    if (!keep_running) {
      if (shutdown_wait_ms == 0) {
        kill(soda_pid, SIGTERM);
      } else if (shutdown_wait_ms >= SODA_SHUTDOWN_GRACE_MS) {
        kill(soda_pid, SIGKILL);
      }
      shutdown_wait_ms += SODA_WAIT_INTERVAL_US / 1000;
    }
    usleep(SODA_WAIT_INTERVAL_US);
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static void release_worker_slot(void)
{
  pthread_mutex_lock(&mut);
  shooting_thread_count--;
  if (shooting_thread_count == 0) {
    pthread_cond_broadcast(&workers_finished);
  }
  pthread_mutex_unlock(&mut);
}

static inline void send_msg_image_buffer(void)
{
  // Check if image is available
  if (image_count > 0) {
    printf("CATIA:\thandle_msg_buffer: Send %d\n", image_idx);
    // Send the image
    image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
    image_count--;

    if (serial_tx_send(CATIA_PAYLOAD, (const uint8_t *)image_buffer[image_idx], IMAGE_SIZE) != 0) {
      fprintf(stderr, "CATIA:\timage UART send failed: %s\n", strerror(errno));
    }
  }
}

static inline void send_msg_status(void)
{
  struct catia_status_struct status_msg;

  pthread_mutex_lock(&mut);
  status_msg.cpu = 0;
  status_msg.threads = shooting_thread_count;
  status_msg.shots = shooting_count;
  status_msg.extra = 0;
  pthread_mutex_unlock(&mut);

  if (serial_tx_send(CATIA_STATUS, (const uint8_t *)&status_msg, sizeof(status_msg)) != 0) {
    fprintf(stderr, "CATIA:\tstatus UART send failed: %s\n", strerror(errno));
  }
}

static void print_usage(const char *program)
{
  puts(CATIA_BUILD_VERSION);
  printf("Usage: %s [--serial DEVICE | --local] [--chdk | --aicam | --aicam-detect | --aicam-home | --lwircam] [--earcam] [--test] [OPTIONS]\n", program);
  printf("       %s (--chdk | --aicam | --aicam-detect | --aicam-home | --lwircam) --speedtest\n", program);
  printf("  --serial DEVICE   serial endpoint (default: %s)\n", CATIA_SERIAL_DEVICE);
  printf("  --local           create local serial bridge %s <-> %s\n",
         CATIA_LOCAL_SIM_DEVICE, CATIA_LOCAL_APP_DEVICE);
  printf("  --chdk            use the CHDK camera backend (default outside local mode)\n");
  printf("  --aicam           use the AI camera backend (plain capture, no detection)\n");
  printf("  --aicam-detect    use the AI camera backend with on-sensor vehicle detection (camera id 2)\n");
  printf("  --aicam-home      use the AI camera backend with on-sensor visual-homing inference (camera id 2)\n");
  printf("  --lwircam         use the Tiny 1-C LWIR camera backend\n");
  printf("  --earcam          also run the acoustic earcam backend (camera id 4)\n");
  printf("  --earcam-sim LAT,LON[,DB]  earcam backend with a virtual loudspeaker instead of a microphone\n");
  printf("  --earcam-band LOW,HIGH  tone search band in Hz (default earcam 1800,3200; motionSCOUT K-T-R: 2400,3200)\n");
  printf("  --test            process a mock image for any camera backend\n");
  printf("                    randomly selects testphotos/*.jpg beside this executable\n");
  printf("  --mocktransform   transform test image using shot roll, pitch, and yaw\n");
  printf("  --debug           show serial, CATIA frame, trigger, and capture diagnostics\n");
  printf("  --mock-image FILE image used by local or test capture (default: %s)\n", CATIA_MOCK_IMAGE);
  printf("  --pose-log DIR    record flight pose samples as CSV in DIR (default: off)\n");
  printf("  --clock-align     send clock probes to bound FC-to-MORA time offset (default: off)\n");
  printf("  --lwir-calibration FILE  camera YAML used to turn LWIR hotspots into coordinates\n");
  printf("  --lwir-raw        keep the full sensor frame as photos/lNNNNNN.jpg.raw instead of\n");
  printf("                    storing the temperatures inside the JPEG\n");
  printf("  --lwir-motion-compensation  advance LWIR GPS over the measured capture delay\n");
    printf("  --speedtest       capture one warm-up plus %d timed photos, report photos/s and time/photo, and exit\n",
      CAMERA_SPEEDTEST_SAMPLE_COUNT);
  printf("  --help            show this help\n");
  printf("  --version         show application version and build Git revision\n");
}

static int camera_backend_select(enum camera_backend_type type, bool test_mode)
{
  switch (type) {
    case CAMERA_BACKEND_CHDK:
      camera = (struct camera_backend) {
        "chdk", chdk_backend_init, chdk_backend_shoot, chdk_pipe_deinit, SODA
      };
      break;
    case CAMERA_BACKEND_LOCAL:
      camera = (struct camera_backend) {
        "local", local_pipe_init, local_backend_shoot, local_pipe_deinit, CATIA_SODA
      };
      break;
    case CAMERA_BACKEND_AI_CAM:
      camera = (struct camera_backend) {
        "aicam", ai_cam_pipe_init, ai_cam_pipe_shoot, ai_cam_pipe_deinit, CATIA_SODA
      };
      break;
    case CAMERA_BACKEND_AI_CAM_DETECT:
      camera = (struct camera_backend) {
        "aicam-detect", vehicle_detect_pipe_init, vehicle_detect_pipe_shoot,
        vehicle_detect_pipe_deinit, CATIA_SODA
      };
      break;
    case CAMERA_BACKEND_AI_CAM_HOME:
      camera = (struct camera_backend) {
        "aicam-home", home_vector_pipe_init, home_vector_pipe_shoot,
        home_vector_pipe_deinit, CATIA_SODA
      };
      break;
    case CAMERA_BACKEND_LWIR_CAM:
      camera = (struct camera_backend) {
        "lwircam", lwir_cam_pipe_init, lwir_cam_pipe_shoot,
        lwir_cam_pipe_deinit, CATIA_SODA
      };
      break;
    case CAMERA_BACKEND_UNSELECTED:
      fprintf(stderr, "CATIA:\tinvalid camera backend\n");
      return -1;
  }

  if (test_mode) {
    camera.init = local_pipe_test_init;
    camera.shoot = local_backend_shoot;
    camera.deinit = local_pipe_deinit;
    camera.soda_application = CATIA_SODA;
  }
  return 0;
}

static int local_backend_shoot(char *filename, size_t filename_size, int image_number)
{
  static const char camera_suffixes[] = {0, 'c', 'a', 'l', 'e'};
  if (optical_camera_id < CATIA_CAMERA_CHDK || optical_camera_id > CATIA_CAMERA_EARCAM) {
    return -1;
  }
  return local_pipe_shoot(filename, filename_size, image_number, camera_suffixes[optical_camera_id]);
}

static int chdk_backend_init(const char *source_image)
{
  (void)source_image;
  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(chdk_photo_directory, resolved_dir, sizeof(resolved_dir));

  if (catia_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "CATIA:\tfailed to create CHDK photo directory %s: %s\n",
            photo_dir, strerror(errno));
    return -1;
  }
  struct stat directory_status;
  if (stat(photo_dir, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(photo_dir, W_OK) != 0) {
    fprintf(stderr, "CATIA:\tCHDK photo directory is not writable: %s\n",
            photo_dir);
    return -1;
  }
  return chdk_pipe_init();
}

static int chdk_backend_shoot(char *filename, size_t filename_size, int image_number)
{
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char downloaded_filename[MAX_FILENAME];
  chdk_pipe_shoot(downloaded_filename);
  if (downloaded_filename[0] == '\0') {
    filename[0] = '\0';
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = catia_resolve_path(chdk_photo_directory, resolved_dir, sizeof(resolved_dir));

  int length = snprintf(filename, filename_size, "%s/c%06d.jpg",
                        photo_dir, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }
  if (unlink(filename) != 0 && errno != ENOENT) {
    fprintf(stderr, "CATIA:\tfailed to replace CHDK image %s: %s\n",
            filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }
  if (move_file(downloaded_filename, filename) != 0) {
    fprintf(stderr, "CATIA:\tfailed to move CHDK image %s to %s: %s\n",
            downloaded_filename, filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }
  return 0;
}

static int move_file(const char *source, const char *destination)
{
  if (rename(source, destination) == 0) {
    return 0;
  }
  if (errno != EXDEV) {
    return -1;
  }

  int source_fd = open(source, O_RDONLY);
  if (source_fd < 0) {
    return -1;
  }
  int destination_fd = open(destination, O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (destination_fd < 0) {
    close(source_fd);
    return -1;
  }

  char buffer[4096];
  int result = 0;
  ssize_t bytes_read;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t bytes_written = 0;
    while (bytes_written < bytes_read) {
      ssize_t count = write(destination_fd, &buffer[bytes_written],
                            (size_t)(bytes_read - bytes_written));
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        result = -1;
        break;
      }
      bytes_written += count;
    }
    if (result != 0) {
      break;
    }
  }
  if (bytes_read < 0) {
    result = -1;
  }
  int saved_errno = errno;
  if (close(source_fd) != 0) {
    result = -1;
    saved_errno = errno;
  }
  if (close(destination_fd) != 0) {
    result = -1;
    saved_errno = errno;
  }
  if (result == 0 && unlink(source) != 0) {
    result = -1;
    saved_errno = errno;
  }
  if (result != 0) {
    unlink(destination);
    errno = saved_errno;
  }
  return result;
}

static int lock_local_instance(void)
{
  local_lock_fd = open(CATIA_LOCAL_LOCK_FILE, O_RDWR | O_CREAT, 0644);
  if (local_lock_fd < 0) {
    fprintf(stderr, "CATIA:\tfailed to open local instance lock %s: %s\n",
            CATIA_LOCAL_LOCK_FILE, strerror(errno));
    return -1;
  }
  if (flock(local_lock_fd, LOCK_EX | LOCK_NB) != 0) {
    fprintf(stderr, "CATIA:\tlocal mode is already running; stop it before starting another instance\n");
    close(local_lock_fd);
    local_lock_fd = -1;
    return -1;
  }
  return 0;
}

static pid_t start_local_serial_bridge(void)
{
  unlink(CATIA_LOCAL_SIM_DEVICE);
  unlink(CATIA_LOCAL_APP_DEVICE);

  pid_t child = fork();
  if (child < 0) {
    perror("CATIA: failed to fork socat");
    return -1;
  }
  if (child == 0) {
    char sim_endpoint[256];
    char app_endpoint[256];
    snprintf(sim_endpoint, sizeof(sim_endpoint), "pty,raw,echo=0,link=%s", CATIA_LOCAL_SIM_DEVICE);
    snprintf(app_endpoint, sizeof(app_endpoint), "pty,raw,echo=0,link=%s", CATIA_LOCAL_APP_DEVICE);
    execlp("socat", "socat", sim_endpoint, app_endpoint, (char *)NULL);
    perror("CATIA: failed to start socat");
    _exit(127);
  }

  if (wait_for_path(CATIA_LOCAL_SIM_DEVICE) != 0 || wait_for_path(CATIA_LOCAL_APP_DEVICE) != 0) {
    fprintf(stderr, "CATIA:\tsocat did not create the local serial endpoints\n");
    kill(child, SIGTERM);
    waitpid(child, NULL, 0);
    unlink(CATIA_LOCAL_SIM_DEVICE);
    unlink(CATIA_LOCAL_APP_DEVICE);
    return -1;
  }
  return child;
}

static int wait_for_path(const char *path)
{
  for (int attempt = 0; attempt < 100; attempt++) {
    if (access(path, F_OK) == 0) {
      return 0;
    }
    usleep(10000);
  }
  return -1;
}

static void stop_local_serial_bridge(void)
{
  if (local_bridge_owned && socat_pid > 0) {
    kill(socat_pid, SIGTERM);
    waitpid(socat_pid, NULL, 0);
    socat_pid = -1;
  }
  if (local_bridge_owned) {
    unlink(CATIA_LOCAL_SIM_DEVICE);
    unlink(CATIA_LOCAL_APP_DEVICE);
    local_bridge_owned = false;
  }
  if (local_lock_fd >= 0) {
    close(local_lock_fd);
    local_lock_fd = -1;
  }
}

static void handle_signal(int signal_number __attribute__((unused)))
{
  keep_running = 0;
}
