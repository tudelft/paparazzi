#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "ai_cam_pipe.h"
#include "ear_cam_pipe.h"
#include "lwir_cam_pipe.h"
#include "serial.h"
#include "chdk_pipe.h"
#include "image_exif.h"
#include "image_mock_transform.h"
#include "local_pipe.h"
#include "std.h"
#include "protocol.h"
#include "socket.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 8
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
// Search&Rescue Onboard Detection Application
#define SODA "./soda_local"
#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define DEBUG_HEARTBEAT_SECONDS 5
#define SERIAL_READ_BUFFER_SIZE 256
#define EVENT_LOOP_TIMEOUT_MS 1000
#define SODA_SHUTDOWN_GRACE_MS 2000
#define SODA_WAIT_INTERVAL_US 10000

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

#ifndef CATIA_LOCAL_SODA
#define CATIA_LOCAL_SODA "soda_local"
#endif

enum camera_backend_type {
  CAMERA_BACKEND_UNSELECTED,
  CAMERA_BACKEND_CHDK,
  CAMERA_BACKEND_LOCAL,
  CAMERA_BACKEND_AI_CAM,
  CAMERA_BACKEND_LWIR_CAM
};

struct camera_backend {
  const char *name;
  int (*init)(const char *source_image);
  int (*shoot)(char *filename, size_t filename_size, int image_number);
  void (*deinit)(void);
  const char *soda_application;
};

static void *handle_msg_shoot(void *ptr);
static void handle_received_message(void);
static void start_shoot_worker(union dc_shot_union *shoot);
static void handle_targeted_stop(int32_t camera_id);
static void send_msg_ear_result(const struct ear_loudest_spot *spot);
static int run_soda(const char *filename, const union dc_shot_union *shoot);
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
static int chdk_backend_init(const char *source_image);
static int chdk_backend_shoot(char *filename, size_t filename_size, int image_number);

static volatile int is_shooting, image_idx, image_count, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t camera_available = PTHREAD_COND_INITIALIZER;
static pthread_cond_t workers_finished = PTHREAD_COND_INITIALIZER;
static bool local_mode;
static enum camera_backend_type requested_camera_backend = CAMERA_BACKEND_UNSELECTED;
static struct camera_backend camera;
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
static int optical_camera_id = MORA_CAMERA_ALL;
static pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;

extern char **environ;

int main(int argc, char *argv[])
{
  const char *serial_device = CATIA_SERIAL_DEVICE;
  bool serial_was_selected = false;
  bool mock_image_was_selected = false;
  bool test_mode = false;
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
    {"lwircam", no_argument, NULL, 'w'},
    {"earcam", no_argument, NULL, 'e'},
    {"earcam-sim", required_argument, NULL, 'E'},
    {"earcam-band", required_argument, NULL, 'B'},
    {"test", no_argument, NULL, 't'},
    {"mocktransform", no_argument, NULL, 'f'},
    {"debug", no_argument, NULL, 'd'},
    {"mock-image", required_argument, NULL, 'i'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };

  while ((option = getopt_long(argc, argv, "s:lcaweE:B:tfdi:h", long_options, NULL)) != -1) {
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
      case 'w':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_LWIR_CAM;
        break;
      case 'e':
        earcam_requested = true;
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

  if (debug_enabled) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IOLBF, 0);
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
  const char *camera_source_image = NULL;
  if (selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode) {
    printf("CATIA:\tmock camera image: %s\n", mock_image);
    camera_source_image = mock_image;
  } else if (test_mode) {
    camera_source_image = mock_image_was_selected ? mock_image : NULL;
  }
  signal(SIGPIPE, SIG_IGN);
  if (camera.init(camera_source_image) != 0) {
    stop_local_serial_bridge();
    return 1;
  }
  switch (selected_camera_backend) {
    case CAMERA_BACKEND_CHDK: optical_camera_id = MORA_CAMERA_CHDK; break;
    case CAMERA_BACKEND_AI_CAM: optical_camera_id = MORA_CAMERA_AICAM; break;
    case CAMERA_BACKEND_LWIR_CAM: optical_camera_id = MORA_CAMERA_LWIRCAM; break;
    default: optical_camera_id = MORA_CAMERA_ALL; break;
  }
  if (earcam_requested) {
    if (ear_cam_pipe_init(NULL) != 0) {
      camera.deinit();
      stop_local_serial_bridge();
      return 1;
    }
    earcam_active = true;
    printf("CATIA:\tacoustic backend: earcam\n");
  }
  int ret = serial_init(serial_device);
  if (ret < 0) {
    if (earcam_active) {
      ear_cam_pipe_deinit();
    }
    camera.deinit();
    stop_local_serial_bridge();
    return -1;
  }
  socket_init(1);
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  // Initial settings
  is_shooting = 0;
  mora_protocol.status = 0;
  image_idx = 0;
  image_count = 0;
  shooting_count = 0;
  shooting_thread_count = 0;

  printf("Started OK\n");
  notify_systemd_ready();
  if (debug_enabled) {
    next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
    printf("CATIA DEBUG:\twaiting for MORA camera messages on %s\n", serial_device);
  }

  struct pollfd event_sources[2] = {
    {.fd = fd, .events = POLLIN, .revents = 0},
    {.fd = socket_get_fd(), .events = POLLIN, .revents = 0}
  };

  // MAIN loop
  while (keep_running) {
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

    // Drain available serial data in one syscall and process frames in order.
    ssize_t bytes_read = 0;
    if ((event_sources[0].revents & POLLIN) != 0) {
      bytes_read = read(fd, serial_buffer, sizeof(serial_buffer));
    }
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; index++) {
        serial_byte_count++;
        uint8_t parser_errors_before = mora_protocol.error;
        if (debug_enabled && serial_buffer[index] == STX) {
          printf("CATIA DEBUG:\treceived MORA frame start at serial byte %llu\n",
                 (unsigned long long)serial_byte_count);
        }
        parse_mora(&mora_protocol, serial_buffer[index]);
        if (mora_protocol.error != parser_errors_before) {
          rejected_frame_count++;
          if (debug_enabled) {
            printf("CATIA DEBUG:\trejected MORA frame at serial byte %llu (total rejected: %llu)\n",
                   (unsigned long long)serial_byte_count,
                   (unsigned long long)rejected_frame_count);
          }
        }
        if (mora_protocol.msg_received) {
          valid_frame_count++;
          if (debug_enabled) {
            printf("CATIA DEBUG:\taccepted MORA message id %u with %u payload bytes "
                   "(frame %llu)\n",
                   mora_protocol.msg_id, mora_protocol.payload_len,
                   (unsigned long long)valid_frame_count);
          }
          handle_received_message();
        }
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      fprintf(stderr, "CATIA:\tserial read failed: %s\n", strerror(errno));
    }

    if (debug_enabled && time(NULL) >= next_debug_heartbeat) {
      printf("CATIA DEBUG:\twaiting for MORA data: %llu bytes, %llu valid frames, "
             "%llu rejected frames\n",
             (unsigned long long)serial_byte_count,
             (unsigned long long)valid_frame_count,
             (unsigned long long)rejected_frame_count);
      next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
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
  close(fd);
  pthread_mutex_lock(&mut);
  pthread_cond_broadcast(&camera_available);
  while (shooting_thread_count > 0) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  if (earcam_active) {
    ear_cam_pipe_deinit();
  }
  camera.deinit();
  stop_local_serial_bridge();

  printf("CATIA:\tShutdown\n");
  return 0;
}

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

  static const char ready_message[] = "READY=1\nSTATUS=LWIR camera and MORA serial ready";
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
  char filename[MAX_FILENAME] = "";
  union dc_shot_union *shoot = (union dc_shot_union *) ptr;
  bool image_ready = false;

  pthread_mutex_lock(&mut);
  while (is_shooting && keep_running) {
    pthread_cond_wait(&camera_available, &mut);
  }
  if (!keep_running) {
    pthread_mutex_unlock(&mut);
    release_worker_slot();
    free(shoot);
    return NULL;
  }

  is_shooting = 1;
  shooting_count++;
  pthread_mutex_unlock(&mut);

  printf("CATIA-%d:\tShooting: start\n", shoot->data.nr);
  if (debug_enabled) {
    if (test_capture_enabled) {
      printf("CATIA-%d DEBUG:\trequesting test image for selected %s backend\n",
             shoot->data.nr, camera.name);
    } else {
      printf("CATIA-%d DEBUG:\trequesting image from %s backend\n", shoot->data.nr, camera.name);
    }
  }
  if (camera.shoot(filename, sizeof(filename), shoot->data.nr) != 0) {
    fprintf(stderr, "CATIA-%d:\t%s camera capture failed\n", shoot->data.nr, camera.name);
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
  if (filename[0] != '\0') {
    if (debug_enabled) {
      printf("CATIA-%d DEBUG:\twriting flight metadata to captured JPEG\n", shoot->data.nr);
    }
    if (image_exif_write(filename, shoot) == 0) {
      image_ready = true;
      printf("CATIA-%d:\tShooting: EXIF metadata added\n", shoot->data.nr);
      printf("Photo take %d\n", shoot->data.nr);
    } else {
      fprintf(stderr, "CATIA-%d:\tfailed to add EXIF metadata to %s\n", shoot->data.nr, filename);
    }
  }

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_cond_broadcast(&camera_available);
  pthread_mutex_unlock(&mut);

  if (image_ready) {
    int soda_result = run_soda(filename, shoot);
    printf("CATIA-%d:\tShooting: soda return %d of image %s\n",
           shoot->data.nr, soda_result, filename);
  }

  release_worker_slot();

  free(shoot);
  return NULL;
}

static void handle_received_message(void)
{
  mora_protocol.msg_received = false;

  if (mora_protocol.msg_id == MORA_SHOOT || mora_protocol.msg_id == MORA_SHOOT_TARGETED) {
    bool targeted = mora_protocol.msg_id == MORA_SHOOT_TARGETED;
    uint8_t expected = targeted ? MORA_SHOOT_TARGETED_MSG_SIZE : MORA_SHOOT_MSG_SIZE;
    if (debug_enabled) {
      printf("CATIA DEBUG:\tphoto trigger received; decoding shot payload\n");
    }
    if (mora_protocol.payload_len != expected) {
      fprintf(stderr, "CATIA:\tinvalid MORA shot payload length %u\n", mora_protocol.payload_len);
      return;
    }

    union dc_shot_union *shoot = malloc(sizeof(*shoot));
    if (shoot == NULL) {
      fprintf(stderr, "CATIA:\tfailed to allocate shot message\n");
      return;
    }
    for (size_t index = 0; index < MORA_SHOOT_MSG_SIZE; index++) {
      shoot->bin[index] = mora_protocol.payload[index];
    }
    int32_t camera_id = MORA_CAMERA_ALL;
    if (targeted) {
      union dc_shot_targeted_union targeted_msg;
      for (size_t index = 0; index < MORA_SHOOT_TARGETED_MSG_SIZE; index++) {
        targeted_msg.bin[index] = mora_protocol.payload[index];
      }
      camera_id = targeted_msg.data.camera_id;
    }
    printf("CATIA:\tSHOT %d cam %d | lat %.7f lon %.7f | MSL %.1f m AGL %.1f m | "
           "roll %.1f pitch %.1f yaw %.1f deg | speed %.1f m/s course %.1f deg\n",
           shoot->data.nr, camera_id,
           shoot->data.lat / 1e7,
           shoot->data.lon / 1e7,
           shoot->data.alt / 1000.0,
           shoot->data.groundalt / POSITION_BFP_SCALE,
           shoot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.vground / SPEED_BFP_SCALE,
           shoot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG);

    bool wants_ear = camera_id == MORA_CAMERA_ALL || camera_id == MORA_CAMERA_EARCAM;
    bool wants_optical = camera_id == MORA_CAMERA_ALL || camera_id == optical_camera_id;
    if (wants_ear && earcam_active) {
      // Geotagging is a memory copy; do it inline so no worker slot is consumed.
      if (ear_cam_pipe_record(shoot) == 0 && debug_enabled) {
        printf("CATIA-%d DEBUG:\tearcam sample recorded\n", shoot->data.nr);
      }
    } else if (camera_id == MORA_CAMERA_EARCAM) {
      fprintf(stderr, "CATIA-%d:\tearcam requested but not active\n", shoot->data.nr);
    }
    if (wants_optical) {
      start_shoot_worker(shoot);
      return;
    }
    if (camera_id != MORA_CAMERA_EARCAM) {
      fprintf(stderr, "CATIA-%d:\tno backend for camera id %d\n", shoot->data.nr, camera_id);
    }
    free(shoot);
  } else if (mora_protocol.msg_id == MORA_STOP_TARGETED) {
    if (mora_protocol.payload_len != MORA_STOP_TARGETED_MSG_SIZE) {
      fprintf(stderr, "CATIA:\tinvalid MORA stop payload length %u\n", mora_protocol.payload_len);
      return;
    }
    int32_t camera_id = 0;
    for (size_t index = 0; index < MORA_STOP_TARGETED_MSG_SIZE; index++) {
      camera_id |= (int32_t)mora_protocol.payload[index] << (8 * index);
    }
    handle_targeted_stop(camera_id);
  } else if (mora_protocol.msg_id == MORA_BUFFER_EMPTY) {
    send_msg_image_buffer();
  }
}

static void start_shoot_worker(union dc_shot_union *shoot)
{
  pthread_mutex_lock(&mut);
  if (shooting_thread_count >= MAX_PROCESSING_THREADS) {
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "CATIA-%d:\tprocessing queue is full\n", shoot->data.nr);
    free(shoot);
    return;
  }
  shooting_thread_count++;
  pthread_mutex_unlock(&mut);

  pthread_t shooting_thread;
  int thread_result = pthread_create(&shooting_thread, NULL, handle_msg_shoot, shoot);
  if (thread_result != 0) {
    pthread_mutex_lock(&mut);
    shooting_thread_count--;
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "CATIA-%d:\tfailed to start shooting thread: %s\n",
            shoot->data.nr, strerror(thread_result));
    free(shoot);
    return;
  }
  pthread_detach(shooting_thread);
  send_msg_status();
}

static void handle_targeted_stop(int32_t camera_id)
{
  bool keep = (camera_id & MORA_STOP_FLAG_KEEP) != 0;
  camera_id &= 0xFF;
  printf("CATIA:\tSTOP cam %d%s\n", camera_id, keep ? " (interim)" : "");
  if (camera_id != MORA_CAMERA_ALL && camera_id != MORA_CAMERA_EARCAM) {
    return;
  }
  if (!earcam_active) {
    if (camera_id == MORA_CAMERA_EARCAM) {
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
    if (ear_cam_pipe_render(&spot, sound_picture, sizeof(sound_picture)) == 0) {
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
        int soda_result = run_soda(sound_picture, &picture_shot);
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
  union mora_ear_result_union result;
  memset(&result, 0, sizeof(result));
  result.data.status = spot->valid ? MORA_EAR_RESULT_VALID : MORA_EAR_RESULT_INVALID;
  if (spot->valid) {
    result.data.lat = (int32_t)llround(spot->lat_deg * 1e7);
    result.data.lon = (int32_t)llround(spot->lon_deg * 1e7);
    result.data.agl_mm = (int32_t)llround(spot->agl_m * 1000.0);
    result.data.alt_mm = (int32_t)llround(spot->alt_m * 1000.0);
    result.data.level_cdb = (int32_t)llround(spot->level_db * 100.0);
    result.data.confidence = (int32_t)llround(spot->confidence * 1000.0);
  }
  result.data.sample_count = (int32_t)spot->sample_count;

  pthread_mutex_lock(&tx_mutex);
  MoraHeader(MORA_EAR_RESULT, MORA_EAR_RESULT_MSG_SIZE);
  for (size_t index = 0; index < MORA_EAR_RESULT_MSG_SIZE; index++) {
    MoraPutUint8(result.bin[index]);
  }
  MoraTrailer();
  pthread_mutex_unlock(&tx_mutex);
}

static int run_soda(const char *filename, const union dc_shot_union *shoot)
{
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
    (char *)camera.soda_application, (char *)filename,
    values[0], values[1], values[2], values[3], values[4],
    values[5], values[6], values[7], values[8], values[9], NULL
  };
  if (debug_enabled) {
    printf("CATIA-%d DEBUG:\tstarting SODA directly: %s\n",
           shoot->data.nr, camera.soda_application);
  }

  pid_t soda_pid;
  int spawn_result = posix_spawnp(&soda_pid, camera.soda_application, NULL, NULL,
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
  int i;

  // Check if image is available
  if (image_count > 0) {
    printf("CATIA:\thandle_msg_buffer: Send %d\n", image_idx);
    // Send the image
    image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
    image_count--;

    MoraHeader(MORA_PAYLOAD, MORA_PAYLOAD_MSG_SIZE);
    for (i = 0; i < IMAGE_SIZE; i++) {
      MoraPutUint8(image_buffer[image_idx][i]);
    }
    MoraTrailer();
  }
}

static inline void send_msg_status(void)
{
  int i;
  struct mora_status_struct status_msg;
  char *buffer = (char *) &status_msg;

  pthread_mutex_lock(&mut);
  status_msg.cpu = 0;
  status_msg.threads = shooting_thread_count;
  status_msg.shots = shooting_count;
  status_msg.extra = 0;
  pthread_mutex_unlock(&mut);

  MoraHeader(MORA_STATUS, MORA_STATUS_MSG_SIZE);
  for (i = 0; i < MORA_STATUS_MSG_SIZE; i++) {
    MoraPutUint8(buffer[i]);
  }
  MoraTrailer();
}

static void print_usage(const char *program)
{
  printf("Usage: %s [--serial DEVICE | --local] [--chdk | --aicam | --lwircam] [--earcam] [--test] [OPTIONS]\n", program);
  printf("  --serial DEVICE   serial endpoint (default: %s)\n", CATIA_SERIAL_DEVICE);
  printf("  --local           create local serial bridge %s <-> %s\n",
         CATIA_LOCAL_SIM_DEVICE, CATIA_LOCAL_APP_DEVICE);
  printf("  --chdk            use the CHDK camera backend (default outside local mode)\n");
  printf("  --aicam           use the AI camera backend\n");
  printf("  --lwircam         use the Tiny 1-C LWIR camera backend\n");
  printf("  --earcam          also run the acoustic earcam backend (camera id 4)\n");
  printf("  --earcam-sim LAT,LON[,DB]  earcam backend with a virtual loudspeaker instead of a microphone\n");
  printf("  --earcam-band LOW,HIGH  tone search band in Hz (default earcam 1800,3200; motionSCOUT K-T-R: 2400,3200)\n");
  printf("  --test            process a mock image for any camera backend\n");
  printf("                    randomly selects testphotos/*.jpg beside this executable\n");
  printf("  --mocktransform   transform test image using shot roll, pitch, and yaw\n");
  printf("  --debug           show serial, MORA frame, trigger, and capture diagnostics\n");
  printf("  --mock-image FILE image used by local or test capture (default: %s)\n", CATIA_MOCK_IMAGE);
  printf("  --help            show this help\n");
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
        "local", local_pipe_init, local_pipe_shoot, local_pipe_deinit, CATIA_LOCAL_SODA
      };
      break;
    case CAMERA_BACKEND_AI_CAM:
      camera = (struct camera_backend) {
        "aicam", ai_cam_pipe_init, ai_cam_pipe_shoot, ai_cam_pipe_deinit, CATIA_LOCAL_SODA
      };
      break;
    case CAMERA_BACKEND_LWIR_CAM:
      camera = (struct camera_backend) {
        "lwircam", lwir_cam_pipe_init, lwir_cam_pipe_shoot,
        lwir_cam_pipe_deinit, CATIA_LOCAL_SODA
      };
      break;
    case CAMERA_BACKEND_UNSELECTED:
      fprintf(stderr, "CATIA:\tinvalid camera backend\n");
      return -1;
  }

  if (test_mode) {
    camera.init = local_pipe_test_init;
    camera.shoot = local_pipe_shoot;
    camera.deinit = local_pipe_deinit;
    camera.soda_application = CATIA_LOCAL_SODA;
  }
  return 0;
}

static int chdk_backend_init(const char *source_image)
{
  (void)source_image;
  if (mkdir(CATIA_CHDK_PHOTO_DIR, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "CATIA:\tfailed to create CHDK photo directory %s: %s\n",
            CATIA_CHDK_PHOTO_DIR, strerror(errno));
    return -1;
  }
  struct stat directory_status;
  if (stat(CATIA_CHDK_PHOTO_DIR, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(CATIA_CHDK_PHOTO_DIR, W_OK) != 0) {
    fprintf(stderr, "CATIA:\tCHDK photo directory is not writable: %s\n",
            CATIA_CHDK_PHOTO_DIR);
    return -1;
  }
  chdk_pipe_init();
  return 0;
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

  int length = snprintf(filename, filename_size, "%s/c%06d.jpg",
                        CATIA_CHDK_PHOTO_DIR, image_number);
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
