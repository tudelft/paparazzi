#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "ai_cam_pipe.h"
#include "serial.h"
#include "chdk_pipe.h"
#include "image_exif.h"
#include "image_fake_transform.h"
#include "local_pipe.h"
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

#ifndef CATIA_FAKE_IMAGE
#define CATIA_FAKE_IMAGE "fake-camera.jpg"
#endif

#ifndef CATIA_LOCAL_SODA
#define CATIA_LOCAL_SODA "soda_local"
#endif

enum camera_backend_type {
  CAMERA_BACKEND_UNSELECTED,
  CAMERA_BACKEND_CHDK,
  CAMERA_BACKEND_LOCAL,
  CAMERA_BACKEND_AI_CAM
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
static int camera_backend_select(enum camera_backend_type type, bool test_mode);
static int chdk_backend_init(const char *source_image);
static int chdk_backend_shoot(char *filename, size_t filename_size, int image_number);

static volatile int is_shooting, image_idx, image_count, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t workers_finished = PTHREAD_COND_INITIALIZER;
static bool local_mode;
static enum camera_backend_type requested_camera_backend = CAMERA_BACKEND_UNSELECTED;
static struct camera_backend camera;
static bool local_bridge_requested;
static bool local_bridge_owned;
static volatile sig_atomic_t keep_running = 1;
static pid_t socat_pid = -1;
static int local_lock_fd = -1;
static const char *fake_image = CATIA_FAKE_IMAGE;
static bool fake_transform_enabled;
static bool debug_enabled;
static bool test_capture_enabled;

extern char **environ;

int main(int argc, char *argv[])
{
  const char *serial_device = CATIA_SERIAL_DEVICE;
  bool serial_was_selected = false;
  bool fake_image_was_selected = false;
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
    {"test", no_argument, NULL, 't'},
    {"faketransform", no_argument, NULL, 'f'},
    {"debug", no_argument, NULL, 'd'},
    {"fake-image", required_argument, NULL, 'i'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };

  while ((option = getopt_long(argc, argv, "s:lcatfdi:h", long_options, NULL)) != -1) {
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
          fprintf(stderr, "CATIA:\tonly one of --chdk and --aicam may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_CHDK;
        break;
      case 'a':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "CATIA:\tonly one of --chdk and --aicam may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_AI_CAM;
        break;
      case 't':
        test_mode = true;
        break;
      case 'f':
        fake_transform_enabled = true;
        break;
      case 'd':
        debug_enabled = true;
        break;
      case 'i':
        fake_image = optarg;
        fake_image_was_selected = true;
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

  if (fake_image_was_selected && !test_mode
      && (!local_mode || requested_camera_backend != CAMERA_BACKEND_UNSELECTED)) {
    fprintf(stderr, "CATIA:\t--fake-image requires --test or --local without --chdk or --aicam\n");
    return 2;
  }
  if (fake_transform_enabled && !test_mode) {
    fprintf(stderr, "CATIA:\t--faketransform requires --test\n");
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
    if (fake_image_was_selected) {
      printf("CATIA:\tcamera test mode: fake image %s\n", fake_image);
    } else {
      printf("CATIA:\tcamera test mode: random image from testphotos when available\n");
    }
    printf("CATIA:\tfake attitude transform: %s\n", fake_transform_enabled ? "enabled" : "disabled");
  }
  if (local_mode) {
    printf("CATIA:\tlocal simulator device: %s\n", CATIA_LOCAL_SIM_DEVICE);
  }
  const char *camera_source_image = NULL;
  if (selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode) {
    printf("CATIA:\tfake camera image: %s\n", fake_image);
    camera_source_image = fake_image;
  } else if (test_mode) {
    camera_source_image = fake_image_was_selected ? fake_image : NULL;
  }
  signal(SIGPIPE, SIG_IGN);
  if (camera.init(camera_source_image) != 0) {
    stop_local_serial_bridge();
    return 1;
  }
  int ret = serial_init(serial_device);
  if (ret < 0) {
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
  while (shooting_thread_count > 0) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  camera.deinit();
  stop_local_serial_bridge();

  printf("CATIA:\tShutdown\n");
  return 0;
}

static void *handle_msg_shoot(void *ptr)
{
  char filename[MAX_FILENAME] = "";
  union dc_shot_union *shoot = (union dc_shot_union *) ptr;
  bool image_ready = false;

  // Test if can shoot
  pthread_mutex_lock(&mut);
  if (is_shooting) {
    pthread_mutex_unlock(&mut);
    printf("CATIA-%d:\tShooting: too fast\n", shoot->data.nr);

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
  if (filename[0] != '\0' && fake_transform_enabled) {
    int transform_result = image_fake_transform(filename, shoot);
    if (transform_result < 0) {
      fprintf(stderr, "CATIA-%d:\tfailed to apply fake attitude transform to %s\n",
              shoot->data.nr, filename);
      filename[0] = '\0';
    } else if (transform_result == IMAGE_FAKE_TRANSFORM_SKIPPED) {
      printf("CATIA-%d:\tShooting: fake attitude transform skipped; source image retained\n",
             shoot->data.nr);
    } else {
      printf("CATIA-%d:\tShooting: fake attitude transform applied\n", shoot->data.nr);
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

  if (mora_protocol.msg_id == MORA_SHOOT) {
    if (debug_enabled) {
      printf("CATIA DEBUG:\tphoto trigger received; decoding shot payload\n");
    }
    if (mora_protocol.payload_len != MORA_SHOOT_MSG_SIZE) {
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
    printf("CATIA:\tSHOT %d | lat %.7f lon %.7f | MSL %.1f m AGL %.1f m | "
           "roll %.1f pitch %.1f yaw %.1f deg | speed %.1f m/s course %.1f deg\n",
           shoot->data.nr,
           shoot->data.lat / 1e7,
           shoot->data.lon / 1e7,
           shoot->data.alt / 1000.0,
           shoot->data.groundalt / POSITION_BFP_SCALE,
           shoot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.vground / SPEED_BFP_SCALE,
           shoot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG);

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
  } else if (mora_protocol.msg_id == MORA_BUFFER_EMPTY) {
    send_msg_image_buffer();
  }
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
  printf("Usage: %s [--serial DEVICE | --local] [--chdk | --aicam] [--test] [OPTIONS]\n", program);
  printf("  --serial DEVICE   serial endpoint (default: %s)\n", CATIA_SERIAL_DEVICE);
  printf("  --local           create local serial bridge %s <-> %s\n",
         CATIA_LOCAL_SIM_DEVICE, CATIA_LOCAL_APP_DEVICE);
  printf("  --chdk            use the CHDK camera backend (default outside local mode)\n");
  printf("  --aicam           use the AI camera backend\n");
  printf("  --test            process a fake image for local, CHDK, or AI-camera testing\n");
  printf("                    randomly selects testphotos/*.jpg beside this executable\n");
  printf("  --faketransform   transform test image using shot roll, pitch, and yaw\n");
  printf("  --debug           show serial, MORA frame, trigger, and capture diagnostics\n");
  printf("  --fake-image FILE image used by local or test capture (default: %s)\n", CATIA_FAKE_IMAGE);
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
  chdk_pipe_init();
  return 0;
}

static int chdk_backend_shoot(char *filename, size_t filename_size, int image_number)
{
  (void)filename_size;
  (void)image_number;
  chdk_pipe_shoot(filename);
  return filename[0] == '\0' ? -1 : 0;
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
