#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "serial.h"
#include "chdk_pipe.h"
#include "protocol.h"
#include "socket.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 8
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
// Search&Rescue Onboard Detection Application
#define SODA "/root/develop/allthings_obc2014/src/soda/soda"
#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232

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

static void *handle_msg_shoot(void *ptr);
static inline void send_msg_image_buffer(void);
static inline void send_msg_status(void);
static void print_usage(const char *program);
static int lock_local_instance(void);
static pid_t start_local_serial_bridge(void);
static int wait_for_path(const char *path);
static int copy_fake_image(const char *source, char *destination, size_t destination_size, int image_number);
static void stop_local_serial_bridge(void);
static void handle_signal(int signal_number);

static volatile int is_shooting, image_idx, image_count, shooting_idx, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static bool local_mode;
static bool local_bridge_requested;
static bool local_bridge_owned;
static volatile sig_atomic_t keep_running = 1;
static pid_t socat_pid = -1;
static int local_lock_fd = -1;
static const char *fake_image = CATIA_FAKE_IMAGE;

int main(int argc, char *argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS];
  const char *serial_device = CATIA_SERIAL_DEVICE;
  bool serial_was_selected = false;
  char c;
  int i;
  int option;

  static const struct option long_options[] = {
    {"serial", required_argument, NULL, 's'},
    {"local", no_argument, NULL, 'l'},
    {"fake-image", required_argument, NULL, 'i'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };

  while ((option = getopt_long(argc, argv, "s:li:h", long_options, NULL)) != -1) {
    switch (option) {
      case 's':
        serial_device = optarg;
        serial_was_selected = true;
        break;
      case 'l':
        local_mode = true;
        local_bridge_requested = true;
        break;
      case 'i':
        fake_image = optarg;
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

  // Initialization
  printf("CATIA:\tStarting Camera Application Triggering Image Analysis\n");
  printf("CATIA:\tserial device: %s\n", serial_device);
  if (local_mode) {
    printf("CATIA:\tlocal simulator device: %s\n", CATIA_LOCAL_SIM_DEVICE);
    printf("CATIA:\tfake camera image: %s\n", fake_image);
  } else {
    chdk_pipe_init();
  }
  int ret = serial_init(serial_device);
  if (ret < 0) {
    stop_local_serial_bridge();
    return -1;
  }
  pthread_mutex_init(&mut, NULL);
  socket_init(1);
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  // Initial settings
  is_shooting = 0;
  mora_protocol.status = 0;
  image_idx = 0;
  image_count = 0;
  shooting_idx = 0;
  shooting_count = 0;
  shooting_thread_count = 0;

  printf("Started OK\n");

  // MAIN loop
  while (keep_running) {

    // Read the serial
    if (read(fd, &c, 1) > 0) {
      parse_mora(&mora_protocol, c);
    } else if (errno != 11) {
      printf("CATIA:\nSerial error: %d\n" , errno);
    }

    // Parse serial commands
    if (mora_protocol.msg_received) {
      // Process Only Once
      mora_protocol.msg_received = false;

      // Shoot an image if not busy
      if (mora_protocol.msg_id == MORA_SHOOT) {
        // Parse the shoot message
        union dc_shot_union *shoot = (union dc_shot_union *) malloc(sizeof(union dc_shot_union));
        for (i = 0; i < MORA_SHOOT_MSG_SIZE; i++) {
          shoot->bin[i] = mora_protocol.payload[i];
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

        pthread_create(&shooting_threads[(shooting_idx++ % MAX_PROCESSING_THREADS)], NULL, handle_msg_shoot, (void *)shoot);
        send_msg_status();
      }

      // Fill the image buffer (happens busy because needs fd anyway)
      if (mora_protocol.msg_id == MORA_BUFFER_EMPTY) {
        send_msg_image_buffer();
      }
    }

    // Read the socket
    if (socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if (image_count < MAX_IMAGE_BUFFERS) {
        image_count++;
      }
    }

    usleep(1000);

  }

  // Close
  close(fd);
  if (!local_mode) {
    chdk_pipe_deinit();
  }
  stop_local_serial_bridge();

  printf("CATIA:\tShutdown\n");
  return 0;
}

static void *handle_msg_shoot(void *ptr)
{
  char filename[MAX_FILENAME], soda_call[1024];
  union dc_shot_union *shoot = (union dc_shot_union *) ptr;

  // Test if can shoot
  pthread_mutex_lock(&mut);
  if (is_shooting) {
    pthread_mutex_unlock(&mut);
    printf("CATIA-%d:\tShooting: too fast\n", shoot->data.nr);

    free(shoot);
    return NULL;
  }

  is_shooting = 1;
  shooting_count++;
  shooting_thread_count++;
  pthread_mutex_unlock(&mut);

  printf("CATIA-%d:\tShooting: start\n", shoot->data.nr);
  if (local_mode) {
    if (copy_fake_image(fake_image, filename, sizeof(filename), shoot->data.nr) != 0) {
      fprintf(stderr, "CATIA-%d:\tfailed to create fake image\n", shoot->data.nr);
      filename[0] = '\0';
    }
  } else {
    chdk_pipe_shoot(filename);
  }
  printf("CATIA-%d:\tShooting: got image %s\n", shoot->data.nr, filename);
  if (filename[0] != '\0') {
    printf("Photo take %d\n", shoot->data.nr);
  }

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_mutex_unlock(&mut);

  if (!local_mode) {
    //Parse the image
    int command_length = snprintf(soda_call, sizeof(soda_call), "%s %s %d %d %d %d %d %d %d %d %d %d", SODA, filename,
                                  shoot->data.nr, shoot->data.lat, shoot->data.lon, shoot->data.alt, shoot->data.phi,
                                  shoot->data.theta, shoot->data.psi, shoot->data.vground, shoot->data.course,
                                  shoot->data.groundalt);
    if (command_length < 0 || (size_t)command_length >= sizeof(soda_call)) {
      fprintf(stderr, "CATIA-%d:\tSODA command is too long\n", shoot->data.nr);
    } else {
      printf("CATIA-%d:\tCalling '%s'\n", shoot->data.nr, soda_call);
      short int ret = system(soda_call);
      printf("CATIA-%d:\tShooting: soda return %d of image %s\n", shoot->data.nr, ret, filename);
    }
  }

  pthread_mutex_lock(&mut);
  shooting_thread_count--;
  pthread_mutex_unlock(&mut);

  free(shoot);
  return NULL;
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
  printf("Usage: %s [--serial DEVICE] [--local] [--fake-image FILE]\n", program);
  printf("  --serial DEVICE   serial endpoint (default: %s)\n", CATIA_SERIAL_DEVICE);
  printf("  --local           create %s <-> %s and use a fake camera\n",
         CATIA_LOCAL_SIM_DEVICE, CATIA_LOCAL_APP_DEVICE);
  printf("  --fake-image FILE image copied for each shot in local mode (default: %s)\n", CATIA_FAKE_IMAGE);
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

static int copy_fake_image(const char *source, char *destination, size_t destination_size, int image_number)
{
  int length = snprintf(destination, destination_size, "/tmp/catia-shot-%06d.jpg", image_number);
  if (length < 0 || (size_t)length >= destination_size) {
    return -1;
  }

  int source_fd = open(source, O_RDONLY);
  if (source_fd < 0) {
    fprintf(stderr, "CATIA:\tfailed to open fake image %s: %s\n", source, strerror(errno));
    return -1;
  }
  int destination_fd = open(destination, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (destination_fd < 0) {
    fprintf(stderr, "CATIA:\tfailed to create %s: %s\n", destination, strerror(errno));
    close(source_fd);
    return -1;
  }

  char buffer[4096];
  ssize_t bytes_read;
  int result = 0;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t written = 0;
    while (written < bytes_read) {
      ssize_t count = write(destination_fd, &buffer[written], (size_t)(bytes_read - written));
      if (count <= 0) {
        result = -1;
        break;
      }
      written += count;
    }
    if (result != 0) {
      break;
    }
  }
  if (bytes_read < 0) {
    result = -1;
  }
  close(destination_fd);
  close(source_fd);
  return result;
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
