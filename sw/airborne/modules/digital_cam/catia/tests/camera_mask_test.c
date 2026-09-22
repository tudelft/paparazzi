#define chdk_pipe_init test_chdk_init
#define chdk_pipe_shoot test_chdk_shoot
#define chdk_pipe_deinit test_chdk_deinit
#define ai_cam_pipe_init test_ai_init
#define ai_cam_pipe_shoot test_ai_shoot
#define ai_cam_pipe_deinit test_ai_deinit
#define lwir_cam_pipe_init test_lwir_init
#define lwir_cam_pipe_init_cancelable test_lwir_init_cancelable
#define lwir_cam_pipe_shoot test_lwir_shoot
#define lwir_cam_pipe_deinit test_lwir_deinit
#define lwir_cam_pipe_geolocate test_geolocate
#define ear_cam_pipe_init test_ear_init
#define ear_cam_pipe_record test_ear_record
#define posix_spawnp test_spawn
#define main catia_application_main
#include "../catia.c"
#undef main

#include <assert.h>

static const char *fixture;
static unsigned captures[5], inits[5], failures;
static unsigned soda_calls[5];
static bool fail_chdk, fail_ai, fail_lwir, fail_lwir_capture, fail_ear;
static int32_t capture_numbers[128];
static size_t capture_count;
static pthread_mutex_t gate_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond = PTHREAD_COND_INITIALIZER;
static bool hold_capture;
static bool hold_lwir_capture;
static bool lwir_capture_waiting;
static int hold_ai_postprocess_number = -1;
static bool ai_postprocess_waiting;
static int last_lwir_capture_number = -1;

int test_chdk_init(void) { ++inits[1]; return fail_chdk ? -1 : 0; }
int test_ai_init(const char *source) { (void)source; ++inits[2]; return 0; }
int test_lwir_init(const char *source) { (void)source; ++inits[3]; return fail_lwir ? -1 : 0; }
int test_lwir_init_cancelable(const char *source, const volatile sig_atomic_t *keep_running)
{
  assert(keep_running != NULL);
  return test_lwir_init(source);
}
int test_ear_init(const char *source) { (void)source; ++inits[4]; return fail_ear ? -1 : 0; }
void test_chdk_deinit(void) {}
void test_ai_deinit(void) {}
void test_lwir_deinit(void) {}
int test_geolocate(char *filename) { assert(access(filename, R_OK) == 0); return 0; }

static void copy_fixture(const char *filename)
{
  FILE *source = fopen(fixture, "rb");
  FILE *destination = fopen(filename, "wb");
  assert(source != NULL && destination != NULL);
  int byte;
  while ((byte = fgetc(source)) != EOF) assert(fputc(byte, destination) != EOF);
  assert(!ferror(source));
  assert(fclose(source) == 0 && fclose(destination) == 0);
}

static int capture(char *filename, size_t size, int number, int camera_id)
{
  pthread_mutex_lock(&gate_mutex);
  if (camera_id == CATIA_CAMERA_LWIRCAM && hold_lwir_capture) {
    lwir_capture_waiting = true;
    pthread_cond_broadcast(&gate_cond);
  }
  while (hold_capture || (camera_id == CATIA_CAMERA_LWIRCAM && hold_lwir_capture)) {
    pthread_cond_wait(&gate_cond, &gate_mutex);
  }
  ++captures[camera_id];
  if (camera_id == CATIA_CAMERA_LWIRCAM) last_lwir_capture_number = number;
  assert(capture_count < sizeof(capture_numbers) / sizeof(capture_numbers[0]));
  capture_numbers[capture_count++] = number;
  pthread_cond_broadcast(&gate_cond);
  pthread_mutex_unlock(&gate_mutex);
  if (camera_id == 2 && fail_ai) { ++failures; return -1; }
  if (camera_id == 3 && fail_lwir_capture) { ++failures; return -1; }
  int length = snprintf(filename, size, "photos/%c%06d.jpg", camera_id == 2 ? 'a' : 'l', number);
  assert(length > 0 && (size_t)length < size);
  copy_fixture(filename);
  return 0;
}

int test_ai_shoot(char *filename, size_t size, int number) { return capture(filename, size, number, 2); }
int test_lwir_shoot(char *filename, size_t size, int number) { return capture(filename, size, number, 3); }
void test_chdk_shoot(char *filename)
{
  ++captures[1];
  strcpy(filename, "photos/download.jpg");
  copy_fixture(filename);
}
int test_ear_record(const union dc_shot_union *shot)
{
  assert(shot->data.lat == 488100000);
  ++captures[4];
  return 0;
}

int test_spawn(pid_t *pid, const char *file, const posix_spawn_file_actions_t *actions,
               const posix_spawnattr_t *attributes, char *const arguments[], char *const environment[])
{
  (void)file;
  int camera_id = 0;
  if (strcmp(arguments[12], "--chdkcam") == 0) camera_id = 1;
  if (strcmp(arguments[12], "--aicam") == 0) camera_id = 2;
  if (strcmp(arguments[12], "--lwircam") == 0) camera_id = 3;
  assert(camera_id != 0);
  pthread_mutex_lock(&gate_mutex);
  if (camera_id == CATIA_CAMERA_AICAM && hold_ai_postprocess_number >= 0) {
    char held_image[32];
    assert(snprintf(held_image, sizeof(held_image), "a%06d.jpg",
                    hold_ai_postprocess_number) > 0);
    if (strstr(arguments[1], held_image) != NULL) {
      ai_postprocess_waiting = true;
      pthread_cond_broadcast(&gate_cond);
      while (hold_ai_postprocess_number >= 0) pthread_cond_wait(&gate_cond, &gate_mutex);
    }
  }
  ++soda_calls[camera_id];
  pthread_mutex_unlock(&gate_mutex);
  const char prefixes[] = " cal";
  assert(arguments[1][7] == prefixes[camera_id]);
  assert(access(arguments[1], R_OK) == 0);
  char *const child_arguments[] = {"true", NULL};
  return posix_spawn(pid, "/usr/bin/true", actions, attributes, child_arguments, environment);
}

static void send_shot(uint8_t message_id, uint32_t selection, int number, bool malformed)
{
  union dc_shot_mask_union message = {0};
  message.data.shot.data.nr = number;
  message.data.shot.data.lat = 488100000;
  message.data.shot.data.lon = 78530000;
  message.data.shot.data.alt = 140000;
  message.data.shot.data.groundalt = 10240;
  message.data.camera_mask = selection;
  catia_protocol.msg_id = message_id;
  catia_protocol.payload_len = message_id == CATIA_SHOOT ? CATIA_SHOOT_MSG_SIZE : sizeof(message.bin);
  if (malformed) --catia_protocol.payload_len;
  for (size_t index = 0; index < sizeof(message.bin); ++index) catia_protocol.payload[index] = message.bin[index];
  handle_received_message();
}

static void wait_captures(void)
{
  pthread_mutex_lock(&mut);
  while (shooting_thread_count != 0) pthread_cond_wait(&workers_finished, &mut);
  pthread_mutex_unlock(&mut);
}

static void wait_lwir_state(bool initialized)
{
  pthread_mutex_lock(&mut);
  while (camera_initialized[CATIA_CAMERA_LWIRCAM] != initialized
         || (!initialized && !camera_unavailable[CATIA_CAMERA_LWIRCAM])) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
}

int main(int argc, char **argv)
{
  assert(argc == 2);
  fixture = argv[1];
  assert(mkdir("photos", 0700) == 0);
  int uart[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, uart) == 0);
  assert(serial_tx_start(uart[0]) == 0);
  lwir_async_recovery_enabled = false;

  send_shot(CATIA_SHOOT_MASK, 0, 1, false);
  send_shot(CATIA_SHOOT_MASK, 256, 1, false);
  send_shot(CATIA_SHOOT_MASK, UINT32_MAX, 1, false);
  send_shot(CATIA_SHOOT_MASK, 255, 1, true);
  send_shot(CATIA_SHOOT_MASK, 128, 1, false);
  assert(shooting_thread_count == 0 && captures[4] == 0);

  send_shot(CATIA_SHOOT_MASK, 5, 2, false);
  wait_captures();
  assert(captures[1] == 1 && captures[2] == 0 && captures[3] == 1 && captures[4] == 0);
  assert(access("photos/c000002.jpg", R_OK) == 0 && access("photos/l000002.jpg", R_OK) == 0);
  send_shot(CATIA_SHOOT_MASK, 10, 3, false);
  wait_captures();
  assert(captures[2] == 1 && captures[4] == 1);
  assert(access("photos/a000003.jpg", R_OK) == 0);
  send_shot(CATIA_SHOOT_MASK, 255, 4, false);
  wait_captures();
  assert(captures[1] == 2 && captures[2] == 2 && captures[3] == 2 && captures[4] == 2);
  for (int camera_id = 1; camera_id <= 4; ++camera_id) assert(inits[camera_id] == 1);

  send_shot(CATIA_SHOOT_TARGETED, 4, 5, false);
  send_shot(CATIA_SHOOT_MASK, 4, 6, false);
  wait_captures();
  assert(captures[4] == 3 && captures[3] == 3);
  send_shot(CATIA_SHOOT_TARGETED, 0, 7, false);
  wait_captures();
  send_shot(CATIA_SHOOT, 0, 8, false);
  wait_captures();
  assert(captures[1] == 4 && captures[2] == 4 && captures[3] == 5 && captures[4] == 5);

  cameras_deinit();
  fail_chdk = fail_ai = true;
  send_shot(CATIA_SHOOT_MASK, 15, 9, false);
  wait_captures();
  assert(captures[1] == 4 && captures[2] == 5 && captures[3] == 6 && captures[4] == 6 && failures == 1);
  assert(!camera_initialized[CATIA_CAMERA_AICAM]);
  unsigned ai_initializations = inits[CATIA_CAMERA_AICAM];
  assert(soda_calls[1] == 4 && soda_calls[2] == 4 && soda_calls[3] == 6);
  fail_chdk = fail_ai = false;
  send_shot(CATIA_SHOOT_MASK, 15, 10, false);
  wait_captures();
  assert(captures[1] == 5 && captures[2] == 6 && captures[3] == 7 && captures[4] == 7);
  assert(camera_initialized[CATIA_CAMERA_AICAM]);
  assert(inits[CATIA_CAMERA_AICAM] == ai_initializations + 1);

  // Live LWIR initialization is asynchronous: a missing Tiny1-C cannot delay the
  // ordered AIcam lane, retries are rate-limited, and a reconnect becomes READY
  // without restarting CATIA.
  cameras_deinit();
  lwir_async_recovery_enabled = true;
  fail_lwir = true;
  unsigned lwir_inits_before = inits[CATIA_CAMERA_LWIRCAM];
  unsigned ai_captures_before = captures[CATIA_CAMERA_AICAM];
  unsigned ear_captures_before = captures[CATIA_CAMERA_EARCAM];
  send_shot(CATIA_SHOOT_MASK,
            CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR | CATIA_CAMERA_MASK_EAR,
            11, false);
  wait_captures();
  wait_lwir_state(false);
  assert(inits[CATIA_CAMERA_LWIRCAM] == lwir_inits_before + 1);
  assert(camera_unavailable[CATIA_CAMERA_LWIRCAM]);
  assert(!camera_initialized[CATIA_CAMERA_LWIRCAM]);
  assert(captures[CATIA_CAMERA_AICAM] == ai_captures_before + 1);
  assert(captures[CATIA_CAMERA_EARCAM] == ear_captures_before + 1);
  send_shot(CATIA_SHOOT_MASK,
            CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR | CATIA_CAMERA_MASK_EAR,
            20, false);
  wait_captures();
  assert(inits[CATIA_CAMERA_LWIRCAM] == lwir_inits_before + 1);
  assert(captures[CATIA_CAMERA_AICAM] == ai_captures_before + 2);
  assert(captures[CATIA_CAMERA_EARCAM] == ear_captures_before + 2);
  assert(camera_unavailable[CATIA_CAMERA_LWIRCAM]);
  fail_lwir = false;
  pthread_mutex_lock(&mut);
  lwir_retry_after_us = 0;
  pthread_mutex_unlock(&mut);
  wait_lwir_state(true);
  assert(!camera_unavailable[CATIA_CAMERA_LWIRCAM]);
  assert(camera_initialized[CATIA_CAMERA_LWIRCAM]);
  assert(captures[CATIA_CAMERA_LWIRCAM] == 7);
  send_shot(CATIA_SHOOT_MASK,
            CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR | CATIA_CAMERA_MASK_EAR,
            21, false);
  wait_captures();
  assert(captures[CATIA_CAMERA_LWIRCAM] == 8);
  puts("LWIR camera unavailable: AIcam and EARcam continue, then LWIR recovers after cooldown");

  unsigned lwir_before_ordering = captures[CATIA_CAMERA_LWIRCAM];
  uint64_t ordering_skips_before = lwir_busy_skipped_count;
  hold_ai_postprocess_number = 22;
  ai_postprocess_waiting = false;
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 22, false);
  pthread_mutex_lock(&gate_mutex);
  while (!ai_postprocess_waiting) pthread_cond_wait(&gate_cond, &gate_mutex);
  pthread_mutex_unlock(&gate_mutex);
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 23, false);
  pthread_mutex_lock(&mut);
  while (lwir_busy_skipped_count == ordering_skips_before) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  pthread_mutex_lock(&gate_mutex);
  hold_ai_postprocess_number = -1;
  pthread_cond_broadcast(&gate_cond);
  pthread_mutex_unlock(&gate_mutex);
  wait_captures();
  assert(captures[CATIA_CAMERA_LWIRCAM] == lwir_before_ordering + 1);
  assert(last_lwir_capture_number == 22);
  puts("LWIR trigger ordering: an earlier reserved thermal request cannot be overtaken");

  unsigned ai_before_busy_lwir = captures[CATIA_CAMERA_AICAM];
  unsigned lwir_before_busy_lwir = captures[CATIA_CAMERA_LWIRCAM];
  uint64_t busy_skips_before = lwir_busy_skipped_count;
  hold_lwir_capture = true;
  lwir_capture_waiting = false;
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 23, false);
  pthread_mutex_lock(&gate_mutex);
  while (!lwir_capture_waiting) pthread_cond_wait(&gate_cond, &gate_mutex);
  pthread_mutex_unlock(&gate_mutex);
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 24, false);
  pthread_mutex_lock(&gate_mutex);
  while (captures[CATIA_CAMERA_AICAM] < ai_before_busy_lwir + 2) {
    pthread_cond_wait(&gate_cond, &gate_mutex);
  }
  pthread_mutex_unlock(&gate_mutex);
  pthread_mutex_lock(&mut);
  while (lwir_busy_skipped_count == busy_skips_before) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  pthread_mutex_lock(&gate_mutex);
  hold_lwir_capture = false;
  pthread_cond_broadcast(&gate_cond);
  pthread_mutex_unlock(&gate_mutex);
  wait_captures();
  assert(captures[CATIA_CAMERA_LWIRCAM] == lwir_before_busy_lwir + 1);
  puts("Busy LWIR lane: following AIcam shot completes without waiting or queueing LWIR");

  fail_lwir_capture = true;
  unsigned lwir_before_capture_failure = captures[CATIA_CAMERA_LWIRCAM];
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 25, false);
  wait_captures();
  wait_lwir_state(false);
  assert(camera_unavailable[CATIA_CAMERA_LWIRCAM]);
  assert(!camera_initialized[CATIA_CAMERA_LWIRCAM]);
  fail_lwir_capture = false;
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 26, false);
  wait_captures();
  assert(captures[CATIA_CAMERA_LWIRCAM] == lwir_before_capture_failure + 1);
  pthread_mutex_lock(&mut);
  lwir_retry_after_us = 0;
  pthread_mutex_unlock(&mut);
  wait_lwir_state(true);
  send_shot(CATIA_SHOOT_MASK, CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, 28, false);
  wait_captures();
  assert(captures[CATIA_CAMERA_LWIRCAM] == lwir_before_capture_failure + 2);
  puts("LWIR capture failure: backoff, AIcam isolation, and asynchronous recovery passed");
  cameras_deinit();
  lwir_async_recovery_enabled = false;

  const struct {
    uint8_t mask;
    unsigned selected[4];
  } examples[] = {
    {0x01, {1, 0, 0, 0}},
    {0x03, {1, 1, 0, 0}},
    {CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR, {0, 1, 1, 0}},
    {0x07, {1, 1, 1, 0}},
    {0x0F, {1, 1, 1, 1}},
    {0x05, {1, 0, 1, 0}},
  };
  for (size_t example = 0; example < sizeof(examples) / sizeof(examples[0]); ++example) {
    unsigned before[4];
    for (size_t camera_index = 0; camera_index < 4; ++camera_index) {
      before[camera_index] = captures[camera_index + 1];
    }
    send_shot(CATIA_SHOOT_MASK, examples[example].mask, 100 + (int)example, false);
    wait_captures();
    for (size_t camera_index = 0; camera_index < 4; ++camera_index) {
      assert(captures[camera_index + 1] == before[camera_index] + examples[example].selected[camera_index]);
    }
  }
  puts("Exact masks 00000001, 00000011, 00000110, 00000111, 00001111 and 00000101 passed");

  /* An optional microphone failure must not suppress either optical camera from
   * the same default AIcam + LWIRcam + EARcam command. */
  earcam_active = false;
  earcam_unavailable = false;
  fail_ear = true;
  unsigned ai_before_missing_ear = captures[CATIA_CAMERA_AICAM];
  unsigned lwir_before_missing_ear = captures[CATIA_CAMERA_LWIRCAM];
  unsigned ear_before_missing_ear = captures[CATIA_CAMERA_EARCAM];
  send_shot(CATIA_SHOOT_MASK,
            CATIA_CAMERA_MASK_AICAM | CATIA_CAMERA_MASK_LWIR | CATIA_CAMERA_MASK_EAR,
            110, false);
  wait_captures();
  assert(captures[CATIA_CAMERA_AICAM] == ai_before_missing_ear + 1);
  assert(captures[CATIA_CAMERA_LWIRCAM] == lwir_before_missing_ear + 1);
  assert(captures[CATIA_CAMERA_EARCAM] == ear_before_missing_ear);
  assert(!earcam_active && earcam_unavailable);
  fail_ear = false;
  earcam_unavailable = false;

  size_t first_queued = capture_count;
  pthread_mutex_lock(&gate_mutex);
  hold_capture = true;
  pthread_mutex_unlock(&gate_mutex);
  for (int number = 20; number < 20 + MAX_PROCESSING_THREADS; ++number) {
    send_shot(CATIA_SHOOT_MASK, 2, number, false);
  }
  send_shot(CATIA_SHOOT_MASK, 2, 99, false);
  pthread_mutex_lock(&mut);
  assert(shooting_thread_count == MAX_PROCESSING_THREADS);
  pthread_mutex_unlock(&mut);
  pthread_mutex_lock(&gate_mutex);
  hold_capture = false;
  pthread_cond_broadcast(&gate_cond);
  pthread_mutex_unlock(&gate_mutex);
  wait_captures();
  for (int index = 0; index < MAX_PROCESSING_THREADS; ++index) assert(capture_numbers[first_queued + index] == 20 + index);
  assert(access("photos/a000099.jpg", F_OK) != 0);
  pthread_mutex_lock(&mut);
  size_t before_shutdown = capture_count;
  keep_running = 0;
  pthread_mutex_unlock(&mut);
  for (int number = 30; number < 33; ++number) send_shot(CATIA_SHOOT_MASK, 2, number, false);
  wait_captures();
  assert(capture_count == before_shutdown);
  cameras_deinit();
  serial_tx_stop();
  close(uart[0]);
  close(uart[1]);
  puts("Camera masks: combinations, prefixes, legacy ALL, failure isolation and FIFO backpressure passed");
  return 0;
}