#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define DC_H
#define TELEMETRY_H
#define STATE_H
#define SYS_TIME_H
#define MCU_PERIPH_UART_H
#define GPS_H
#define COMMON_NAV_H
#define BOARD_CONFIG "generated/airframe.h"
#define CAMERA_LINK test_uart
#define POS_BFP_OF_REAL(value) ((int32_t)((value) * 256))
#define DC_IMAGE_BUFFER 1000
#define DC_SHOOT 1
#define DC_TALLER 2
#define DC_WIDER 3
#define DC_ON 4
#define DC_OFF 5

struct test_device {
  int (*check_free_space)(void *, long *, uint16_t);
  void *periph;
  void (*put_byte)(void *, long, uint8_t);
  int (*char_available)(void *);
  uint8_t (*get_byte)(void *);
};
struct uart_periph { struct test_device device; };
struct NedCoor_i { int32_t x, y, z; };
struct test_position { int32_t lat, lon, alt; };
struct test_attitude { int32_t phi, theta, psi; };
struct test_utm { float alt; };
static struct { float alt_agl_f; } state = {40};
float ground_alt = 100;
static struct test_position position = {488100000, 78530000, 140000};
static struct test_attitude attitude = {100, -200, 300};
static struct test_utm utm_position = {140};
static struct NedCoor_i velocity = {1000, -2000, 3000};
struct test_gps {
  uint32_t tow, week, hacc, vacc, sacc, fix, num_sv, valid_fields;
} gps = {123456, 2435, 100, 200, 30, 3, 18, 127};

static uint8_t transmitted[1024];
static size_t transmitted_size;
static uint8_t incoming[256];
static size_t incoming_size, incoming_index;
static int free_space = 256;
static uint16_t space_requested;
static uint32_t clock_value;
static unsigned int clock_reads, position_reports, periodic_calls;
int dc_photo_nr = 20;

static void put_byte(void *unused, long descriptor, uint8_t value)
{
  (void)unused;
  (void)descriptor;
  assert(transmitted_size < sizeof(transmitted));
  transmitted[transmitted_size++] = value;
}
static int char_available(void *unused) { (void)unused; return incoming_index < incoming_size; }
static uint8_t get_byte(void *unused) { (void)unused; return incoming[incoming_index++]; }

static int check_free_space(void *parent, long *descriptor, uint16_t length)
{
  assert(parent != NULL && descriptor == NULL);
  space_requested = length;
  return free_space >= length ? free_space : 0;
}
static struct uart_periph test_uart = {
  .device = {
    .check_free_space = check_free_space,
    .periph = &test_uart,
    .put_byte = put_byte,
    .char_available = char_available,
    .get_byte = get_byte
  }
};
uint32_t get_sys_time_usec(void) { ++clock_reads; return clock_value++; }
struct test_position *stateGetPositionLla_i(void) { return &position; }
struct test_attitude *stateGetNedToBodyEulers_i(void) { return &attitude; }
struct test_utm *stateGetPositionUtm_f(void) { return &utm_position; }
struct NedCoor_i *stateGetSpeedNed_i(void) { return &velocity; }
int32_t stateGetHorizontalSpeedNorm_i(void) { return 2500; }
int32_t stateGetHorizontalSpeedDir_i(void) { return 400; }
void dc_periodic(void) { ++periodic_calls; }
void dc_send_shot_position(void) { ++position_reports; ++dc_photo_nr; }
void dc_send_command_common(uint8_t command) { (void)command; }

#include "../../uart_cam_ctrl.c"

static struct catia_transport parse_transmitted(void)
{
  struct catia_transport transport = {0};
  for (size_t index = 0; index < transmitted_size; ++index) parse_catia(&transport, transmitted[index]);
  assert(transport.msg_received && transport.error == 0);
  return transport;
}

static void request_clock(uint32_t low, uint32_t high, size_t length)
{
  union catia_clock_request_union token = {.data = {low, high}};
  transmitted_size = 0;
  CatiaHeader(CATIA_CLOCK_REQUEST, length);
  for (size_t index = 0; index < length; ++index) CatiaPutUint8(token.bin[index]);
  CatiaTrailer();
  incoming_size = transmitted_size;
  incoming_index = 0;
  for (size_t index = 0; index < incoming_size; ++index) incoming[index] = transmitted[index];
  transmitted_size = 0;
  digital_cam_uart_event();
}

int main(void)
{
#ifdef EXPECTED_CAMERA_MASK
  assert(digital_cam_uart_camera_mask == EXPECTED_CAMERA_MASK);
#endif
  digital_cam_uart_init();
  clock_value = UINT32_MAX;
  digital_cam_uart_periodic();
  assert(periodic_calls == 1 && dc_photo_nr == 20 && position_reports == 0);
#if DIGITAL_CAM_UART_POSE_STREAM
  assert(transmitted_size == CatiaSizeOf(CATIA_POSE_SAMPLE_MSG_SIZE));
  assert(clock_reads == 2 && space_requested == transmitted_size + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE));
  struct catia_transport transport = parse_transmitted();
  assert(transport.msg_id == CATIA_POSE_SAMPLE && transport.payload_len == CATIA_POSE_SAMPLE_MSG_SIZE);
  union catia_pose_sample_union sample = {0};
  for (size_t index = 0; index < sizeof(sample.bin); ++index) sample.bin[index] = transport.payload[index];
  assert(sample.data.sequence == 0 && sample.data.sample_begin_us == UINT32_MAX && sample.data.sample_end_us == 0);
  assert(sample.data.shot.data.nr == 21 && sample.data.shot.data.lat == position.lat);
  assert(sample.data.shot.data.theta == attitude.theta && sample.data.shot.data.groundalt == 10240);
  assert(sample.data.velocity_north_bfp == velocity.x && sample.data.velocity_east_bfp == velocity.y);
  assert(sample.data.velocity_down_bfp == velocity.z);
#if USE_GPS
  assert(sample.data.flags == CATIA_POSE_SAMPLE_GPS_PRESENT);
  assert(sample.data.gps_tow_ms == gps.tow && sample.data.gps_hacc_cm == gps.hacc);
  assert(sample.data.gps_fix == gps.fix && sample.data.gps_num_sv == gps.num_sv);
#else
  assert(sample.data.flags == 0 && sample.data.gps_hacc_cm == 0 && sample.data.gps_tow_ms == 0);
#endif
  transmitted_size = 0;
  free_space = CatiaSizeOf(CATIA_POSE_SAMPLE_MSG_SIZE);
  digital_cam_uart_periodic();
  assert(transmitted_size == 0 && clock_reads == 2 && dc_photo_nr == 20);
  free_space = 256;
  digital_cam_uart_periodic();
  transport = parse_transmitted();
  for (size_t index = 0; index < sizeof(sample.bin); ++index) sample.bin[index] = transport.payload[index];
  assert(sample.data.sequence == 2);
#else
  assert(transmitted_size == 0 && clock_reads == 0 && space_requested == 0);
#endif
  transmitted_size = 0;
  digital_cam_uart_shoot(CATIA_CAMERA_LWIRCAM, false);
  struct catia_transport shot = parse_transmitted();
  assert(shot.msg_id == CATIA_SHOOT_TARGETED && shot.payload_len == CATIA_SHOOT_TARGETED_MSG_SIZE);
  assert(dc_photo_nr == 21 && position_reports == 0);
  transmitted_size = 0;
  digital_cam_uart_shoot(CATIA_CAMERA_ALL, true);
  shot = parse_transmitted();
  assert(shot.msg_id == CATIA_SHOOT && shot.payload_len == CATIA_SHOOT_MSG_SIZE);
  assert(dc_photo_nr == 22 && position_reports == 1);
  transmitted_size = 0;
  digital_cam_uart_stop(CATIA_CAMERA_EARCAM, true);
  shot = parse_transmitted();
  assert(shot.msg_id == CATIA_STOP_TARGETED);
  assert(shot.payload[0] == CATIA_CAMERA_EARCAM && shot.payload[1] == 1);
  clock_value = UINT32_MAX;
  request_clock(123, 456, CATIA_CLOCK_REQUEST_MSG_SIZE);
#if DIGITAL_CAM_UART_POSE_STREAM
  shot = parse_transmitted();
  assert(shot.msg_id == CATIA_CLOCK_REPLY && shot.payload_len == CATIA_CLOCK_REPLY_MSG_SIZE);
  union catia_clock_reply_union reply;
  for (size_t index = 0; index < sizeof(reply.bin); ++index) reply.bin[index] = shot.payload[index];
  assert(reply.data.request.data.token_low == 123 && reply.data.request.data.token_high == 456);
  assert(reply.data.receive_us == UINT32_MAX && reply.data.transmit_us == 0);
  transmitted_size = 0;
  digital_cam_uart_periodic();
  shot = parse_transmitted();
  assert(shot.msg_id == CATIA_POSE_CLOCKED && shot.payload_len == CATIA_POSE_CLOCKED_MSG_SIZE);
  assert(space_requested == CatiaSizeOf(CATIA_POSE_CLOCKED_MSG_SIZE) + CatiaSizeOf(CATIA_SHOOT_TARGETED_MSG_SIZE));
  for (size_t index = 0; index < sizeof(reply.data.request.bin); ++index) {
    assert(shot.payload[CATIA_POSE_SAMPLE_MSG_SIZE + index] == reply.data.request.bin[index]);
  }
  request_clock(999, 111, CATIA_CLOCK_REQUEST_MSG_SIZE);
  assert(transmitted_size == 0 && pose_clock_token.data.token_low == 123);
  clock_value = 600000;
  request_clock(999, 111, CATIA_CLOCK_REQUEST_MSG_SIZE - 1);
  assert(transmitted_size == 0 && pose_clock_token.data.token_low == 123);
  request_clock(0, 0, CATIA_CLOCK_REQUEST_MSG_SIZE);
  assert(transmitted_size == 0 && pose_clock_token.data.token_low == 123);
  free_space = CatiaSizeOf(CATIA_CLOCK_REPLY_MSG_SIZE);
  request_clock(999, 111, CATIA_CLOCK_REQUEST_MSG_SIZE);
  assert(transmitted_size == 0 && pose_clock_token.data.token_low == 123);
  free_space = 256;
  request_clock(999, 111, CATIA_CLOCK_REQUEST_MSG_SIZE);
  assert(transmitted_size > 0 && pose_clock_token.data.token_low == 999);
  digital_cam_uart_init();
  transmitted_size = 0;
  digital_cam_uart_periodic();
  shot = parse_transmitted();
  assert(shot.msg_id == CATIA_POSE_SAMPLE && !clock_reply_sent);
#else
  assert(transmitted_size == 0);
#endif
  assert(dc_photo_nr == 22 && position_reports == 1);
  for (unsigned selection = 0; selection <= 255; ++selection) {
    transmitted_size = 0;
    int previous_photo_nr = dc_photo_nr;
    unsigned int previous_reports = position_reports;
    assert(uart_cam_ctrl_set_camera_mask(selection));
    assert(digital_cam_uart_camera_mask == selection);
    assert(transmitted_size == 0 && dc_photo_nr == previous_photo_nr);
    assert(position_reports == previous_reports);
    const float invalid[] = {-1, 256, 2.5f, NAN, INFINITY, -INFINITY};
    for (size_t invalid_index = 0; invalid_index < sizeof(invalid) / sizeof(invalid[0]); ++invalid_index) {
      assert(!uart_cam_ctrl_set_camera_mask(invalid[invalid_index]));
      assert(digital_cam_uart_camera_mask == selection);
      assert(transmitted_size == 0 && dc_photo_nr == previous_photo_nr);
    }
    dc_send_command(DC_SHOOT);
    if (selection == 0) {
      assert(transmitted_size == 0 && dc_photo_nr == previous_photo_nr);
      assert(position_reports == previous_reports);
      continue;
    }
    shot = parse_transmitted();
    assert(dc_photo_nr == previous_photo_nr + 1 && position_reports == previous_reports + 1);
    assert(shot.msg_id == CATIA_SHOOT_MASK && shot.payload_len == CATIA_SHOOT_MASK_MSG_SIZE);
    union dc_shot_mask_union masked;
    for (size_t byte_index = 0; byte_index < sizeof(masked.bin); ++byte_index) {
      masked.bin[byte_index] = shot.payload[byte_index];
    }
    assert(masked.data.camera_mask == selection);
    assert(masked.data.shot.data.nr == previous_photo_nr + 1);
  }
  transmitted_size = 0;
  int previous_photo_nr = dc_photo_nr;
  unsigned int previous_reports = position_reports;
  for (unsigned camera_id = 0; camera_id <= 8; ++camera_id) {
    assert(uart_cam_ctrl_set_camera(camera_id));
    assert(digital_cam_uart_camera_mask == (camera_id == 0 ? 255 : 1U << (camera_id - 1)));
    assert(!uart_cam_ctrl_set_camera(9));
    assert(!uart_cam_ctrl_set_camera(-1));
    assert(!uart_cam_ctrl_set_camera(2.5f));
    assert(!uart_cam_ctrl_set_camera(NAN));
  }
  assert(transmitted_size == 0 && dc_photo_nr == previous_photo_nr && position_reports == previous_reports);
  assert(uart_cam_ctrl_set_camera(CATIA_CAMERA_AICAM));
  (void)state;
  puts("Actual FC sender: pose, backpressure, runtime camera selection and legacy commands passed");
}