#define main catia_application_main
#include "../catia.c"
#undef main

#include <assert.h>
#include <limits.h>
#include <libexif/exif-data.h>

int main(int argc, char **argv)
{
  assert(argc == 4);
  assert(lwir_cam_pipe_set_calibration(NULL) != 0);
  assert(lwir_cam_pipe_set_calibration("") != 0);
  assert(lwir_cam_pipe_set_calibration(argv[3]) == 0);
  int uart[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, uart) == 0);
  assert(serial_tx_start(uart[0]) == 0);
  assert(pose_log_start(argv[2]) == 0);
  union catia_pose_sample_union sample = {0};
  sample.data.sequence = 42;
  sample.data.shot.data.lat = 488100000;
  sample.data.shot.data.lon = 78530000;
  catia_protocol.msg_id = CATIA_POSE_SAMPLE;
  catia_protocol.payload_len = CATIA_POSE_SAMPLE_MSG_SIZE;
  for (size_t index = 0; index < sizeof(sample.bin); ++index) catia_protocol.payload[index] = sample.bin[index];
  serial_receive_monotonic_us = 123456789;
  handle_received_message();
  assert(pose_log_status().accepted + pose_log_status().dropped == 1);
  assert(shooting_count == 0 && shooting_thread_count == 0);
  assert(pose_log_stop() == 0);
  assert(camera_backend_select(CAMERA_BACKEND_LWIR_CAM, true) == 0);
  assert(camera.init(argv[1]) == 0);
  camera_initialized[CATIA_CAMERA_LWIRCAM] = true;
  camera_source_image = argv[1];
  test_capture_enabled = true;
  optical_camera_id = CATIA_CAMERA_LWIRCAM;

  union dc_shot_targeted_union message = {0};
  message.data.shot.data.nr = 39;
  message.data.shot.data.lat = 434639600;
  message.data.shot.data.lon = 12728610;
  message.data.shot.data.alt = 50000;
  message.data.shot.data.groundalt = 12800;
  message.data.camera_id = 99;
  catia_protocol.msg_id = CATIA_SHOOT_TARGETED;
  catia_protocol.payload_len = CATIA_SHOOT_TARGETED_MSG_SIZE;
  for (size_t index = 0; index < sizeof(message.bin); ++index) {
    catia_protocol.payload[index] = message.bin[index];
  }
  handle_received_message();
  assert(shooting_count == 0 && shooting_thread_count == 0);

  message.data.shot.data.nr = 38;
  message.data.camera_id = CATIA_CAMERA_AICAM;
  for (size_t index = 0; index < sizeof(message.bin); ++index) {
    catia_protocol.payload[index] = message.bin[index];
  }
  handle_received_message();
  message.data.shot.data.nr = 39;
  message.data.camera_id = CATIA_CAMERA_LWIRCAM;
  for (size_t index = 0; index < sizeof(message.bin); ++index) {
    catia_protocol.payload[index] = message.bin[index];
  }
  handle_received_message();
  pthread_mutex_lock(&mut);
  while (shooting_thread_count != 0) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  assert(shooting_count == 2);
  assert(optical_camera_id == CATIA_CAMERA_LWIRCAM);

  char image_path[PATH_MAX];
  int length = snprintf(image_path, sizeof(image_path), "%s/m000039.jpg", argv[2]);
  assert(length > 0 && (size_t)length < sizeof(image_path));
  ExifData *metadata = exif_data_new_from_file(image_path);
  assert(metadata != NULL);
  ExifEntry *altitude = exif_content_get_entry(metadata->ifd[EXIF_IFD_GPS], EXIF_TAG_GPS_ALTITUDE);
  assert(altitude != NULL && altitude->size == 8);
  ExifRational value = exif_get_rational(altitude->data, exif_data_get_byte_order(metadata));
  assert(value.denominator != 0 && value.numerator / (double)value.denominator == 50.0);
  ExifEntry *hotspots = exif_content_get_entry(metadata->ifd[EXIF_IFD_EXIF], EXIF_TAG_USER_COMMENT);
  assert(hotspots != NULL && hotspots->size > 8 && hotspots->data[hotspots->size - 1] == 0);
  const char *information = (const char *)hotspots->data + 8;
  assert(strstr(information, "LWIR_HOTSPOTS_V1") != NULL);
  assert(strstr(information, "count=2;") != NULL);
  assert(strstr(information, "center_temperature_c=") != NULL);
  assert(strstr(information, "location_status=estimated; latitude_deg=") != NULL);
  exif_data_unref(metadata);
  local_mode = true;
  assert(run_soda(image_path, &message.data.shot, CATIA_CAMERA_CHDK) == 0);
  assert(run_soda(image_path, &message.data.shot, CATIA_CAMERA_EARCAM) == 0);
  assert(run_soda(image_path, &message.data.shot, -1) == -1);
  local_mode = false;

  struct capture_job *shot = malloc(sizeof(*shot));
  assert(shot != NULL);
  shot->shot = message.data.shot;
  shot->shot.data.nr = 40;
  shot->camera_mask = CATIA_CAMERA_MASK_AICAM;
  shot->ticket = next_job_ticket++;
  shooting_thread_count = 1;
  handle_msg_shoot(shot);
  cameras_deinit();
  puts("CATIA LWIR/AICam integration tests passed");
  union dc_shot_union moving = message.data.shot;
  moving.data.vground = (int32_t)(15 * SPEED_BFP_SCALE);
  moving.data.course = 0;
  const struct capture_timing capture_times = {
    .request_monotonic_us = 1000000, .arrival_monotonic_us = 1320000,
    .callback_sequence = 200, .callback_drops = 3, .callback_arrival = true
  };
  assert(image_exif_write_capture(image_path, &moving, 3, 1, &capture_times) == 0);
  metadata = exif_data_new_from_file(image_path);
  assert(metadata != NULL);
  ExifEntry *description = exif_content_get_entry(metadata->ifd[EXIF_IFD_0], EXIF_TAG_IMAGE_DESCRIPTION);
  assert(description != NULL && description->data[description->size - 1] == 0);
  information = (const char *)description->data;
  assert(strstr(information, "original_lat_deg=43.4639600") != NULL);
  assert(strstr(information, "request_to_frame_arrival_s=0.320000000") != NULL);
  assert(strstr(information, "position_compensation=constant_ground_velocity_estimate") != NULL);
  assert(strstr(information, "camera_request_monotonic_us=1000000") != NULL);
  assert(strstr(information, "frame_arrival_monotonic_us=1320000") != NULL);
  assert(strstr(information, "capture_time_kind=sdk_callback_not_exposure") != NULL);
  assert(strstr(information, "timing_reference=server_request_to_callback") != NULL);
  assert(strstr(information, "callback_sequence=200; callback_drops=3") != NULL);
  assert(strstr(information, "mora_boot_id=") != NULL);
  ExifEntry *latitude_tag = exif_content_get_entry(metadata->ifd[EXIF_IFD_GPS], EXIF_TAG_GPS_LATITUDE);
  assert(latitude_tag != NULL && latitude_tag->size == 24);
  double latitude = 0;
  double divisor = 1;
  for (size_t index = 0; index < 3; ++index) {
    value = exif_get_rational(latitude_tag->data + index * 8, exif_data_get_byte_order(metadata));
    assert(value.denominator != 0);
    latitude += value.numerator / (double)value.denominator / divisor;
    divisor *= 60;
  }
  assert(latitude > 43.464002 && latitude < 43.464005);
  exif_data_unref(metadata);
  assert(moving.data.lat == message.data.shot.data.lat);
  assert(image_exif_write_timed(image_path, &moving, 3, 1) == 0);
  metadata = exif_data_new_from_file(image_path);
  assert(metadata != NULL);
  description = exif_content_get_entry(metadata->ifd[EXIF_IFD_0], EXIF_TAG_IMAGE_DESCRIPTION);
  assert(description != NULL);
  assert(strstr((const char *)description->data, "position_compensation=not_applied") != NULL);
  assert(strstr((const char *)description->data, "frame_arrival_monotonic_us=") == NULL);
  exif_data_unref(metadata);
  puts("Timed EXIF compensation and provenance tests passed");
  uint8_t transmitted[256];
  const ssize_t bytes = read(uart[1], transmitted, sizeof(transmitted));
  assert(bytes > 0);
  struct catia_transport status_frame = {0};
  unsigned status_count = 0;
  for (ssize_t index = 0; index < bytes; ++index) {
    parse_catia(&status_frame, transmitted[index]);
    if (status_frame.msg_received) {
      assert(status_frame.msg_id == CATIA_STATUS && status_frame.error == 0);
      ++status_count;
      status_frame.msg_received = false;
    }
  }
  assert(status_count == 2);
  serial_tx_stop();
  close(uart[0]);
  close(uart[1]);
  return 0;
}