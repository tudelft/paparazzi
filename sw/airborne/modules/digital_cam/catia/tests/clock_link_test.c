#define main catia_application_main
#include "../catia.c"
#undef main
#include <assert.h>
#include <dirent.h>
#include <inttypes.h>

static void receive_message(uint8_t message, const uint8_t *payload, size_t length, uint64_t received_us)
{
  assert(length <= sizeof(catia_protocol.payload));
  catia_protocol.msg_id = message;
  catia_protocol.payload_len = (uint8_t)length;
  for (size_t index = 0; index < length; ++index) catia_protocol.payload[index] = payload[index];
  serial_receive_monotonic_us = received_us;
  handle_received_message();
}

static union catia_clock_request_union read_request(int input)
{
  uint8_t bytes[CatiaSizeOf(CATIA_CLOCK_REQUEST_MSG_SIZE)];
  assert(read(input, bytes, sizeof(bytes)) == (ssize_t)sizeof(bytes));
  struct catia_transport message = {0};
  for (size_t index = 0; index < sizeof(bytes); ++index) parse_catia(&message, bytes[index]);
  assert(message.msg_received && message.error == 0 && message.msg_id == CATIA_CLOCK_REQUEST);
  assert(message.payload_len == CATIA_CLOCK_REQUEST_MSG_SIZE);
  union catia_clock_request_union token;
  for (size_t index = 0; index < sizeof(token.bin); ++index) token.bin[index] = message.payload[index];
  return token;
}

int main(void)
{
  char directory[] = "/tmp/catia-clock-link.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  int uart[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, uart) == 0);
  assert(serial_tx_start(uart[0]) == 0);
  assert(pose_log_start(directory) == 0);
  send_clock_probe(monotonic_time_us());
  assert(clock_probe_count == 0);
  clock_probes_enabled = true;
  uint64_t now = monotonic_time_us();
  send_clock_probe(now);
  assert(clock_probe_count == 1 && fc_clock.pending);
  union catia_clock_request_union token = read_request(uart[1]);
  send_clock_probe(now);
  assert(clock_probe_count == 1);
  uint64_t sent = fc_clock.sent_us;
  union catia_clock_reply_union reply = {.data = {token, UINT32_MAX - 1000, UINT32_MAX - 500}};
  receive_message(CATIA_CLOCK_REPLY, reply.bin, sizeof(reply.bin), sent + 10000);
  assert(clock_reply_count == 1 && fc_clock.valid && !fc_clock.pending);
  assert(fc_clock.mapped_sent_us == sent);
  union catia_pose_clocked_union pose = {0};
  pose.data.request = token;
  pose.data.sample.data.sequence = 3;
  pose.data.sample.data.sample_begin_us = 49499;
  pose.data.sample.data.sample_end_us = 49509;
  pose.data.sample.data.shot.data.lat = 488100000;
  pose.data.sample.data.shot.data.lon = 78530000;
  receive_message(CATIA_POSE_CLOCKED, pose.bin, sizeof(pose.bin), sent + 61000);
  assert(pose_log_status().accepted == 1 && shooting_count == 0 && shooting_thread_count == 0);
  receive_message(CATIA_CLOCK_REPLY, reply.bin, sizeof(reply.bin), sent + 62000);
  assert(clock_reply_count == 1 && clock_rejected_count == 1);
  receive_message(CATIA_POSE_CLOCKED, pose.bin, sizeof(pose.bin) - 1, sent + 62000);
  assert(pose_log_status().rejected == 1);
  pose.data.sample.data.sequence = 4;
  receive_message(CATIA_POSE_CLOCKED, pose.bin, sizeof(pose.bin), sent + 2100001);
  assert(pose_log_status().accepted == 2);
  receive_message(CATIA_POSE_SAMPLE, pose.data.sample.bin, sizeof(pose.data.sample.bin), sent + 2200000);
  assert(!fc_clock.valid && pose_log_status().accepted == 3);
  assert(shooting_count == 0 && shooting_thread_count == 0);
  assert(pose_log_stop() == 0);
  assert(pose_log_status().synced == 3 && pose_log_status().dropped == 0);

  DIR *files = opendir(directory);
  assert(files != NULL);
  struct dirent *entry;
  int count_files = 0;
  while ((entry = readdir(files)) != NULL) {
    if (entry->d_name[0] == '.') continue;
    char path[4096];
    assert(snprintf(path, sizeof(path), "%s/%s", directory, entry->d_name) > 0);
    FILE *file = fopen(path, "r");
    assert(file != NULL);
    char line[2048];
    assert(fgets(line, sizeof(line), file) != NULL);
    assert(strstr(line, "sample_earliest_monotonic_us") != NULL);
    for (int row = 0; row < 3; ++row) {
      assert(fgets(line, sizeof(line), file) != NULL);
      uint64_t fields[40] = {0};
      char *context = NULL;
      size_t column = 0;
      for (char *value = strtok_r(line, ",", &context); value != NULL; value = strtok_r(NULL, ",", &context)) {
        assert(column < 40);
        if (column != 1) fields[column] = strtoull(value, NULL, 10);
        ++column;
      }
      assert(column == 40 && fields[0] == 2);
      if (row == 0) {
        assert(fields[30] == token.data.token_low && fields[31] == token.data.token_high && fields[32] == 1);
        const uint64_t actual_begin = sent + 55000, actual_end = actual_begin + 10;
        assert(fields[33] <= actual_begin && fields[34] >= actual_end);
        assert(fields[35] == sent && fields[36] == sent + 10000);
      } else {
        assert(fields[32] == 0 && fields[33] == 0 && fields[34] == 0 && fields[35] == 0 && fields[36] == 0);
        assert(fields[30] == (row == 1 ? token.data.token_low : 0));
      }
      assert(fields[39] == CLOCK_ALIGNMENT_DRIFT_PPM);
    }
    assert(fgets(line, sizeof(line), file) == NULL && fclose(file) == 0);
    assert(unlink(path) == 0);
    ++count_files;
  }
  assert(closedir(files) == 0 && count_files == 1 && rmdir(directory) == 0);
  serial_tx_stop();
  close(uart[0]);
  close(uart[1]);
  puts("Clock link: real probe, reply, wrapped pose mapping, stale/reset rejection and CSV passed");
}