#include "../pose_log.c"
#include <assert.h>
#include <dirent.h>
#include <string.h>

static union catia_pose_sample_union sample(void)
{
  union catia_pose_sample_union result = {0};
  result.data.sequence = 7;
  result.data.sample_begin_us = UINT32_MAX;
  result.data.sample_end_us = 5;
  result.data.shot.data.nr = 21;
  result.data.shot.data.lat = 488100000;
  result.data.shot.data.lon = 78530000;
  result.data.shot.data.alt = 140000;
  result.data.velocity_north_bfp = -1234;
  result.data.gps_hacc_cm = 100;
  result.data.flags = CATIA_POSE_SAMPLE_GPS_PRESENT;
  return result;
}

static void remove_logs(const char *directory, bool check_contents)
{
  DIR *listing = opendir(directory);
  assert(listing != NULL);
  struct dirent *entry;
  int files = 0;
  while ((entry = readdir(listing)) != NULL) {
    if (entry->d_name[0] == '.') continue;
    char path[PATH_MAX];
    assert(snprintf(path, sizeof(path), "%s/%s", directory, entry->d_name) > 0);
    if (check_contents) {
      FILE *file = fopen(path, "r");
      assert(file != NULL);
      char line[2048];
      assert(fgets(line, sizeof(line), file) != NULL && strcmp(line, header) == 0);
      assert(fgets(line, sizeof(line), file) != NULL);
      char prefix[200];
      int length = snprintf(prefix, sizeof(prefix), "2,%s,123456789,1,0,7,4294967295,5,21,488100000,78530000,140000,", boot_id);
      assert(length > 0 && (size_t)length < sizeof(prefix) && strncmp(line, prefix, (size_t)length) == 0);
      assert(strstr(line, ",-1234,0,0,0,0,100,0,0,0,0,0,1,10,20,1,123456000,123456700,123450000,123451000,200,300,1000\n") != NULL);
      assert(fgets(line, sizeof(line), file) == NULL && fclose(file) == 0);
    }
    assert(unlink(path) == 0);
    ++files;
  }
  assert(closedir(listing) == 0 && files == 1);
}

int main(void)
{
  char directory[] = "/tmp/catia-pose-log.XXXXXX";
  assert(mkdtemp(directory) != NULL);
  assert(pose_log_start(NULL) == 0);
  union catia_pose_sample_union pose = sample();
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == 0);
  assert(pose_log_start("/nonexistent/catia-pose-test") == -1);
  assert(pose_log_start(directory) == 0);
  assert(strlen(boot_id) == 36);
  assert(pose_log_start(directory) == -1 && errno == EBUSY);
  assert(pose_log_record(pose.bin, sizeof(pose.bin) - 1, 123) == -1);
  assert(pose_log_record(NULL, sizeof(pose.bin), 123) == -1);
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 0) == -1);
  pose.data.flags = 2;
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == -1);
  pose = sample();
  pose.data.shot.data.lat = 900000001;
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == -1);
  pose = sample();
  pose.data.sample_end_us = 2000000;
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == -1);
  pose = sample();
  struct pose_clock_evidence evidence = {
    .token = {.data = {10, 20}}, .sample_time = {123456000, 123456700},
    .probe_sent_us = 123450000, .probe_received_us = 123451000,
    .probe_receive_fc_us = 200, .probe_transmit_fc_us = 300, .mapped = true
  };
  assert(pose_log_record_clocked(pose.bin, sizeof(pose.bin), 123456789, &evidence) == 1);
  evidence.sample_time.latest_us = 123456790;
  assert(pose_log_record_clocked(pose.bin, sizeof(pose.bin), 123456789, &evidence) == -1);
  pthread_mutex_lock(&mutex);
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 456) == 0);
  pthread_mutex_unlock(&mutex);
  assert(pose_log_stop() == 0);
  struct pose_log_stats status = pose_log_status();
  assert(status.accepted == 1 && status.synced == 1 && status.rejected == 7 && status.dropped == 1);
  assert(status.error == 0);
  remove_logs(directory, true);
  active = true;
  stopping = false;
  count = POSE_LOG_CAPACITY;
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 789) == 0);
  assert(pose_log_status().dropped == 2);
  active = false;
  count = 0;
  assert(pose_log_start(directory) == 0);
  bytes_written = POSE_LOG_MAX_BYTES;
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == 1);
  assert(pose_log_stop() == -1 && errno == EFBIG);
  assert(pose_log_status().synced == 0);
  remove_logs(directory, false);
  assert(pose_log_start(directory) == 0);
  assert(fclose(output) == 0);
  output = fopen("/dev/full", "w");
  assert(output != NULL);
  assert(pose_log_record(pose.bin, sizeof(pose.bin), 123) == 1);
  assert(pose_log_stop() == -1 && pose_log_status().error == ENOSPC);
  assert(pose_log_status().synced == 0);
  remove_logs(directory, false);
  assert(pose_log_stop() == 0);
  assert(rmdir(directory) == 0);
  puts("Bounded pose log: CSV, validation, drops, disk/size errors and synced shutdown passed");
}