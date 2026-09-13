#include "../lwir_cam_pipe.c"
#include <assert.h>

static void check_response(const char *response, double expected)
{
  int descriptors[2];
  assert(pipe(descriptors) == 0);
  assert(write(descriptors[1], response, strlen(response)) == (ssize_t)strlen(response));
  close(descriptors[1]);
  capture_server_output = descriptors[0];
  capture_delay_s = -1;
  capture_times = (struct capture_timing){0};
  assert(read_server_status("LWIR_SERVER_OK", 100) == 0);
  assert(lwir_cam_pipe_capture_delay() == expected);
  close(descriptors[0]);
  capture_server_output = -1;
}

int main(void)
{
  check_response("LWIR_SERVER_DELAY 0.32\nLWIR_SERVER_OK\n", .32);
  check_response("LWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_DELAY nan\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_DELAY 21\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_DELAY 0.32 junk\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 1000000 1320000\nLWIR_SERVER_OK\n", .32);
  assert(lwir_cam_pipe_capture_timing().arrival_monotonic_us == 1320000);
  check_response("LWIR_SERVER_TIMING 1000000 1320000\nLWIR_SERVER_DELAY 4\nLWIR_SERVER_OK\n", .32);
  check_response("LWIR_SERVER_DELAY 4\nLWIR_SERVER_TIMING 1000000 1320000\nLWIR_SERVER_OK\n", .32);
  check_response("LWIR_SERVER_TIMING 100 99\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 0 1\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING -1 5\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 1 20000002\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 18446744073709551616 18446744073709551617\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 1000000 1320000 junk\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 200 3\nLWIR_SERVER_OK\n", .32);
  assert(capture_times.callback_arrival && capture_times.callback_sequence == 200 && capture_times.callback_drops == 3);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 200 3\nLWIR_SERVER_TIMING 1 2\nLWIR_SERVER_OK\n", .32);
  assert(capture_times.callback_arrival);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 0 0\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 2 2\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1000000 2 0\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 2\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_CALLBACK_TIMING 1000000 1320000 2 0 junk\nLWIR_SERVER_OK\n", -1);
  check_response("LWIR_SERVER_TIMING 1000000 1320000\nLWIR_SERVER_OK\n", .32);
  assert(!capture_times.callback_arrival && capture_times.callback_sequence == 0);
  check_response("LWIR_SERVER_OK\n", -1);
  assert(!capture_timing_valid(&capture_times));

  /* The buffered reader must carry any bytes read past the matched line over to the
   * next call, exactly as the byte-at-a-time reader used to, instead of discarding or
   * re-reading them from a (possibly now-different) descriptor. */
  status_buffer_size = status_buffer_pos = 0;
  int descriptors[2];
  assert(pipe(descriptors) == 0);
  const char *combined = "LWIR_SERVER_READY\nEXTRA_LINE\n";
  assert(write(descriptors[1], combined, strlen(combined)) == (ssize_t)strlen(combined));
  close(descriptors[1]);
  capture_server_output = descriptors[0];
  assert(read_server_status("LWIR_SERVER_READY", 100) == 0);
  assert(read_server_status("EXTRA_LINE", 100) == 0);
  close(descriptors[0]);
  capture_server_output = -1;
  status_buffer_size = status_buffer_pos = 0;
}