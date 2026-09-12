#include "serial_tx.h"
#include "protocol.h"
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#define SERIAL_TX_CAPACITY 4096

static pthread_mutex_t transmit_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t pending[SERIAL_TX_CAPACITY];
static size_t head, count;
static int output_fd = -1;
static int output_error;

static int flush_locked(void)
{
  if (output_fd < 0 || output_error != 0) {
    errno = output_error != 0 ? output_error : ENOTCONN;
    return -1;
  }
  while (count > 0) {
    size_t length = SERIAL_TX_CAPACITY - head;
    if (length > count) length = count;
    ssize_t sent = write(output_fd, pending + head, length);
    if (sent > 0) {
      head = (head + (size_t)sent) % SERIAL_TX_CAPACITY;
      count -= (size_t)sent;
    } else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return 0;
    } else {
      output_error = sent < 0 ? errno : EIO;
      errno = output_error;
      return -1;
    }
  }
  return 0;
}

int serial_tx_start(int descriptor)
{
  const int flags = fcntl(descriptor, F_GETFL);
  if (flags < 0) return -1;
  if (!(flags & O_NONBLOCK)) {
    errno = EINVAL;
    return -1;
  }
  pthread_mutex_lock(&transmit_mutex);
  if (output_fd >= 0) {
    pthread_mutex_unlock(&transmit_mutex);
    errno = EBUSY;
    return -1;
  }
  output_fd = descriptor;
  output_error = 0;
  head = count = 0;
  pthread_mutex_unlock(&transmit_mutex);
  return 0;
}

void serial_tx_stop(void)
{
  pthread_mutex_lock(&transmit_mutex);
  output_fd = -1;
  output_error = 0;
  head = count = 0;
  pthread_mutex_unlock(&transmit_mutex);
}

static int send_frame(uint8_t message, const uint8_t *payload, size_t length, bool idle_only)
{
  if (length > 250 || (length > 0 && payload == NULL)) {
    errno = EINVAL;
    return -1;
  }
  uint8_t frame[255];
  const size_t frame_length = length + 5;
  frame[0] = STX;
  frame[1] = (uint8_t)frame_length;
  frame[2] = message;
  uint8_t checksum_a = frame[1], checksum_b = checksum_a;
  for (size_t index = 0; index <= length; ++index) {
    if (index > 0) frame[index + 2] = payload[index - 1];
    checksum_a += frame[index + 2];
    checksum_b += checksum_a;
  }
  frame[frame_length - 2] = checksum_a;
  frame[frame_length - 1] = checksum_b;
  if (idle_only) {
    if (pthread_mutex_trylock(&transmit_mutex) != 0) {
      errno = EAGAIN;
      return -1;
    }
    if (count > 0) {
      pthread_mutex_unlock(&transmit_mutex);
      errno = EAGAIN;
      return -1;
    }
  } else {
    pthread_mutex_lock(&transmit_mutex);
  }
  int result = flush_locked();
  if (result == 0 && frame_length > SERIAL_TX_CAPACITY - count) {
    errno = ENOBUFS;
    result = -1;
  } else if (result == 0) {
    for (size_t index = 0; index < frame_length; ++index) {
      pending[(head + count + index) % SERIAL_TX_CAPACITY] = frame[index];
    }
    count += frame_length;
    result = flush_locked();
  }
  pthread_mutex_unlock(&transmit_mutex);
  return result;
}

int serial_tx_send(uint8_t message, const uint8_t *payload, size_t length)
{
  return send_frame(message, payload, length, false);
}

int serial_tx_send_if_idle(uint8_t message, const uint8_t *payload, size_t length)
{
  return send_frame(message, payload, length, true);
}

int serial_tx_flush(void)
{
  pthread_mutex_lock(&transmit_mutex);
  int result = flush_locked();
  pthread_mutex_unlock(&transmit_mutex);
  return result;
}

bool serial_tx_pending(void)
{
  pthread_mutex_lock(&transmit_mutex);
  /** Once set, output_error never clears itself; also report pending when it is set
   * (even with an empty queue) so the caller's poll loop keeps asking for POLLOUT and
   * promptly calls serial_tx_flush(), which surfaces the failure instead of leaving
   * UART output silently and permanently stuck. */
  bool result = output_fd >= 0 && (count > 0 || output_error != 0);
  pthread_mutex_unlock(&transmit_mutex);
  return result;
}