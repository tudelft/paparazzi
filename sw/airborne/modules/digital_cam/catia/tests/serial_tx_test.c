#define _GNU_SOURCE
#include "../serial_tx.c"
#include <assert.h>
#include <stdio.h>
#include <sys/socket.h>

static bool fragment_writes;
static unsigned int fragment_calls;
static size_t observed_short_writes;

ssize_t __real_write(int descriptor, const void *buffer, size_t length);

ssize_t __wrap_write(int descriptor, const void *buffer, size_t length)
{
  if (fragment_writes) {
    ++fragment_calls;
    if (fragment_calls % 3 == 0) {
      errno = EAGAIN;
      return -1;
    }
    if (fragment_calls % 7 == 0) {
      errno = EINTR;
      return -1;
    }
    if (length > 3) {
      length = 3;
      ++observed_short_writes;
    }
  }
  return __real_write(descriptor, buffer, length);
}

int main(void)
{
  int sockets[2];
  assert(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sockets) == 0);
  int buffer_size = 1024;
  assert(setsockopt(sockets[0], SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size)) == 0);
  assert(serial_tx_start(sockets[0]) == 0);
  assert(serial_tx_start(sockets[0]) == -1 && errno == EBUSY);
  pthread_mutex_lock(&transmit_mutex);
  assert(serial_tx_send_if_idle(CATIA_CLOCK_REQUEST, NULL, 0) == -1 && errno == EAGAIN);
  pthread_mutex_unlock(&transmit_mutex);
  uint8_t payload[250];
  unsigned int accepted = 0, received = 0;
  for (; accepted < 1000; ++accepted) {
    for (size_t index = 0; index < sizeof(payload); ++index) payload[index] = (uint8_t)accepted;
    if (serial_tx_send(CATIA_PAYLOAD, payload, sizeof(payload)) != 0) break;
  }
  assert(accepted > 1 && accepted < 1000 && errno == ENOBUFS && serial_tx_pending());
  const size_t queued = count;
  assert(serial_tx_send_if_idle(CATIA_CLOCK_REQUEST, NULL, 0) == -1 && errno == EAGAIN);
  assert(count == queued);
  struct catia_transport transport = {0};
  for (size_t attempt = 0; received < accepted && attempt < 10000; ++attempt) {
    uint8_t bytes[71];
    ssize_t length = read(sockets[1], bytes, sizeof(bytes));
    if (length < 0) assert(errno == EAGAIN || errno == EWOULDBLOCK);
    for (ssize_t index = 0; index < length; ++index) {
      parse_catia(&transport, bytes[index]);
      assert(transport.error == 0);
      if (transport.msg_received) {
        assert(transport.msg_id == CATIA_PAYLOAD && transport.payload_len == 250);
        for (size_t offset = 0; offset < 250; ++offset) assert(transport.payload[offset] == (uint8_t)received);
        ++received;
        transport.msg_received = false;
      }
    }
    assert(serial_tx_flush() == 0);
  }
  assert(received == accepted && !serial_tx_pending());
  fragment_writes = true;
  for (size_t index = 0; index < sizeof(payload); ++index) payload[index] = (uint8_t)index;
  assert(serial_tx_send(CATIA_PAYLOAD, payload, sizeof(payload)) == 0);
  assert(serial_tx_send(CATIA_STATUS, payload, CATIA_STATUS_MSG_SIZE) == 0);
  assert(serial_tx_pending());
  unsigned int packets = 0;
  transport = (struct catia_transport){0};
  for (size_t attempt = 0; packets < 2 && attempt < 1000; ++attempt) {
    uint8_t bytes[71];
    ssize_t length = read(sockets[1], bytes, sizeof(bytes));
    if (length < 0) assert(errno == EAGAIN || errno == EWOULDBLOCK);
    for (ssize_t index = 0; index < length; ++index) {
      parse_catia(&transport, bytes[index]);
      assert(transport.error == 0);
      if (transport.msg_received) {
        assert(packets < 2);
        assert(transport.msg_id == (packets == 0 ? CATIA_PAYLOAD : CATIA_STATUS));
        assert(transport.payload_len == (packets == 0 ? 250 : CATIA_STATUS_MSG_SIZE));
        for (size_t offset = 0; offset < transport.payload_len; ++offset) assert(transport.payload[offset] == (uint8_t)offset);
        ++packets;
        transport.msg_received = false;
      }
    }
    assert(serial_tx_flush() == 0);
  }
  assert(packets == 2 && observed_short_writes > 0 && !serial_tx_pending());
  fragment_writes = false;
  assert(serial_tx_send(1, NULL, 1) == -1 && errno == EINVAL);
  assert(serial_tx_send(1, payload, 251) == -1 && errno == EINVAL);
  assert(serial_tx_send_if_idle(CATIA_BUFFER_EMPTY, NULL, 0) == 0);
  uint8_t empty[5];
  assert(read(sockets[1], empty, sizeof(empty)) == 5);
  for (size_t index = 0; index < sizeof(empty); ++index) parse_catia(&transport, empty[index]);
  assert(transport.msg_received && transport.payload_len == 0 && transport.error == 0);
  serial_tx_stop();
  assert(serial_tx_send(1, NULL, 0) == -1 && errno == ENOTCONN);
  assert(close(sockets[0]) == 0 && close(sockets[1]) == 0);
  assert(pipe(sockets) == 0);
  assert(serial_tx_start(sockets[1]) == -1 && errno == EINVAL);
  close(sockets[0]);
  close(sockets[1]);
  puts("UART queue: overload, idle-only probes, fragmented writes, EAGAIN/EINTR and packet order passed");
}