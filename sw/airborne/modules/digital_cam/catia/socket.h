
#ifndef DIGITAL_CAM_CATIA_SOCKET_H
#define DIGITAL_CAM_CATIA_SOCKET_H

/**
 * @file socket.h
 * @brief Local non-blocking UDP channel used by CATIA's development pipeline.
 * @details This is intentionally loopback-only infrastructure for local SODA and
 * simulator integration, not a network-facing control protocol.
 */

/** @brief Create the loopback UDP endpoint.
 * @param is_server Nonzero to bind the fixed local port; zero for an unbound sender.
 * @return 0 on success or -1 when socket creation/binding fails. */
int socket_init(int is_server);
/** @brief Return the active UDP descriptor for polling.
 * @return Descriptor, or a negative value before successful initialization. */
int socket_get_fd(void);
/** @brief Receive one datagram without blocking.
 * @param buffer Destination bytes.
 * @param len Destination capacity.
 * @return recvfrom() result, including -1/EAGAIN when no datagram is ready. */
int socket_recv(char *buffer, int len);
/** @brief Send one loopback datagram without blocking.
 * @param buffer Payload bytes.
 * @param len Payload length. */
void socket_send(char *buffer, int len);

#endif /* DIGITAL_CAM_CATIA_SOCKET_H */
