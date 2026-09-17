/**
 * @file socket.c
 * @brief Loopback UDP implementation for CATIA's local development transport.
 * @details The fixed port permits the local bridge and test tools to find each other
 * without configuration. All I/O is non-blocking so local consumers cannot delay the
 * flight-controller UART event loop.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include "socket.h"


#define SOCKET_PORT 32000

static int socket_fd;
static struct sockaddr_in socket_server;

int socket_init(int is_server)
{
  // Initialize socket
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("Socket: socket");
    return -1;
  }

  bzero(&socket_server, sizeof(socket_server));
  socket_server.sin_family = AF_INET;
  socket_server.sin_port = htons(SOCKET_PORT);
  //inet_aton("192.168.1.69", &socket_server.sin_addr);
  inet_aton("127.0.0.1", &socket_server.sin_addr);

  if (is_server) {
    if (bind(socket_fd, (struct sockaddr *)&socket_server, sizeof(socket_server)) != 0) {
      int bind_errno = errno;
      if (bind_errno == EADDRINUSE) {
        fprintf(stderr,
                "CATIA:\tanother CATIA instance or payload listener already owns "
                "127.0.0.1:%d\n"
                "CATIA:\tstop the existing process before starting another instance\n"
                "CATIA:\tfor the managed MORA service, use: "
                "sudo -n systemctl stop catia.service\n",
                SOCKET_PORT);
      } else {
        fprintf(stderr, "CATIA:\tfailed to bind local payload port 127.0.0.1:%d: %s\n",
                SOCKET_PORT, strerror(bind_errno));
      }
      close(socket_fd);
      socket_fd = -1;
      errno = bind_errno;
      return -1;
    }
  }
  return 0;
}

int socket_get_fd(void)
{
  return socket_fd;
}

int socket_recv(char *buffer, int len)
{
  socklen_t slen = sizeof(socket_server);
  return recvfrom(socket_fd, buffer, len, MSG_DONTWAIT | MSG_TRUNC,
                  (struct sockaddr *)&socket_server, &slen);
}

void socket_send(char *buffer, int len)
{
  socklen_t slen = sizeof(socket_server);
  sendto(socket_fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)&socket_server, slen);
}
