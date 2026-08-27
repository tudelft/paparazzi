
#ifndef DIGITAL_CAM_CATIA_SOCKET_H
#define DIGITAL_CAM_CATIA_SOCKET_H

void socket_init(int is_server);
int socket_recv(char *buffer, int len);
void socket_send(char *buffer, int len);

#endif /* DIGITAL_CAM_CATIA_SOCKET_H */
