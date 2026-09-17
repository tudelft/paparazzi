#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../socket.h"

int main(void)
{
  assert(socket_init(1) == 0);

  int diagnostic_pipe[2];
  assert(pipe(diagnostic_pipe) == 0);
  pid_t child = fork();
  assert(child >= 0);
  if (child == 0) {
    close(diagnostic_pipe[0]);
    close(socket_get_fd());
    assert(dup2(diagnostic_pipe[1], STDERR_FILENO) >= 0);
    close(diagnostic_pipe[1]);
    assert(socket_init(1) == -1);
    assert(errno == EADDRINUSE);
    _exit(EXIT_SUCCESS);
  }

  close(diagnostic_pipe[1]);
  char diagnostic[1024];
  ssize_t length = read(diagnostic_pipe[0], diagnostic, sizeof(diagnostic) - 1);
  assert(length > 0);
  diagnostic[length] = '\0';
  close(diagnostic_pipe[0]);

  int status;
  assert(waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
  assert(strstr(diagnostic, "another CATIA instance or payload listener") != NULL);
  assert(strstr(diagnostic, "systemctl stop catia.service") != NULL);

  close(socket_get_fd());
  assert(socket_init(1) == 0);
  close(socket_get_fd());
  puts("CATIA socket ownership: duplicate rejected early and active owner preserved");
  return EXIT_SUCCESS;
}