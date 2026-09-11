#include <assert.h>
#include <signal.h>
#include "../chdk_pipe.h"

int main(void)
{
  signal(SIGPIPE, SIG_IGN);
  assert(chdk_pipe_init() == -1);
  chdk_pipe_deinit();
  assert(chdk_pipe_init() == -1);
  chdk_pipe_deinit();
  return 0;
}