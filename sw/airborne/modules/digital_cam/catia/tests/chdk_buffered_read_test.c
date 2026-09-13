/*
 * The buffered read_character() reader must reproduce exactly the same
 * multi-character CHDK protocol parsing (hash-counted filename echo, then the
 * '>' prompt) as the previous byte-at-a-time poll()+read() version, just with
 * far fewer syscalls per response.
 */
#include "../chdk_pipe.c"
#include <assert.h>
#include <string.h>
#include <unistd.h>

int main(void)
{
  int descriptors[2];
  assert(pipe(descriptors) == 0);
  fo = descriptors[0];
  const char *response = "AA##BB#CC#>";
  assert(write(descriptors[1], response, strlen(response)) == (ssize_t)strlen(response));

  char filename[MAX_FILENAME];
  assert(wait_for_img(filename, 5) == 0);
  assert(strcmp(filename, "BBCC") == 0);

  close(descriptors[1]);
  close(descriptors[0]);
  fo = -1;
  chdk_pipe_deinit();
  puts("CHDK buffered reader: multi-character protocol parsing across one chunked read passed");
  return 0;
}
