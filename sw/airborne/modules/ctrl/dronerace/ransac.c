
#include "ransac.h"
#include "filter.h"

#include "math/RANSAC.h"

#define  RANSAC_BUF_SIZE   20

// Time, x_predict, y, x_measured, y
struct dronerace_ransac_buf_struct
{
  // Settings
  float time;

  // Predicted States
  float x;
  float y;

  // Measured States
  float mx;
  float my;
};

struct dronerace_ransac_buf_struct ransac_buf[RANSAC_BUF_SIZE];

struct dronerace_ransac_struct dr_ransac;


void ransac_reset(void)
{
  int i;
  for (i=0; i<RANSAC_BUF_SIZE; i++) {
    ransac_buf[i].time = 0;
    ransac_buf[i].x = 0;
    ransac_buf[i].y = 0;
    ransac_buf[i].mx = 0;
    ransac_buf[i].my = 0;
  }
  dr_ransac.dt_max = 1.0;
}

// From newest (0) to oldest (RANSAC_BUF_SIZE)
static int get_index(int element)
{
  int ind = dr_ransac.buf_index_of_last - element;
  if (ind < 0) { ind += RANSAC_BUF_SIZE; }
  return ind;
}

void ransac_push(float time, float x, float y, float mx, float my)
{
  int i = 0;

  // Insert in the buffer
  dr_ransac.buf_index_of_last++;
  if (dr_ransac.buf_index_of_last >= RANSAC_BUF_SIZE)
  {
    dr_ransac.buf_index_of_last = 0;
  }
  ransac_buf[dr_ransac.buf_index_of_last].time = time;
  ransac_buf[dr_ransac.buf_index_of_last].x = x;
  ransac_buf[dr_ransac.buf_index_of_last].y = y;
  ransac_buf[dr_ransac.buf_index_of_last].mx = mx;
  ransac_buf[dr_ransac.buf_index_of_last].my = my;

  // Update buffer size
  dr_ransac.buf_size = 0;
  for (i=0; i<RANSAC_BUF_SIZE; i++ )
  {
    float mt = ransac_buf[get_index(i)].time;
    if ((mt == 0) || ((dr_state.time - mt) > dr_ransac.dt_max))
    {
      break;
    }
    dr_ransac.buf_size++;
  }

  // log
  //char name[128];
  //FILE* fp = fopen()
  
}

#include <stdio.h>

static void ransac_get_vector(void)
{
  int i = 0;
  for ( i=0; i<dr_ransac.buf_size; i++ )
  {
    // Get index
    ransac_buf[get_index(i)];
  }
}

