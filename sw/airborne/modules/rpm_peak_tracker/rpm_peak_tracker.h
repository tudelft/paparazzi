/*
File: rpm_peak_tracker.h
Goal: Include header of different functions used to compute peak in rpm
*/

#ifndef RPM_peak_tracker_operations
#define RPM_peak_tracker_operations


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>


#define QUEUE_EMPTY

typedef struct _queue{
    int *values;
    int head, tail, num_entries, size, max_val;
} queue;


extern void init_queue(queue *q, int max_size);

extern bool queue_empty(queue* q);

extern bool queue_full(queue* q);

extern void queue_destroy(queue *q);

extern bool enqueue(queue *q, float value);

extern bool dequeue(queue *q);

extern float readqueue(queue *q);

extern float call_array(queue *q, int i);

extern float call_max(queue *q);

extern void filter_below_11000(queue *q, float data_add, float data);

extern float abs_seb(float val_check);

/*
Main functions in conf file:
  <init fun="rpm_peak_tracker_init()"/>
  <periodic fun="rpm_peak_tracker_run()" freq="512" />
*/
extern void rpm_peak_tracker_init(void);

extern void rpm_peak_tracker_run(void);

/*
Main output of this module:
*/
extern float max_rpm_input_nn;

// extern float max_error;
// Debug loggs
extern float diff_int_1, diff_int_2, diff_int_3, diff_int_4;
extern float rpm_1_exp, rpm_2_exp, rpm_3_exp, rpm_4_exp;

extern float com_1_fil, com_2_fil, com_3_fil, com_4_fil;
extern float obs_1_fil, obs_2_fil, obs_3_fil, obs_4_fil;

//static void send_max_rpm_input_nn(struct transport_tx *trans, struct link_device *dev);

#endif
