#include "rpm_peak_tracker.h"

/** ABI Messaging */
#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID ABI_BROADCAST
#endif

#include "boards/bebop/actuators.h"
#include "modules/core/abi.h"

//#include "modules/datalink/telemetry.h" // Used to plot max_rpm_input_nn in real time

/*
uint16_t nn_rpm_obs_peak_tracker[4] = {0,0,0,0};
//uint16_t nn_rpm_ref[4] = {0,0,0,0};
static abi_event rpm_read_ev;
static void rpm_read_cb(uint8_t __attribute__((unused)) sender_id, uint16_t *rpm_obs_read, uint8_t __attribute__((unused)) num_act)
{
   nn_rpm_obs_peak_tracker[0] = rpm_obs_read[0];
   nn_rpm_obs_peak_tracker[1] = rpm_obs_read[1];
   nn_rpm_obs_peak_tracker[2] = rpm_obs_read[2];
   nn_rpm_obs_peak_tracker[3] = rpm_obs_read[3];
}
*/

/*
Struct: Create queue and attributes (Front of array, end, #of elements in the queue, size of array)

typedef struct _queue{
    int *values;
    int head, tail, num_entries, size, max_val;
} queue;
*/

/*
Function: Initialize array
*/
void init_queue(queue *q, int max_size){
    q->size = max_size;
    q->values = malloc(sizeof(int) * q->size);
    q->num_entries = 0; // array is empty
    q->head = 0;
    q->tail = 0; // Initial value doesn't matter here
    q->max_val = 1; // Keep track of max value
}

/*
Function: Returns true if array empty
*/
bool queue_empty(queue* q){
    return (q->num_entries == 0);
}

/*
Function: Returns true if array full
*/
bool queue_full(queue* q){
    return (q->num_entries == q->size);
}

/*
Function: Allows you to delete the array
*/
void queue_destroy(queue *q){
    free(q->values);
}

/*
Function: Allows to add value to array
*/
bool enqueue(queue *q, float value){
    // Check if max
    if (value > q->max_val){
        q->max_val = value;
    } 
    // Return false if full
    if (queue_full(q)) {
        printf("Array full\n");
        return false;
    }
    // If not full add:
    q->values[q->tail] = value; // Add data
    q->num_entries++; // Increase the number of entries by one
    q->tail = (q->tail + 1) % q->size; // Use modulus operator to start at beginning again
    return true;
}

/*
Function: Removing head of array - to make space for new data
*/
bool dequeue(queue *q){
    if (queue_empty(q)){
        return QUEUE_EMPTY;
    }
    q->head = (q->head + 1) % q->size;
    q->num_entries--; // Decrease number of entries by one
    return false;
}
/*
Function: Print all values in queue

float readqueue(queue *q){
    int i;
    float val;
    for (i = 0; i < q->size; ++i){
        if (q->head + i < q->size){
            val  = q->values[q->head + i];
            printf("array number %d: %f\n", i, val);
        } else {
            val  = q->values[q->head + i - q->size];
            printf("array number %d: %f\n", i, val);
        }
    }
    return val;
}
*/
/*
Function: call value at specific index of queue
*/
float call_array(queue *q, int i){
    // Call on specific value from index i
    float val;
    if (abs(i) > q->size) {
        printf("ERROR: Index out of bounds. (i=%d)\n",i);
	return 0;
    } else {
        if (i < 0){
            i = i + q->size;
        }
        if (q->head + i < q->size){
            val  = q->values[q->head + i];
        } else {
            val  = q->values[q->head + i - q->size];
        }
    return val;
    }
}
/*
Function: call max value in queue
THERE IS A PROBLEM WITH THIS function... how do you make sure that the max is not a value that has been dequeued?
*/
float call_max(queue *q){
    return (q->max_val);
}
/*
Function: filter RPMs below 11000 by only adding 0 if that's the case
Changelog: added data_add (value to be added) and data. In this case I only want to add if both the commanded AND observed RPMs are above 11000
*/
void filter_below_11000(queue *q, float data_add, float data){
    if (data_add < 11000.0 || data < 11000.0) {
        enqueue(q, 0.0);
    } else {
        enqueue(q, data_add);
    }
    return;
}
/*
Function: absolute value function
*/
float abs_seb(float val_check){
    if (val_check < 0){
    return (-1)*val_check; }
    else {
    return val_check; }
}


// NEED TO MANUALLY CHANGE ALL NUMBERS BECAUSE GLOABL VARIABLES HAVE TO BE INITIALIZED WITH 
queue u1_mod, u2_mod, u3_mod, u4_mod; // Commanded RPMs (4 arrays)
queue om1_mod, om2_mod, om3_mod, om4_mod; // Observed RPMs (4 arrays)
queue diff_1, diff_2, diff_3, diff_4; // Errors in integrals (4 arrays)

// Log frequency [Hz] and time step [s]
int frequency_paparazzi = 450; 

// Time window to check observed rpm peaks [s]
int peak_delta_t = 0.15;
int peak_delta = 67; // 450*0.15; // frequency_paparazzi*peak_delta_t;

// Time window to compute the integrals [s]
float int_delta_t = 0.1;
int int_delta = 45; // 450*0.1; // frequency_paparazzi*int_delta_t;

// Integral error threshold to trigger change in RPM
// Here compute it! Ideally 10% of max possible error (for int_delta = 45 max error is 46000)
float error_threshold = 4600;

// Increment by which to increase limit if no error is detected
float rpm_increase_increment = 1;

// Number of steps for integration
// Might have to make this lower if computational load is too high. If you want to make it lower divide it by an integer ideally, such that the indices remain integers and not rounded floats. Or run script separately to avoid slowing down the G&CNET main frequency
int n_steps = 45;  // 450*0.1; // int_delta; 
float dx = (450*0.1)/(450*0.1); // int_delta / n_steps;

float max_rpm_input_nn = 12000;

int transient_limit_change = 0;

// float max_error = 0;
// Debug code

float com_rpm1_float=0.0, com_rpm2_float=0.0, com_rpm3_float=0.0, com_rpm4_float=0.0;
float obs_rpm1_float=0.0, obs_rpm2_float=0.0, obs_rpm3_float=0.0, obs_rpm4_float=0.0;

float diff_int_1=0, diff_int_2=0, diff_int_3=0, diff_int_4=0;
float com_1_fil=-1, com_2_fil=-1, com_3_fil=-1, com_4_fil=-1;
float obs_1_fil=-1, obs_2_fil=-1, obs_3_fil=-1, obs_4_fil=-1;

/*
static void send_max_rpm_input_nn(struct transport_tx *trans, struct link_device *dev){
// Send telemetry message
pprz_msg_send_max_rpm_input_nn(trans , dev , AC_ID , &(max_rpm_input_nn), max_rpm_input_nn);
}
*/

void rpm_peak_tracker_init(void)
{

	// Initialize circular queues for RPM peak tracker

	// Commanded RPMs (4 arrays)
	init_queue(&u1_mod, peak_delta);
	init_queue(&u2_mod, peak_delta);
	init_queue(&u3_mod, peak_delta);
	init_queue(&u4_mod, peak_delta);

	// Observed RPMs (4 arrays)
	init_queue(&om1_mod, peak_delta);
	init_queue(&om2_mod, peak_delta);
	init_queue(&om3_mod, peak_delta);
	init_queue(&om4_mod, peak_delta);

/*
	// Errors in integrals (4 arrays)
	init_queue(&diff_1, int_delta);
	init_queue(&diff_2, int_delta);
	init_queue(&diff_3, int_delta);
	init_queue(&diff_4, int_delta);
*/

	// Register callbacks
	//register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_max_rpm_input_nn , send_max_rpm_input_nn);
}


void rpm_peak_tracker_run(void)
{
        // Remove first value when full
        if (queue_full(&u1_mod)){
            dequeue(&u1_mod);
            dequeue(&u2_mod);
            dequeue(&u3_mod);
            dequeue(&u4_mod);

            dequeue(&om1_mod);
            dequeue(&om2_mod);
            dequeue(&om3_mod);
            dequeue(&om4_mod);           
        }
/*
        if (queue_full(&diff_1)){
            dequeue(&diff_1);
            dequeue(&diff_2);
            dequeue(&diff_3);
            dequeue(&diff_4);  
	}
*/

        // Add RPM data if above 11000, else add 0

// Create to new variable because you need floats for the latter operations
/* new code
    com_rpm1_float = (float)actuators_bebop.rpm_ref[0];
    com_rpm2_float = (float)actuators_bebop.rpm_ref[1];
    com_rpm3_float = (float)actuators_bebop.rpm_ref[2];
    com_rpm4_float = (float)actuators_bebop.rpm_ref[3];

    obs_rpm1_float = (float)nn_rpm_obs_peak_tracker[0];
    obs_rpm2_float = (float)nn_rpm_obs_peak_tracker[1];
    obs_rpm3_float = (float)nn_rpm_obs_peak_tracker[2];
    obs_rpm4_float = (float)nn_rpm_obs_peak_tracker[3];

//

    filter_below_11000(&u1_mod, com_rpm1_float, obs_rpm1_float);
    filter_below_11000(&u2_mod, com_rpm2_float, obs_rpm2_float);
    filter_below_11000(&u3_mod, com_rpm3_float, obs_rpm3_float);
    filter_below_11000(&u4_mod, com_rpm4_float, obs_rpm4_float);

    // printf("Reference RPMs: %d %d %d %d\n", actuators_bebop.rpm_ref[0], actuators_bebop.rpm_ref[1], actuators_bebop.rpm_ref[2], actuators_bebop.rpm_ref[3]);

    filter_below_11000(&om1_mod, obs_rpm1_float, com_rpm1_float);
    filter_below_11000(&om2_mod, obs_rpm2_float, com_rpm2_float);
    filter_below_11000(&om3_mod, obs_rpm3_float, com_rpm3_float);
    filter_below_11000(&om4_mod, obs_rpm4_float, com_rpm4_float);
*/

// new code 2	       
    filter_below_11000(&u1_mod, (float)actuators_bebop.rpm_ref[0], (float)actuators_bebop.rpm_obs[0]);
    filter_below_11000(&u2_mod, (float)actuators_bebop.rpm_ref[1], (float)actuators_bebop.rpm_obs[1]);
    filter_below_11000(&u3_mod, (float)actuators_bebop.rpm_ref[2], (float)actuators_bebop.rpm_obs[2]);
    filter_below_11000(&u4_mod, (float)actuators_bebop.rpm_ref[3], (float)actuators_bebop.rpm_obs[3]);

    // printf("Reference RPMs: %d %d %d %d\n", actuators_bebop.rpm_ref[0], actuators_bebop.rpm_ref[1], actuators_bebop.rpm_ref[2], actuators_bebop.rpm_ref[3]);

    filter_below_11000(&om1_mod, (float)actuators_bebop.rpm_obs[0], (float)actuators_bebop.rpm_ref[0]);
    filter_below_11000(&om2_mod, (float)actuators_bebop.rpm_obs[1], (float)actuators_bebop.rpm_ref[1]);
    filter_below_11000(&om3_mod, (float)actuators_bebop.rpm_obs[2], (float)actuators_bebop.rpm_ref[2]);
    filter_below_11000(&om4_mod, (float)actuators_bebop.rpm_obs[3], (float)actuators_bebop.rpm_ref[3]);
//

/* old code	       
    filter_below_11000(&u1_mod, (float)actuators_bebop.rpm_ref[0], (float)nn_rpm_obs_peak_tracker[0]);
    filter_below_11000(&u2_mod, (float)actuators_bebop.rpm_ref[1], (float)nn_rpm_obs_peak_tracker[1]);
    filter_below_11000(&u3_mod, (float)actuators_bebop.rpm_ref[2], (float)nn_rpm_obs_peak_tracker[2]);
    filter_below_11000(&u4_mod, (float)actuators_bebop.rpm_ref[3], (float)nn_rpm_obs_peak_tracker[3]);

    // printf("Reference RPMs: %d %d %d %d\n", actuators_bebop.rpm_ref[0], actuators_bebop.rpm_ref[1], actuators_bebop.rpm_ref[2], actuators_bebop.rpm_ref[3]);

    filter_below_11000(&om1_mod, (float)nn_rpm_obs_peak_tracker[0], (float)actuators_bebop.rpm_ref[0]);
    filter_below_11000(&om2_mod, (float)nn_rpm_obs_peak_tracker[1], (float)actuators_bebop.rpm_ref[1]);
    filter_below_11000(&om3_mod, (float)nn_rpm_obs_peak_tracker[2], (float)actuators_bebop.rpm_ref[2]);
    filter_below_11000(&om4_mod, (float)nn_rpm_obs_peak_tracker[3], (float)actuators_bebop.rpm_ref[3]);
*/

// DEBUG CODE
	com_1_fil = call_array(&u1_mod, -1);
	com_2_fil = call_array(&u2_mod, -1);
	com_3_fil = call_array(&u3_mod, -1);
	com_4_fil = call_array(&u4_mod, -1);

	obs_1_fil = call_array(&om1_mod, -1);
	obs_2_fil = call_array(&om2_mod, -1);
	obs_3_fil = call_array(&om3_mod, -1);
	obs_4_fil = call_array(&om4_mod, -1);
// DEBUG CODE


	// printf("Observed RPMs: %d %d %d %d\n", nn_rpm_obs_peak_tracker[0], nn_rpm_obs_peak_tracker[1], nn_rpm_obs_peak_tracker[2], nn_rpm_obs_peak_tracker[3]);
	// printf("Observed RPMs in queue: %f\n", call_array(&om1_mod, -1));

        // Compute integral error (Technically you could start computing the integral earlier)
        if (queue_full(&u1_mod)) {

            // Trapezoidal rule
            float sum_u1=0, sum_u2=0, sum_u3=0, sum_u4=0;
            float sum_om1=0, sum_om2=0, sum_om3=0, sum_om4=0;
            // float diff_int_1, diff_int_2, diff_int_3, diff_int_4;
            int k;
            
            // Loop over integral window
            for(k = 1 ; k < int_delta-1 ; k++){
                sum_u1 = sum_u1 + call_array(&u1_mod, - int_delta + k*dx);
                sum_u2 = sum_u2 + call_array(&u2_mod, - int_delta + k*dx);
                sum_u3 = sum_u3 + call_array(&u3_mod, - int_delta + k*dx);
                sum_u4 = sum_u4 + call_array(&u4_mod, - int_delta + k*dx);

                sum_om1 = sum_om1 + call_array(&om1_mod, - int_delta + k*dx);
                sum_om2 = sum_om2 + call_array(&om2_mod, - int_delta + k*dx);
                sum_om3 = sum_om3 + call_array(&om3_mod, - int_delta + k*dx);
                sum_om4 = sum_om4 + call_array(&om4_mod, - int_delta + k*dx);
            }

            diff_int_1 = abs_seb( (dx/2)*(call_array(&u1_mod, -int_delta) + call_array(&u1_mod, -1) + 2*sum_u1) - ( (dx/2)*(call_array(&om1_mod, -int_delta) + call_array(&om1_mod, -1) + 2*sum_om1) ) );
            diff_int_2 = abs_seb( (dx/2)*(call_array(&u2_mod, -int_delta) + call_array(&u2_mod, -1) + 2*sum_u2) - ( (dx/2)*(call_array(&om2_mod, -int_delta) + call_array(&om2_mod, -1) + 2*sum_om2) ) );
            diff_int_3 = abs_seb( (dx/2)*(call_array(&u3_mod, -int_delta) + call_array(&u3_mod, -1) + 2*sum_u3) - ( (dx/2)*(call_array(&om3_mod, -int_delta) + call_array(&om3_mod, -1) + 2*sum_om3) ) );
            diff_int_4 = abs_seb( (dx/2)*(call_array(&u4_mod, -int_delta) + call_array(&u4_mod, -1) + 2*sum_u4) - ( (dx/2)*(call_array(&om4_mod, -int_delta) + call_array(&om4_mod, -1) + 2*sum_om4) ) );
	    

	    	// printf("First part diff: %f Second part diff: %f dx: %d\n",(call_array(&u1_mod, -int_delta) + call_array(&u1_mod, -1) + 2*sum_u1), (call_array(&om1_mod, -int_delta) + call_array(&om1_mod, -1) + 2*sum_om1), dx);
            // printf("diff_int_1 %f\n", diff_int_1);

/* Why do I keep track of past integral errors? You can just base your decision on the current one. This code would only make sense to speed up the process above. i.e. avoid having to
recompute the sum
            enqueue(&diff_1, diff_int_1);
            enqueue(&diff_2, diff_int_2);
            enqueue(&diff_3, diff_int_3);
            enqueue(&diff_4, diff_int_4);
            
            // Find highest error and corresponding rotor
            max_error = call_max(&diff_1);
            int max_error_index=0;

            if (call_max(&diff_1) > max_error){
                max_error = call_max(&diff_1);
                max_error_index = 0;
            }
            if (call_max(&diff_2) > max_error){
                max_error = call_max(&diff_2);
                max_error_index = 1;
            }            
            if (call_max(&diff_3) > max_error){
                max_error = call_max(&diff_3);
                max_error_index = 2;
            }            
            if (call_max(&diff_4) > max_error){
                max_error = call_max(&diff_4);
                max_error_index = 3;
            }

*/
	    // printf("Last integral error computed: %f %f %f %f\n", call_array(&diff_1, -1), call_array(&diff_2, -1), call_array(&diff_3, -1), call_array(&diff_4, -1));

	    // printf("Max integral errors computed: %f %f %f %f\n", call_max(&diff_1), call_max(&diff_2), call_max(&diff_3), call_max(&diff_4));

	    // printf("Check max_rpm: %f\n",max_rpm_input_nn);

	    // printf("CHeck max plus increment: %f\n",(max_rpm_input_nn+rpm_increase_increment));

            // Check if error peaks above threshold
/* 
           if (max_error > error_threshold){
                if (max_error_index == 0){
                    max_rpm_input_nn = call_max(&om1_mod);
                }
                if (max_error_index == 1){
                    max_rpm_input_nn = call_max(&om2_mod);
                }                
                if (max_error_index == 2){
                    max_rpm_input_nn = call_max(&om3_mod);
                }                
                if (max_error_index == 3){
                    max_rpm_input_nn = call_max(&om4_mod);
                } 
            } 
*/
	    // Check if any of the 4 integral errors is above the threshold
            if (diff_int_1 > error_threshold || diff_int_2 > error_threshold || diff_int_3 > error_threshold || diff_int_4 > error_threshold){
                float max_rpm_input_nn_temp = 11000;

		if (diff_int_1 > error_threshold && call_array(&om1_mod, -1) > max_rpm_input_nn_temp){
                    max_rpm_input_nn_temp = call_array(&om1_mod, -1);
                }
		if (diff_int_2 > error_threshold && call_array(&om2_mod, -1) > max_rpm_input_nn_temp){
                    max_rpm_input_nn_temp = call_array(&om2_mod, -1);
                }
		if (diff_int_3 > error_threshold && call_array(&om3_mod, -1) > max_rpm_input_nn_temp){
                    max_rpm_input_nn_temp = call_array(&om3_mod, -1);
                }
		if (diff_int_4 > error_threshold && call_array(&om4_mod, -1) > max_rpm_input_nn_temp){
                    max_rpm_input_nn_temp = call_array(&om4_mod, -1);
                }

	    // Only allow a change after every int_delta or peak_delta timesteps (otherwise you might change the limit too conservatively because the integral error is still high)
		if (transient_limit_change > peak_delta){
			transient_limit_change = 0;
			max_rpm_input_nn = max_rpm_input_nn_temp;
		} else {
	    // Don't increase if above lower (acceptable threshold
			transient_limit_change++;
		}
            } 
		else {
                // It should be possible to challenge the max rpm
                // Transient behavior for rpm limit increase
		
                max_rpm_input_nn = max_rpm_input_nn+rpm_increase_increment;
            }



	    // Safety conditions to not confuse G&CNET in case of error above
            if (max_rpm_input_nn > 12000){
                max_rpm_input_nn = 12000;
            }
            if (max_rpm_input_nn < 11000){
                max_rpm_input_nn = 11000;
            }
        }
	    // printf("Current max rpm: %d\n\n", max_rpm_input_nn);

}
