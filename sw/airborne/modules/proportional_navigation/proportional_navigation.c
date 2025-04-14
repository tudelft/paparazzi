#include <stdio.h>
#include "state.h"  // Assumes this header declares stateGetPositionNed_f()

/**
 * @brief Periodic function to print the current drone position.
 *
 * This function is intended to be called by the autopilot periodically.
 */
void simple_position_print_periodic(void) {
    // Retrieve the current position from the state.
    const struct FloatVect3 *current_pos = stateGetPositionNed_f();
    
    // Print the current position.
    // Format: X, Y, Z coordinates (e.g., in meters).
    printf("Drone Position (NED): X = %.2f, Y = %.2f, Z = %.2f\n",
           current_pos->x, current_pos->y, current_pos->z);
}
