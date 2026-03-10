/*
 * MAV Course 2026 - Luke Optical Flow Module
 *
 * Header file: declares the module's public interface.
 * Other modules can include this to read the threshold variable or
 * call the init/periodic functions (though Paparazzi calls them automatically
 * based on the XML configuration).
 */

#ifndef LUKE_OPTICAL_FLOW_H
#define LUKE_OPTICAL_FLOW_H

// The divergence threshold used to decide "obstacle detected".
// Declared extern so the GCS dl_setting can write to it at runtime.
extern float luke_of_divergence_threshold;

// Called once at autopilot startup (registered in the module XML as <init>).
extern void luke_optical_flow_init(void);

// Called at 4 Hz by the autopilot loop (registered in the XML as <periodic>).
extern void luke_optical_flow_periodic(void);

#endif /* LUKE_OPTICAL_FLOW_H */
