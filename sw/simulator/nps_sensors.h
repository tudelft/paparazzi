#ifndef NPS_SENSORS_H
#define NPS_SENSORS_H

#include "pprz_algebra.h"
#include "nps_sensor_gyro.h"
//nclude "nps_sensor_accel.h"
//nclude "nps_sensor_mag.h"
//nclude "nps_sensor_baro.h"
//#include "nps_sensor_gps.h"


struct NpsSensors {
  struct DoubleRMat body_to_imu_rmat;
  struct NpsSensorGyro  gyro;
  //  struct NpsSensorAccel accel;
  //  struct NpsSensorMag   mag;
  //  struct NpsSensorBaro  baro;
  //  struct NpsSensorGps   gps;

};

extern struct NpsSensors sensors;

extern void nps_sensors_init(double time);
extern void nps_sensors_run_step(double time);

extern bool_t nps_sensors_gyro_available();

#endif /* NPS_SENSORS_H */
