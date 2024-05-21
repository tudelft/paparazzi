#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsDroneSdk.h"
#include <string>
#include <sstream>
#include <glib.h>
#include <ivy.h>
#include <ivyglibloop.h>
#include <sys/time.h>
#include <math.h>

using namespace Sds::DroneSDK;

ConfigRelativeBeacon config_rel_beacon; // use defaults
ConfigRelativeAngle config_rel_angle; // use defaults
Config6Dof config6Dof; // use defaults

float euler_angles[3], UAV_ned_pos[3]; 

std::unique_ptr<DroneTrackingManager> droneManager;

int verbose = 1; 

int current_sixdof_mode = 1; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 

struct timeval current_time;

// input map
Beacon b1 {0.288, -0.222, -0.005, 1640};
Beacon b2 {0.265, 0, -0.03, 1636};
Beacon b3 {0.292, 0.224, -0.005, 1645};
Beacon b4 {-0.298, 0.23, -0.005, 1632};
Beacon b5 {-0.30, -0.222, -0.005, 1633};

/**
 * Transpose an array from body reference frame to earth reference frame
 * using the provided rotation matrix.
 */
static void from_body_to_earth(float * out_array, float * in_array, float Phi, float Theta, float Psi){
    float R_be_matrix[3][3];
    
    // Compute the elements of the matrix
    R_be_matrix[0][0] = cos(Theta) * cos(Psi);
    R_be_matrix[0][1] = -cos(Phi) * sin(Psi) + sin(Phi) * sin(Theta) * cos(Psi);
    R_be_matrix[0][2] = sin(Phi) * sin(Psi) + cos(Phi) * sin(Theta) * cos(Psi);
    R_be_matrix[1][0] = cos(Theta) * sin(Psi);
    R_be_matrix[1][1] = cos(Phi) * cos(Psi) + sin(Phi) * sin(Theta) * sin(Psi);
    R_be_matrix[1][2] = -sin(Phi) * cos(Psi) + cos(Phi) * sin(Theta) * sin(Psi);
    R_be_matrix[2][0] = -sin(Theta);
    R_be_matrix[2][1] = sin(Phi) * cos(Theta);
    R_be_matrix[2][2] = cos(Phi) * cos(Theta);

    // Do the multiplication between the input array and the rotation matrix:
    for (int j = 0; j < 3; j++) {
        out_array[j] = 0.; // Initialize output array element to zero
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_be_matrix[j][k];
        }
    }
}

static void ivy_get_attitude_and_ned_pos(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  if (argc != 12)
  {
    fprintf(stderr,"ERROR: invalid message length ROTORCRAFT_FP\n");
  }
  else{
    UAV_ned_pos[1] = (float) atof(argv[0])*0.0039063; 
    UAV_ned_pos[0] = (float) atof(argv[1])*0.0039063; 
    UAV_ned_pos[2] = (float) -atof(argv[2])*0.0039063; 

    euler_angles[0] = (float) atof(argv[6])*0.0139882; 
    euler_angles[1] = (float) atof(argv[7])*0.0139882;
    euler_angles[2] = (float) atof(argv[8])*0.0139882;

    if(verbose){
      fprintf(stderr,"Received ROTORCRAFT_FP packet - Phi_deg = %.2f, Theta_deg = %.2f Psi_deg = %.2f; UAV_x_pos_NED = %.2f; UAV_y_pos_NED = %.2f; UAV_z_pos_NED = %.2f; \n",euler_angles[0]*180/M_PI,euler_angles[1]*180/M_PI,euler_angles[2]*180/M_PI,UAV_ned_pos[0],UAV_ned_pos[1],UAV_ned_pos[2]);
    }
  }
}

static void ivy_set_rel_beacon_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
    if(current_sixdof_mode != 1){
        bool sixdofSuccess = droneManager->initializeRelativeBeacon(config_rel_beacon);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed relative beacon setup" << std::endl;
        }
        // Start tracking
        std::cout << "TrackingMode RelativeBeacon" << std::endl;
        droneManager->setTrackingMode(TrackingMode::RelativeBeacon);
        current_sixdof_mode = 1;
    }
}

static void ivy_set_rel_angle_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
    if(current_sixdof_mode != 2){
        std::cout << "Switching to tracking mode = RelativeAngle" << std::endl;
        bool sixdofSuccess = droneManager->initializeRelativeAngle(config_rel_angle);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed relative angle setup" << std::endl;
        }
        droneManager->setTrackingMode(TrackingMode::RelativeAngle);
        current_sixdof_mode = 2;
    }
}

static void ivy_set_sixdof_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{

    if(current_sixdof_mode != 3){
        config6Dof.map.push_back(b1);
        config6Dof.map.push_back(b2);
        config6Dof.map.push_back(b3);
        config6Dof.map.push_back(b4);
        config6Dof.map.push_back(b5);

        bool sixdofSuccess = droneManager->initialize6Dof(config6Dof);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed sixdof setup" << std::endl;
        }

        // Start tracking
        std::cout << "TrackingMode Sixdof" << std::endl;
        droneManager->setTrackingMode(TrackingMode::Sixdof);
        current_sixdof_mode = 3;
    }
}

/* This is an example of Relative Beacon tracking */
int main(int ac, const char *av[]) {

    std::cout << "Init ivy bus" << std::endl;
    //Initialize the Ivy bus address based on the operating system

    const char* ivy_bus;
    #ifdef __APPLE__
        ivy_bus = "224.255.255.255";
    #else
        ivy_bus = "127.255.255.255";
    #endif

    // Initialize the GLib main loop
    GMainLoop *ml = g_main_loop_new(NULL, FALSE);

    Version version = getVersion();
    std::cout << "Using SdsDroneSdk version: " << version.getString() << std::endl;

    // Initialize DroneTrackingManager
    droneManager = createDroneTrackingManager();
    std::cout << "Drone Manager created" << std::endl;

    // Register callbacks before doing anything else so we can get error messages printed out
    droneManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
    });

    droneManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        for (const RelativeAngle& ra : col) { 
            gettimeofday(&current_time, NULL);
            double current_timestamp = (double) (current_time.tv_sec + current_time.tv_usec*1e-6);
            if(verbose){
                std::cout << "ID: " << ra.id
                        << std::fixed << std::setprecision(5)
                        << "Timestamp: " << current_timestamp << std::setprecision(0)
                        << " XAng: " << std::setw(3) << ra.x_angle * 180.0 / 3.14159
                        << " ZAng: " << std::setw(3) << ra.z_angle * 180.0 / 3.14159 
                        << " Intensity: " << std::setw(4) << ra.intensity 
                        << std::fixed << std::setprecision(2)
                        << " Width: " << std::setw(5) << ra.width << std::endl;
            }
            //Send values over IVYBUS
            IvySendMsg("RELATIVE_BEACON_ANGLE %f %d %f %f %f %f", current_timestamp, ra.id, ra.x_angle, ra.z_angle, ra.intensity, ra.width);
        }
    });

    droneManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        for (const PoseRelativeBeacon& b : col) {
            gettimeofday(&current_time, NULL);
            double current_timestamp = (double) (current_time.tv_sec + current_time.tv_usec*1e-6);
            if(verbose){
            std::cout << "ID: " << b.id
                    << std::fixed << std::setprecision(3)
                    << "Timestamp: " << current_timestamp
                    << " X: " << b.x 
                    << " Y: " << b.y 
                    << " Z: " << b.z << std::endl;
            }
            //Correct this readings with the AP attitude angle and generate a reading with respect to the UAV earth RF. 
            float relative_beacon_pos_body_rf = {b.z, b.x, b.y}; 
            float relative_beacon_pos_earth_rf[3];
            
            from_body_to_earth(relative_beacon_pos_earth_rf, relative_beacon_pos_body_rf, euler_angles[0], euler_angles[1], euler_angles[2]); 
            //Sum UAV NED pos: 
            float absolute_beacon_pos_earth_rf = {relative_beacon_pos_earth_rf[0] + UAV_ned_pos[0],
                                                  relative_beacon_pos_earth_rf[1] + UAV_ned_pos[1],
                                                  relative_beacon_pos_earth_rf[2] + UAV_ned_pos[2]};
            //Sum the coordinate of the beacon associated to the reading: 
            if(b.id == b1.id)
    
            IvySendMsg("ABSOLUTE_NED_RF_POS %f %d %f %f %f ", current_timestamp, b.id, b.x, b.y, b.z);

            //Send values over IVYBUS
            IvySendMsg("RELATIVE_BEACON_POS %f %d %f %f %f ", current_timestamp, b.id, b.x, b.y, b.z);
        }    
    });

    droneManager->registerPose6DofCallback([](const Pose6Dof& sd){
        gettimeofday(&current_time, NULL);
        double current_timestamp = (double) (current_time.tv_sec + current_time.tv_usec*1e-6);
        if(verbose){
            std::cout << std::fixed << std::setprecision(5)
                    << "Timestamp: " << current_timestamp << std::setprecision(3)
                    << "X: " << sd.x 
                    << " Y: " << sd.y 
                    << " Z: " << sd.z 
                    << " Qw: " << sd.qw 
                    << " Qx: " << sd.qx 
                    << " Qy: " << sd.qy 
                    << " Qz: " << sd.qz << std::endl;
        }
        //Send values over IVYBUS
        IvySendMsg("SIXDOF_TRACKING %f %f %f %f %f %f %f %f", current_timestamp, sd.x, sd.y, sd.z, sd.qw, sd.qx, sd.qy, sd.qz);
    });

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 92, "enx3c8cf8fbb199" };
    bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Drone Manager failed network setup" << std::endl;
        return 1;
    }

    //Default state
    bool sixdofSuccess = droneManager->initializeRelativeBeacon(config_rel_beacon);
    if (!sixdofSuccess) {
        std::cout << "Drone Manager failed relative beacon setup" << std::endl;
        return 1;
    }
    // Start tracking
    std::cout << "TrackingMode RelativeBeacon" << std::endl;
    droneManager->setTrackingMode(TrackingMode::RelativeBeacon);


    IvyInit ("SixDofSysSensor", "SixDofSysSensor READY", NULL, NULL, NULL, NULL);

    IvyBindMsg(ivy_set_rel_beacon_mode, NULL, "SET_SIXDOF_SYS_MODE %d",1);
    IvyBindMsg(ivy_set_rel_angle_mode, NULL, "SET_SIXDOF_SYS_MODE %d",2);
    IvyBindMsg(ivy_set_sixdof_mode, NULL, "SET_SIXDOF_SYS_MODE %d",3);
    IvyBindMsg(ivy_get_attitude_and_ned_pos, NULL, "1 ROTORCRAFT_FP  (\\S*) (\\S*) (\\S*)  (\\S*) (\\S*) (\\S*)  (\\S*) (\\S*) (\\S*)  (\\S*) (\\S*) (\\S*)");
    IvyStart(ivy_bus);

    g_main_loop_run(ml);

    // std::this_thread::sleep_for(std::chrono::seconds(10));

    // std::cout << "Switching to tracking mode = RelativeAngle" << std::endl;
    // sixdofSuccess = droneManager->initializeRelativeAngle(config_rel_angle);
    // if (!sixdofSuccess) {
    //     std::cout << "Drone Manager failed relative angle setup" << std::endl;
    //     return 1;
    // }
    // droneManager->setTrackingMode(TrackingMode::RelativeAngle);

    // std::this_thread::sleep_for(std::chrono::seconds(10));


    std::cout << "Done" << std::endl;
    return 0;
}