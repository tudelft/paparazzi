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

using namespace Sds::DroneSDK;

ConfigRelativeBeacon config_rel_beacon; // use defaults
ConfigRelativeAngle config_rel_angle; // use defaults

int verbose = 1; 

static void on_ChangeSensorMode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  
  if (argc != 20)
  {
    fprintf(stderr,"ERROR: invalid message length SHIP_INFO_MSG_GROUND\n");
  }
  else{
    //Change mode! 
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
    std::unique_ptr<DroneTrackingManager> droneManager = createDroneTrackingManager();
    std::cout << "Drone Manager created" << std::endl;

    // Register callbacks before doing anything else so we can get error messages printed out
    droneManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
    });

    droneManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        for (const RelativeAngle& ra : col) { 
            if(verbose){
                std::cout << "ID: " << ra.id
                        << std::fixed << std::setprecision(0)
                        << " XAng: " << std::setw(3) << ra.x_angle * 180.0 / 3.14159
                        << " ZAng: " << std::setw(3) << ra.z_angle * 180.0 / 3.14159 
                        << " Intensity: " << std::setw(4) << ra.intensity 
                        << std::fixed << std::setprecision(2)
                        << " Width: " << std::setw(5) << ra.width << std::endl;
            }
            //Send values over IVYBUS
            IvySendMsg("RELATIVE_BEACON_ANGLE %d %f %f %f %f", ra.id, ra.x_angle, ra.z_angle, ra.intensity, ra.width);
        }
    });

    droneManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        for (const PoseRelativeBeacon& b : col) {
            if(verbose){
            std::cout << "ID: " << b.id
                    << std::fixed << std::setprecision(3)
                    << " X: " << b.x 
                    << " Y: " << b.y 
                    << " Z: " << b.z << std::endl;
            }
            //Send values over IVYBUS
              IvySendMsg("RELATIVE_BEACON_POS %d %f %f %f ", b.id, b.x, b.y, b.z);
        }    
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

    IvyBindMsg(on_ChangeSensorMode, NULL, "CHANGE_SIXDOF_SYS_MODE (\\S*)");

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