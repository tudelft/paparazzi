#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsDroneSdk.h"

using namespace Sds::DroneSDK;


/* This is an example of of using the Field of View report
   Sending the FOV report is a debug feature to help users understand what beacons are in the 
   field of view for a given pose. It should not be used in production.
 */
int main(int ac, const char *av[]) {

    Version version = getVersion();
    std::cout << "Using SdsDroneSdk version: " << version.getString() << std::endl;
    
    // Initialize DroneTrackingManager
    std::unique_ptr<DroneTrackingManager> droneManager = createDroneTrackingManager();
    std::cout << "Drone Manager created" << std::endl;

    // Register callbacks before doing anything else so we can get error messages printed out
    droneManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
    });

    droneManager->registerPose6DofCallback([](const Pose6Dof& sd){
        std::cout << std::fixed << std::setprecision(3)
                  << "X: " << sd.x 
                  << " Y: " << sd.y 
                  << " Z: " << sd.z 
                  << " Qw: " << sd.qw 
                  << " Qx: " << sd.qx 
                  << " Qy: " << sd.qy 
                  << " Qz: " << sd.qz << std::endl;
    });

    droneManager->registerFieldOfViewReportCallback([](const FieldOfViewReport& report){
        std::cout << "New Report:" << std::endl;
        for (const auto& kv : report.seenBeacons) {
            std::cout << "Beacon: " << kv.first << " " << kv.second << std::endl;
        }
    });

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 0, "Ethernet" };
    bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Drone Manager failed network setup" << std::endl;
        return 1;
    }

    Config6Dof config6Dof;

    // input map
    Beacon b1 {0.000, 0.0, 0.000, 753};
    Beacon b2 {0.174, 0.0, 0.145, 752};
    Beacon b3 {0.162, 0.0, -0.118, 765};
    Beacon b4 {-0.163, 0.0, -0.120, 760};
    Beacon b5 {-0.179, 0.0, 0.145, 759};
    config6Dof.map.push_back(b1);
    config6Dof.map.push_back(b2);
    config6Dof.map.push_back(b3);
    config6Dof.map.push_back(b4);
    config6Dof.map.push_back(b5);

    bool sixdofSuccess = droneManager->initialize6Dof(config6Dof);
    if (!sixdofSuccess) {
        std::cout << "Drone Manager failed sixdof setup" << std::endl;
        return 1;
    }

    // Start tracking
    std::cout << "TrackingMode Sixdof" << std::endl;
    droneManager->setTrackingMode(TrackingMode::Sixdof);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}