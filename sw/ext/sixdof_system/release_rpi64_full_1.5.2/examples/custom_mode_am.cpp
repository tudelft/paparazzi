#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsDroneSdk.h"

using namespace Sds::DroneSDK;


/* This is an example of Relative Beacon tracking */
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

    droneManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        for (const RelativeAngle& ra : col) { 
            std::cout << "ID: " << ra.id
                    << std::fixed << std::setprecision(0)
                    << " XAng: " << std::setw(3) << ra.x_angle * 180.0 / 3.14159
                    << " ZAng: " << std::setw(3) << ra.z_angle * 180.0 / 3.14159 
                    << " Intensity: " << std::setw(4) << ra.intensity 
                    << std::fixed << std::setprecision(2)
                    << " Width: " << std::setw(5) << ra.width << std::endl;
        }
    });

    droneManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        for (const PoseRelativeBeacon& b : col) {
            std::cout << "ID: " << b.id
                    << std::fixed << std::setprecision(3)
                    << " X: " << b.x 
                    << " Y: " << b.y 
                    << " Z: " << b.z << std::endl;
        }
    });

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 92, "enx3c8cf8fbb199" };
    bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Drone Manager failed network setup" << std::endl;
        return 1;
    }

    ConfigRelativeBeacon config_rel_beacon; // use defaults
    ConfigRelativeAngle config_rel_angle; // use defaults

    bool sixdofSuccess = droneManager->initializeRelativeBeacon(config_rel_beacon);
    if (!sixdofSuccess) {
        std::cout << "Drone Manager failed relative beacon setup" << std::endl;
        return 1;
    }

    // Start tracking
    std::cout << "TrackingMode RelativeBeacon" << std::endl;
    droneManager->setTrackingMode(TrackingMode::RelativeBeacon);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Switching to tracking mode = RelativeAngle" << std::endl;
    sixdofSuccess = droneManager->initializeRelativeAngle(config_rel_angle);
    if (!sixdofSuccess) {
        std::cout << "Drone Manager failed relative angle setup" << std::endl;
        return 1;
    }
    droneManager->setTrackingMode(TrackingMode::RelativeAngle);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Done" << std::endl;
    return 0;
}