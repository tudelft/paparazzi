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

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 92, "eth0" };
    bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Drone Manager failed network setup" << std::endl;
        return 1;
    }

    ConfigRelativeAngle config; // use defaults

    bool success = droneManager->initializeRelativeAngle(config);
    if (!success) {
        std::cout << "Drone Manager failed relative angle setup" << std::endl;
        return 1;
    }

    // Start tracking
    droneManager->setTrackingMode(TrackingMode::RelativeAngle);

    std::this_thread::sleep_for(std::chrono::seconds(60));

    std::cout << "Done" << std::endl;
    return 0;
}