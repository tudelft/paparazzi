#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                    Relative Beacon Tracking Example                     *
// *                                                                         *
// *   This example demonstrates the typical workflow for using the          *
// *   SdsFalcon SDK for Relative Beacon Tracking.                           *
// *   It is important to note the order of function calls:                  *
// *   - Create a Falcon Manager                                             *
// *   - Register Callbacks                                                  *
// *   - Create the Network Configuration                                    *
// *   - Initialize the Network (establishes a connection to the sensor)     *
// *   - Create the Tracking Mode Configuration                              *
// *   - Initialize the Tracking Mode                                        *
// *   - Start the Tracking Mode                                             *
// *   - View the outputs from the registered callbacks                      *
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {

    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Register callbacks before calling the init function so we can get error messages printed out
    falconManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
    });

    falconManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        for (const PoseRelativeBeacon& b : col) {
            std::cout << "ID: " << b.id
                    << std::fixed << std::setprecision(3)
                    << " X: " << b.x 
                    << " Y: " << b.y 
                    << " Z: " << b.z << std::endl;
        }
    });

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 0, "Ethernet" };
    bool networkSuccess = falconManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Falcon Manager failed network setup" << std::endl;
        return 1;
    }

    // Enter the configuration data for your specific setup
    ConfigRelativeBeacon config; // use defaults

    // Initialize Relative Beacon Tracking Mode
    bool sixdofSuccess = falconManager->initializeRelativeBeacon(config);
    if (!sixdofSuccess) {
        std::cout << "Falcon Manager failed relative beacon setup" << std::endl;
        return 1;
    }

    // Start Relative Beacon Tracking
    std::cout << "TrackingMode RelativeBeacon" << std::endl;
    falconManager->setTrackingMode(TrackingMode::RelativeBeacon);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Done" << std::endl;
    return 0;
}