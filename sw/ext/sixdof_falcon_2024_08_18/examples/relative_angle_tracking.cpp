#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                     Relative Angle Tracking Example                     *
// *                                                                         *
// *   This example demonstrates the typical workflow for using the          *
// *   SdsFalcon SDK for Relative Angle Tracking.                           *
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

    falconManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
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
    NetworkConfig networkConfiguration { 0, "Ethernet" };
    bool networkSuccess = falconManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Falcon Manager failed network setup" << std::endl;
        return 1;
    }

    // Enter the configuration data for your specific setup
    ConfigRelativeAngle config; // use defaults

    // Initialize Relative Angle Tracking Mode
    bool success = falconManager->initializeRelativeAngle(config);
    if (!success) {
        std::cout << "Falcon Manager failed relative angle setup" << std::endl;
        return 1;
    }

    // Start Relative Angle Tracking
    falconManager->setTrackingMode(TrackingMode::RelativeAngle);

    std::this_thread::sleep_for(std::chrono::seconds(60));

    std::cout << "Done" << std::endl;
    return 0;
}