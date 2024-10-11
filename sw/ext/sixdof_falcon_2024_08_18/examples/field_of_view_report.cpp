#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                      Field of View Report Example                       *
// *                                                                         *
// *   This example demonstrates how to use the Field of View Report.        *
// *   The FOV Report is a callback that shows which beacons are seen by the * 
// *   sensor. Sending the FOV Report is a debug feature to help users       *
// *   understand what beacons are in the field of view for a given pose.    *
// *   The FOV Report should not be used in production.                      *
// *   This example runs the 6DOF tracking mode, however, the FOV Report can *
// *   be used with any tracking mode.                                       *
// *   It is important to note the order of function calls:                  *
// *   - Create a Falcon Manager                                             *
// *   - Register Callbacks, this is where the FOV Report callback is set    *
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

    falconManager->registerPose6DofCallback([](const Pose6Dof& sd){
        std::cout << std::fixed << std::setprecision(3)
                  << "X: " << sd.x 
                  << " Y: " << sd.y 
                  << " Z: " << sd.z 
                  << " Qw: " << sd.qw 
                  << " Qx: " << sd.qx 
                  << " Qy: " << sd.qy 
                  << " Qz: " << sd.qz << std::endl;
    });

    // Register the Field of View Report callback
    falconManager->registerFieldOfViewReportCallback([](const FieldOfViewReport& report){
        std::cout << "New Report:" << std::endl;
        for (const auto& kv : report.seenBeacons) {
            std::cout << "Beacon: " << kv.first << " " << kv.second << std::endl;
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
    Config6Dof config6Dof;

    // input map
    Beacon b1 {0.0, 0.0, 0.0, 1};
    Beacon b2 {0.0, 0.0, 0.0, 2};
    Beacon b3 {0.0, 0.0, 0.0, 3};
    Beacon b4 {0.0, 0.0, 0.0, 4};
    Beacon b5 {0.0, 0.0, 0.0, 5};
    config6Dof.map.push_back(b1);
    config6Dof.map.push_back(b2);
    config6Dof.map.push_back(b3);
    config6Dof.map.push_back(b4);
    config6Dof.map.push_back(b5);

    // Initialize 6DOF Tracking Mode
    bool sixdofSuccess = falconManager->initialize6Dof(config6Dof);
    if (!sixdofSuccess) {
        std::cout << "Falcon Manager failed sixdof setup" << std::endl;
        return 1;
    }

    // Start 6DOF Tracking
    std::cout << "TrackingMode Sixdof" << std::endl;
    falconManager->setTrackingMode(TrackingMode::Sixdof);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;
}