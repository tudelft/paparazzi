#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                      Heartbeat Feature Example #1                       *
// *                                                                         *
// *   This example demonstrates how to use the Heartbeat feature.           *
// *   The Heartbeat is a callback which shows that the communication        *
// *   between the computer and the sensor is working, and that the runnning *
// *   state of the Falcon Manager is valid.                                 * 
// *   This example runs the 6DOF tracking mode, however, the Heartbeat can  *
// *   be used with any tracking mode.                                       *
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {
    
    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Register callback before calling the init function so we can get the heartbeat
    falconManager->registerHeartbeatCallback([](const Heartbeat& h){
            std::cout << " comm " << (h.sensor_communication_ok ? "ON" : "OFF")
                      << " mode " << to_string(h.current_tracking_mode)
                      << " state " << to_string(h.current_tracking_mode_state) << std::endl;
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
    falconManager->setTrackingMode(TrackingMode::Sixdof);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Done" << std::endl;

    return 0;
}