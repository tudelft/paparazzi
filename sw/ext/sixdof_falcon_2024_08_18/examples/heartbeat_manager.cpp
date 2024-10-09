#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                      Heartbeat Feature Example #2                       *
// *                                                                         *
// *   This example demonstrates another way to use the Heartbeat feature.   *
// *   In this example, a thread is dedicated to checking the Heartbeat.     *
// *   Then if the heartbeat is not received we can do an action or throw    *
// *   an error.                                                             *
// *                                                                         *
// ***************************************************************************

// This manager creates a new thread which checks the heartbeat periodically
// and can do an action in the case that the heartbeat stops.
class HeartbeatManager {
public:

    HeartbeatManager() {
        thread = std::thread([&](){
            periodic_check();
        });
    }

    ~HeartbeatManager() {
        keep_running = false;
        if (thread.joinable()) {
            thread.join();
        }
    }

    void update(const Heartbeat& heartbeat) {
        std::lock_guard<std::mutex> lock(mutex);
        got_first_heartbeat = true;
        time_last_heartbeat = std::chrono::steady_clock::now();

        if (!heartbeat.sensor_communication_ok) {
            std::cout << "Sensor communication not connected" << std::endl;
        }

        if (heartbeat.current_tracking_mode_state == TrackingModeState::Error) {
            std::cout << "Positioning is an Error state" << std::endl;
        }
    }

private:
    std::atomic<bool> keep_running { true };
    bool got_first_heartbeat { false };
    std::mutex mutex;
    std::thread thread;
    std::chrono::steady_clock::time_point time_last_heartbeat;

    void periodic_check() {
        while (keep_running) {
            { // anonymous scope so that the lock is released before the sleep
                std::lock_guard<std::mutex> lock(mutex);
                if (got_first_heartbeat) {   
                    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                    double delay_seconds = (now - time_last_heartbeat).count() * 1.0e-9;
                    if (delay_seconds > 1.5) {
                        std::cout << "SDK Heartbeat has stopped for " << delay_seconds << " seconds" << std::endl;
                        // here you can do an action like restart the SDK 
                        // or throw an error.
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
    }
};


int main(int ac, const char *av[]) {

    // Initialize the HeartbeatManager which handles checking the heartbeat in its own thread
    HeartbeatManager heartbeatManager;

    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Initialize the Falcon Manager
    std::unique_ptr<FalconManager> falconManager = createFalconManager();
    std::cout << "Falcon Manager created" << std::endl;

    // Register callback before calling the init function so we can get the heartbeat
    falconManager->registerHeartbeatCallback([&](const Heartbeat& h){
            heartbeatManager.update(h);
        });

    falconManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
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