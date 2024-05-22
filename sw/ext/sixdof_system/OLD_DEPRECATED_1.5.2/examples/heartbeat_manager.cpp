#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include "SdsDroneSdk.h"

using namespace Sds::DroneSDK;


/* This is an example of one way of using the heartbeat feature.
   In this example we dedicate a thread to checking the heartbeat. 
   Then if the heartbeat is not received we can do an action or throw an error. */

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

    HeartbeatManager heartbeatManager;

    Version version = getVersion();
    std::cout << "Using SdsDroneSdk version: " << version.getString() << std::endl;

    // Initialize DroneTrackingManager
    std::unique_ptr<DroneTrackingManager> droneManager = createDroneTrackingManager();
    std::cout << "Drone Manager created" << std::endl;

    // Register callback before doing anything else so we can get the heartbeat
    droneManager->registerHeartbeatCallback([&](const Heartbeat& h){
            heartbeatManager.update(h);
        });

    droneManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl;
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
    droneManager->setTrackingMode(TrackingMode::Sixdof);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Done" << std::endl;

    return 0;
}