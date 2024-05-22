#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsDroneSdk.h"

using namespace Sds::DroneSDK;


/* This is an example of how to use the gyro calibration feature */
int main(int ac, const char *av[]) {

    Version version = getVersion();
    std::cout << "Using SdsDroneSdk version: " << version.getString() << std::endl;

    std::vector<GyroOffset> gyroCalibration;

    // get the gyro calibration
    // anonymous scope so drone tracking manager will go out of scope
    {
        std::unique_ptr<DroneTrackingManager> droneManager = createDroneTrackingManager();

        droneManager->registerMessageCallback([](const StatusMessage& msg){
            std::cout << msg.message << std::endl;
        });

        NetworkConfig networkConfiguration { 22, "Ethernet" };
        bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
        if (!networkSuccess) {
            std::cout << "Drone Manager failed network setup" << std::endl;
            return 1;
        }

        ConfigGyroCalibration config;
        config.duration_seconds = 10;

        std::cout << "Doing Gyro Calibration, please keep the sensor still" << std::endl;
        gyroCalibration = droneManager->getGyroCalibration(config);
        std::cout << "Finished Gyro Calibration, now you can move the sensor" << std::endl;

        // print out the gyro calibration just to see what it is
        for (const GyroOffset& c : gyroCalibration) {
            std::cout << std::endl;
            std::cout << "datetime: " << c.datetime << std::endl 
                        << "temperature: " << (int)c.temperature << std::endl
                        << "gyro_bias: " << c.gyro_x << " " << c.gyro_y << " " << c.gyro_z << " " << std::endl
                        << "std: " <<c.standard_dev << std::endl;
        }

        // in a real situation you would save the gyro calibration data to a file, database ect.
    }

    // now we can use the gyro calibration, in real situation you would read from file, database ect.
    {
        // Initialize DroneTrackingManager
        std::unique_ptr<DroneTrackingManager> droneManager = createDroneTrackingManager();

        // Register callbacks before doing anything else so we can get error messages printed out
        droneManager->registerMessageCallback([](const StatusMessage& msg){
            std::cout << msg.message << std::endl;
        });

        droneManager->registerPose6DofCallback([](const Pose6Dof& sd){
            std::cout << "X: " << sd.x << " Y: " << sd.y << " Z: " << sd.z << std::endl;
        });

        NetworkConfig networkConfiguration { 22, "Ethernet" };
        bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
        if (!networkSuccess) {
            std::cout << "Drone Manager failed network setup" << std::endl;
            return 1;
        }

        // Initialize the tracking mode in this case 6Dof
        Config6Dof config6Dof;
        config6Dof.gyroCalibration = gyroCalibration; // here is where we pass in the gyro calibration 

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

        // because we are passing in the gyro calibration, that part of initialization sequence will be skipped
        // you can even shake the sensor during initialization
        std::cout << "Using Gyro Calibration, you can even shake the sensor" << std::endl;
        bool sixdofSuccess = droneManager->initialize6Dof(config6Dof);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed sixdof setup" << std::endl;
            return 1;
        }

        // Start tracking
        std::cout << "TrackingMode Sixdof" << std::endl;
        droneManager->setTrackingMode(TrackingMode::Sixdof);

        std::this_thread::sleep_for(std::chrono::seconds(10));

        droneManager->setTrackingMode(TrackingMode::TrackingOFF);
    }

    return 0;
}