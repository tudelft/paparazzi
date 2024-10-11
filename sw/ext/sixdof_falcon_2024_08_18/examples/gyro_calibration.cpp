#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsFalcon.h"

using namespace Sds::Falcon;


// ***************************************************************************
// *                                                                         *
// *                    Gyro Calibration Feature Example                     *
// *                                                                         *
// *   This example demonstrates how to use the Gyro Calibration feature.    *
// *   The Gyro Calibration feature allows for customizing the configuration *
// *   settings for Gyro Calibration, including skipping it during           * 
// *   initialization by loading it from a previously done configuration.    *
// *   Gyro Calibration is only relevant for the 6DOF tracking mode.         *
// *   In the example, there are 2 anonymous scopes:                         *
// *   1. The gyro calibration is performed and saved.                       *
// *   2. 6DOF tracking mode is initialized using the saved gyro calibration * 
// *      and then is run.                                                   *
// *                                                                         *
// ***************************************************************************


int main(int ac, const char *av[]) {

    // Print the SdsFalcon SDK version
    Version version = getVersion();
    std::cout << "Using SdsFalcon version: " << version.getString() << std::endl;

    // Variable for storing the Gyro Calibration
    // In a real situation you would save the gyro calibration data to a file, database etc.
    std::vector<GyroOffset> gyroCalibration;

    // -------------------- Get the Gyro Calibration --------------------
    // Performed in anonymous scope so the Falcon Manager will go out of scope
    {
        // Initialize the Falcon Manager
        std::unique_ptr<FalconManager> falconManager = createFalconManager();

        // Register callbacks before calling the init function so we can get error messages printed out
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

        // Getting the Gyro Calibration
        ConfigGyroCalibration config;
        config.duration_seconds = 10;

        std::cout << "Doing Gyro Calibration, please keep the sensor still" << std::endl;
        gyroCalibration = falconManager->getGyroCalibration(config);
        std::cout << "Finished Gyro Calibration, now you can move the sensor" << std::endl;

        // Print out the gyro calibration just to see what it is
        for (const GyroOffset& c : gyroCalibration) {
            std::cout << std::endl;
            std::cout << "datetime: " << c.datetime << std::endl 
                        << "temperature: " << (int)c.temperature << std::endl
                        << "gyro_bias: " << c.gyro_x << " " << c.gyro_y << " " << c.gyro_z << " " << std::endl
                        << "std: " <<c.standard_dev << std::endl;
        }

        // This is where you would save the gyro calibration data to a file, database etc.
    }

    // -------------------- Use the Gyro Calibration --------------------
    // Now we can use the gyro calibration, in real situation you would read from file, database etc.
    {
        // Initialize the Falcon Manager
        std::unique_ptr<FalconManager> falconManager = createFalconManager();

        // Register callbacks before calling the init function so we can get error messages printed out
        falconManager->registerMessageCallback([](const StatusMessage& msg){
            std::cout << msg.message << std::endl;
        });

        falconManager->registerPose6DofCallback([](const Pose6Dof& sd){
            std::cout << "X: " << sd.x << " Y: " << sd.y << " Z: " << sd.z << std::endl;
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
        config6Dof.gyroCalibration = gyroCalibration; // here is where we pass in the gyro calibration 

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
        // Because we are passing in the gyro calibration, that part of initialization sequence will be skipped
        // Therefore, you can even shake the sensor during initialization
        std::cout << "Using Gyro Calibration, you can even shake the sensor" << std::endl;
        bool sixdofSuccess = falconManager->initialize6Dof(config6Dof);
        if (!sixdofSuccess) {
            std::cout << "Falcon Manager failed sixdof setup" << std::endl;
            return 1;
        }

        // Start 6DOF Tracking
        std::cout << "TrackingMode Sixdof" << std::endl;
        falconManager->setTrackingMode(TrackingMode::Sixdof);

        std::this_thread::sleep_for(std::chrono::seconds(10));

        // Stop Tracking
        falconManager->setTrackingMode(TrackingMode::TrackingOFF);
    }

    return 0;
}