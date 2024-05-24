//Copyright © 2017-2024 Six Degrees Space Ltd.  All rights reserved.
//Proprietary and Confidential.  Unauthorized use, disclosure or reproduction is strictly prohibited.

/**
 * @file
 * @brief Header file containing declarations for the SDS DroneSDK library.
 *
 * This file contains the declarations for classes and data structures related to the SDS DroneSDK library.
 * It provides functionality for managing drone tracking, including retrieving pose information, managing the Sixdof sensor, 
 * and registering callbacks for various events.
 */

#pragma once 
#include <stdint.h>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <functional>

#ifdef _WIN32
    // Windows platform
    #ifdef SDS_EXPORTS
        #define SDS_API __declspec(dllexport)
    #else
        #define SDS_API __declspec(dllimport)
    #endif
#else
    // Non-Windows platform
    #define SDS_API __attribute__((visibility("default")))
#endif

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define DEPRECATED(msg)
#endif

/**
 * @mainpage SdsDroneSdk Documentation
 * 
 * @image html SixdofSpace_Logo_2023.png "" width=400px
 * @image latex SixdofSpace_Logo_2023.png "" width=300px
 *
 * @section intro_sec Introduction
 * Sixdof Space has created an optical tracking solution offering precision landing with cm level accuracy both indoors and outdoors in direct sunlight. 
 * Our patented technology leverages infrared lighting to serve as location beacons. Our sensor unit, connected to your drone can see the beacons on the ground at up to 40 meters.
 * Our algorithms will report the 6dof positional data to the drone, to direct an autonomous landing. 
 * 
 * SdsDroneSdk is a shared library that enables users to interface with the Sixdof tracking technology.
 * Drone guidance is the main application of this library, however other applications that require a programmatic interface to the Sixdof technology can use this library as well. 
 *
 * The SdsDroneSdk shared library is available on the following platforms:
 * - Windows 
 * - Linux Ubuntu
 * - Linux Raspberry Pi OS (32 bit)
 * - Linux Raspberry Pi OS (64 bit)
 * 
 * If you want to use the SdsDroneSdk on another platform, please contact the Sixdof team.
 * 
 * @section coordinate_sys_sec Coordinate Systems
 * Before using the Sixdof tracking system it is important to understand the coordinate systems involved. The sensor coordinate system is defined according to the image below:
 * @image html sensor_coordinates.jpg "Sensor coordinate system" width=400px
 * @image latex sensor_coordinates.jpg "Sensor coordinate system" width=150px
 * 
 * Where the positive x-axis points to the right of the sensor, the positive y-axis point out from the sensor and the positive z-axis completes the right handed system. 
 * The field of view can be approximated by a 60 degrees vision cone around the y-axis.
 * 
 * In many applications the user is interested in the location of the drone, and not just the location of the sensor.
 * In these applications, it is the users responsibility to do the rigid body transformation from the sensor to the drone, based on how the sensor is mounted on the drone. 
 * 
 * @section tracking_modes_sec Tracking Modes
 * Three tracking modes are available: 
 * - Sixdof: Track the full 6 degrees of freedom relative to the map. This is intended for applications where the drone is within 6m of the map.
 * - Relative Beacon: Track the position of a beacon relative to the sensor. This is intended to guide a drone to a landing pad with a high powered beacon from 40m down to the final 2m.
 * - Relative Angle: Track the angle of a beacon relative to the sensor. The intent of this mode is similar to Relative Beacon, however it is designed for tracking from very far ranges down to the final descent. 
 * 
 * The SdsDroneSdk provides an easy way to run these two tracking modes, and seamlessly transition between them.
 * 
 * In addition to tracking, a method is provided to enable the user to do gyroscope calibration. 
 * During gyroscope calibration, the sensor must be still. In many applications, it is desirable to initialize the system while already in motion.
 * Therefore, gyroscope calibration can be done in advance to the mission and the calibrated values can be fed in at run time. 
 *
 * @section sixdof_sec Tracking mode Sixdof
 * @image html 6DOF_Light.png "" width=400px
 * @image latex 6DOF_Light.png "" width=250px
 * In this mode, the postion of the sensor is tracked relative to the map. The map is made up of multiple IR beacons that flash unique ids, multiple beacons are driven by a single Basalt board.
 * The user must place these beacons in a rigid formation and provide the X, Y, Z location of each beacon. The unique id of each beacon is provided on each LED individually.
 * The user can define the coordinate system of the map in any way they choose, however it must use a right handed coordinate system.
 * When designing a map, it is important to ensure the lights are not co-linear. Keep in mind the desired area of operation, try to design the map so that there will always be at least 3 beacons in the field of view of the sensor.
 * 
 * @section rel_bec_sec Tracking mode Relative Beacon
 * @image html RelB_Light.png "" width=400px
 * @image latex RelB_Light.png "" width=250px
 * In this mode, the position of a high-powered beacon is tracked relative to the coordinate system of the sensor. 
 * The goal of this mode is to be able to guide a drone from a high elevation (40m) down to a landing pad. This mode does not provide any orientation information.
 * It is important to note that the distance estimate (y-direction) of the beacon is not accurate when far from the beacon. 
 * However, this mode does provide the ability to derive the direction of the beacon in order to align for the descent. 
 * When the beacon is closer (~2m) to the sensor, the distance estimate will be more accurate.
 * The reason for this inaccuracy at large distances is due to the very small baseline on the sensor.
 *  
 * @section rel_angle_sec Tracking mode Relative Angle
 * @image html RelA_Light.png "" width=400px
 * @image latex RelA_Light.png "" width=250px
 * In this mode the angle of a beacon is tracked relative to the coordinate system of the sensor.
 * The goal of this mode is to be able to guide a drone from a high elevation (> 40m) all the way down to a landing pad. This mode does not provide any orientation information of the senor.
 * Additionally, due to the far distance of the beacon the baseline on the sensor is insufficent to calculate the distance directly.
 * In this mode the intensity of the beacon and the peak width in pixels are reported in order to get a rough estimate of the distance of the beacon. 
 * In a configuration where full Sixdof landing is desired, the rough distance estimate is can be used as a signal that the sensor is close enough to transition to Sixdof mode.  
 * 
 * @section tracking_mode_switch Switching between Tracking Modes
 * Switching between tracking modes can be done almost instantly via the setTrackingMode function. 
 * In many drone applications there is a need for the drone to be guided to the landing pad from a high elevation, as well as do a precision landing.
 * In these cases a single high-powered LED can be used along side a map of multiple small LEDs.
 * When the drone is at a high elevation it is guided to the landing target by using "Relative Angles" mode, then when it gets sufficiently close (~6m) it switches to "Sixdof" mode for the final high-precision landing.
 * The combination of "Relative Angles" and "Sixdof" modes is a highly effective way of achieving a targeted landing from a high elevation, however the key component to this solution is the ability to decided when to switch between the tracking modes.
 * In the "Relative Angles" tracking modes the distance to the LED is unknown, however the intensity of the LED and the width in pixel space are provided.
 * From this information a rough estimate of the distance to the landing target can be calculated.
 * Shown below are the results of an experiment where the intensity of the LED was measured at known distances, y-axis is distance in meters and x-axis is the intensity,
 * a polynomial trend line is shown in red:
 * @image html intensity_to_dis.png "" width=400px
 * @image latex intensity_to_dis.png "" width=450px
 * 
 * A rough estimate of the distance between the sensor and the landing target can be obtained from the polynomial fit shown above.
 * Additionally, the width in pixels of the LED can help to enhance the distance estimate when the sensor is close to the light.
 * Note that this relationship is only valid for the beacon that was used in the experiment, in this case a 100 watt beacon, and the intensity to distance relationship can change slightly with lighting conditions.
 * Many drones are equipped with other sensors such as range finders and barometers, these sensors can assist with estimating the distance to the landing target.
 * 
 * 
 * @section change_log_sec Change Log
 *    Version    | Date       | Notes |
 * ------------- | ---------- | ----- |
 * 1.5.3         | 29/02/2024 | Added NetworkAdvancedConfig, getFirmwareVersion, skip_gyro_calibration is now defaulted to true |
 * 1.5.2         | 20/02/2024 | Added expected_beacon_ids to ConfigRelativeBeacon and ConfigRelativeAngle, also changed shutter backed, deprecated setShutter function |
 * 1.5.1         | 01/02/2024 | Added skip_gyro_calibration flag to Config6Dof |
 * 1.5.0         | 20/12/2023 | Added setShutter feature, current shutter callback, put shutter_settings into each of the config objects |
 * 1.4.1         | 27/11/2023 | Added swapMap feature |
 * 1.4.0         | 05/11/2023 | Added Relative Angle feature |
 * 1.3.0         | 17/10/2023 | Added health monitoring feature in the form of Heartbeat callbacks |
 * 1.2.1         | 06/09/2023 | Exposed "No Gyro" mode as a parameter in Config6Dof. Made changes to support multiple sensors each with its own DroneTrackingManager |
 * 1.2.0         | 22/08/2023 | Changed API so that PoseRelativeBeacons that happen at the same time are bundled together |
 * 1.1.2         | 14/08/2023 | Updated RelativeBeacon logic, parameter rel_lights_cooldown_ms is no longer needed, instead we have the parameter field_of_view_cutoff_deg |
 * 1.1.1         | 14/08/2023 | Statically link GCC libraries, add MSVC build to the release |
 * 1.1.0-beta    | 24/07/2023 | Changed header, included gyro calibration, field of view report features and data logging for replay |
 * 1.0.0-beta    | 01/05/2023 | inital beta version |
 * 
 * @section troubleshooting Trouble Shooting Guide
 * @subsection missing_libraries Missing Libraries
 * Make sure to always copy the two shared libraries (.dll on Windows and .so on Linux) that are provided in the release folder for your platform into the the directory with your program. 
 * Alternatively, you can statically link the shared libraries to your program. Also on Windows you have the option of adding the shared libraries to your system PATH. 
 * 
 * @subsection ping_failed Ping Failed
 * Errors relating to ping failing mean that the host computer was unable to connect to the Sixdof sensor.
 * Try the following steps:
 *  -# Ensure the Sixdof sensor is connected to power and the RJ45 ethernet cable is connected to your computer
 *  -# Look at the red indicator lights that are exposed on the top of the senor casing, the lights should be flashing, if the lights are constant please contact the Sixdof team
 *  -# Ensure your NetworkConfig parameters are correct, this is the sensor board number and the network name that the sensor is connected to
 *  -# Your computers ARP table is used to connect to the sensor, try clearing the ARP table and trying again. The ARP table can be cleared vai a command console with administrator privileges, use the command "arp -d" 
 * 
 * @subsection firewall Firewall
 * If SdsDroneSdk is reporting errors related to communication issues, it is recommended to turn off your firewall.
 * 
 * @subsection underprocessing Underprocessing
 * If you are getting the "Warning" status message that says "Not receiving enough buffers from board" that means that your program is not getting enough CPU time. 
 * On Windows systems this is usually due to a spontaneous virus scan, you can temporarily turn off virus scans from in your "Windows Security" settings.
 */

namespace Sds {
    namespace DroneSDK {
    
        /**
         * @class Version
         * @brief A class representing the version of either the shared library, or the sensor firmware.
         *
         * This class provides utility functions around a system version (major, minor, patch), either the version of the shared library, or the sensor firmware version.
         */
        class SDS_API Version {
        public:
            Version(uint8_t major, uint8_t minor, uint8_t patch);

            /**
             * @brief Get the major component of the version.
             * @return The major component of the version as a uint8_t.
             */
            uint8_t getMajor() const;

            /**
             * @brief Get the minor component of the version.
             * @return The minor component of the version as a uint8_t.
             */
            uint8_t getMinor() const;

            /**
             * @brief Get the patch component of the version.
             * @return The patch component of the version as a uint8_t.
             */
            uint8_t getPatch() const;

            /**
             * @brief Check if the version is equal to the specified components.
             *
             * @param major The major component to compare.
             * @param minor The minor component to compare.
             * @param patch The patch component to compare.
             * @return True if the version is equal to the specified components, false otherwise.
             */
            bool isEqual(uint8_t major, uint8_t minor, uint8_t patch) const;

            /**
             * @brief Check if the version is at least the specified version.
             *
             * @param major The major component to compare.
             * @param minor The minor component to compare.
             * @param patch The patch component to compare.
             * @return True if the version is at least the specified version, false otherwise.
             */
            bool isAtLeast(uint8_t major, uint8_t minor, uint8_t patch) const;

            /**
             * @brief Get a string representation of the version.
             * 
             * Get a string representation of the version.
             * @return A string representing the version.
             */
            std::string getString() const;

        private:
            uint8_t _major, _minor, _patch;
        };

        /**
         * @brief Get the version of the SDS DroneSDK shared library.
         * @return The Version object representing the library version.
         */
        SDS_API Version getVersion();

        /**
         * @struct NetworkConfig
         * @brief A structure representing network configuration for the Sixdof sensor.
         *
         * This structure contains the necessary information in order to connect to a Sixdof sensor.
         */
        struct SDS_API NetworkConfig {
            uint16_t sensor_id;               ///< sensor id.
            std::string sensor_network_name;  ///< name of the network the sensor is connected to.
            uint16_t udp_read_port { 0 };     ///< udp port to read from the sensor, to be auto-assigned a port leave as 0. (Optional)
        };

        /**
         * @struct NetworkAdvancedConfig
         * @brief A structure representing advanced network configuration for the Sixdof sensor.
         *
         * This structure contains the necessary information in order to connect to a Sixdof sensor.
         * This is different to the NetworkConfig object that only needs the sensor id and the network name.
         * NetworkAdvancedConfig is for cases where the user will add the entry to the ARP table themselves.
         * Typically this is used on system where the basic network configuration will fail, for example in situations where the 
         * computer operating system is not in English.
         */
        struct SDS_API NetworkAdvancedConfig {
            uint16_t sensor_id;               ///< sensor id.
            std::string sensor_ip;            ///< desired sensor ip address.
            uint16_t udp_read_port { 0 };     ///< udp port to read from the sensor, to be auto-assigned a port leave as 0. (Optional)
        };

        /**
         * @struct Beacon
         * @brief A structure representing a beacon with 3D coordinates and an id.
         *
         * This structure contains the 3D coordinates (x, y, z) of a beacon in meters and its unique id.
         * Both the small beacons used for 6dof tracking, and the large beacon used for Relative Light tracking are represented with the Beacon struct.
         * Each beacon provided is labeled with its unique id.
         */
        struct SDS_API Beacon {
            double x;     ///< x-coordinate in meters.
            double y;     ///< y-coordinate in meters.
            double z;     ///< z-coordinate in meters.
            uint16_t id;  ///< unique id.
        };

        /**
         * @struct Pose6Dof
         * @brief A structure representing a 6dof pose.
         *
         * This structure contains 6dof (6 degree of freedom) pose information, specifically the pose of the Sixdof sensor with respect to the map. 
         * The pose is represented by position (x, y, z) in meters, and orientation as a quaternion (qx, qw, qy, qz).
         * Pose accuracy is represented as variance of the position (var_x, var_y, var_z) in meters squared, and the variance of the orientation (var_h, var_p, var_r) in radians squared.
         * 
         * This is the return type when in Sixdof tracking mode.
         */
        struct SDS_API Pose6Dof {
            bool valid { false };   ///< flag indicating if the pose is valid. This can be False when calling the getPose6Dof function before a valid pose is obtained.
            double x;               ///< x-coordinate in meters.
            double y;               ///< y-coordinate in meters.
            double z;               ///< z-coordinate in meters.
            double qw;              ///< quaternion w component.
            double qx;              ///< quaternion x component.
            double qy;              ///< quaternion y component.
            double qz;              ///< quaternion x component.
            float var_x, var_y, var_z; // m^2
            float var_h, var_p, var_r; // rad^2
        };

        /**
         * @struct PoseRelativeBeacon
         * @brief A structure representing the relative pose of a beacon.
         *
         * This structure contains the 3D pose of a single beacon with respect to the sensor.
         * Pose is represented as 3D position (x, y, z) in meters.
         */
        struct SDS_API PoseRelativeBeacon {
            double x;             ///< x-coordinate in meters.
            double y;             ///< y-coordinate in meters.
            double z;             ///< z-coordinate in meters.
            uint16_t id;          ///< unique id of the beacon.
        };

        /**
         * @typedef PoseRelativeBeaconCollection
         * @brief A structure representing a collection of PoseRelativeBeacon.
         *
         * This structure contains multiple PoseRelativeBeacon estimates. 
         * All PoseRelativeBeacon estimates were calculated at the same and therefore bundled together.
         * 
         * This is the return type when in RelativeBeacon tracking mode.
         */
        typedef std::vector<PoseRelativeBeacon> PoseRelativeBeaconCollection;


        /**
         * @struct RelativeAngle
         * @brief A structure representing the relative angle of a beacon.
         *
         * This structure contains the angle of a single beacon with respect to the sensor.
         * Where x_angle is the angular offset along the x-axis, and z_angle is the angular offset along the z-axis.
         */
        struct SDS_API RelativeAngle {
            double x_angle;           ///< angular offset in the x-axis in radians.
            double z_angle;           ///< angular offset in the z-axis in radians.
            uint16_t id;              ///< unique id of the beacon.
            double intensity;         ///< light intensity of the beacon, this can be used to roughly indicate distance.
            double width;             ///< width of the detected peak in pixels, also useful for indicating distance.
        };

        /**
         * @typedef RelativeAngleCollection
         * @brief A structure representing a collection of RelativeAngle.
         *
         * This structure contains multiple RelativeAngle estimates. 
         * All RelativeAngle estimates were calculated at the same and therefore bundled together.
         * 
         * This is the return type when in RelativeAngle tracking mode.
         */
        typedef std::vector<RelativeAngle> RelativeAngleCollection;

        /**
         * @enum Severity
         * @brief An enum representing the severity level of a status message.
         *
         * This enumeration defines severity levels for status messages.
         */
        enum class Severity {
            Exception = 1, ///< Exception indicates that there was a error in operation.
            Warning,       ///< Warning indicates that something went wrong but it is not critical.
            Informative,   ///< Informative indicates general debugging information, these can be ignored.
        };

        /**
         * @enum CallingLayer
         * @brief An enum representing the calling layer for a status message.
         *
         * This enum defines the calling layers for status messages.
         * Each layer indicates the source of a particular message.
         */
        enum class CallingLayer {
            SdsDroneSdk = 1,         ///< Message from the top layer of the library. Generally indicates the library was used incorrectly.
            Algorithm6Dof,           ///< Message from Sixdof tracking mode.
            AlgorithmRelativeBeacon, ///< Message from RelativeBeacon tracking mode.
            SdsCommLib,              ///< Message from the UDP communication library, this is used to communicate with the sensor.
            NetworkConnection,       ///< Message from the network connection stage.
            GyroCalibration,         ///< Message from the gyro calibration feature.
            AlgorithmRelativeAngle,  ///< Message from RelativeAngle tracking mode.
        };

        /**
         * @enum DroneSdkEventCode
         * @brief An enum representing event codes specific to the SdsDroneSdk calling layer.
         *
         * This enum defines event codes that are specific to the SdsDroneSdk calling layer.
         */
        enum class DroneSdkEventCode {
            UnexpectedAlgoInstance,     ///< indicates internal error, contact Sixdof for tech support.
            TrackingModeNotInitalized,  ///< indicates tracking mode was set before the corresponding initialization.
            NetworkNotInitalized,       ///< indicates that the initializeNetwork function was not called.
            GyroCalibrationRejected,    ///< indicates gyro calibration values were rejected.
            CommunicationLoopNotClosed, ///< communication loop is not closed, check your firewall
        };

        /**
         * @struct StatusMessage
         * @brief A structure representing a status message.
         *
         * This structure contains information about a status message, including its severity,
         * calling layer, event code, and the content of the message.
         */
        struct SDS_API StatusMessage {
            Severity severity;   ///< severity level of the status message.
            CallingLayer layer;  ///< calling layer for the status message.
            uint8_t event_code;  ///< event code of the status message.
            std::string message; ///< verbose status message.
        };

        /**
         * @enum TrackingMode
         * @brief An enum representing different tracking modes.
         *
         * This enum is used to set the tracking mode.
         */
        enum class TrackingMode {
            TrackingOFF,    ///< tracking is turned off. This is the default and when the DroneTrackingManager goes out of scope it is set to TrackingOFF automatically.
            RelativeBeacon, ///< tracking is set to RelativeBeacon mode.
            Sixdof,         ///< tracking is set to Sixdof (6dof) mode.
            RelativeAngle,  ///< tracking is set to RelativeAngle mode.
        };

        /**
         * @brief Convert a TrackingMode enum value to a string.
         *
         * @param mode TrackingMode enum value.
         * @return string representation of the TrackingMode.
         */
        SDS_API std::string to_string(TrackingMode mode);

        /**
         * @enum TrackingModeState
         * @brief An enum representing different states of the current tracking mode.
         */
        enum class TrackingModeState {
            Initializing, ///< tracking mode is currently initializing.
            Running,      ///< tracking mode is currently running.
            Error,        ///< tracking mode is currently in the error state.
        };

        /**
         * @brief Convert a TrackingModeState enum value to a string.
         *
         * @param state TrackingModeState enum value.
         * @return string representation of the TrackingModeState.
         */
        SDS_API std::string to_string(TrackingModeState state);

        /**
         * @struct Heartbeat
         * @brief A structure representing health data of the SDK.
         *
         * This structure is output via a callback every second. This enables the user to quickly identify that the SDK is still running.
         * Additionally, the Heartbeat structure contains information regarding the state of communications with the sensor and the tracking algorithm.
         */
        struct SDS_API Heartbeat {
            bool sensor_communication_ok { false };                                            ///< status of communication with the sensor.
            TrackingMode current_tracking_mode { TrackingMode::TrackingOFF };                  ///< current tracking mode.
            TrackingModeState current_tracking_mode_state { TrackingModeState::Initializing }; ///< state of the current tracking mode.
        };

        /**
         * @struct CurrentShutter
         * @brief A structure representing the current shutter value.
         *
         * This structure is output via a callback, and indicates the current shutter value.
         * Shutter values are between 0 and 16, where 0 is the shortest shutter and 16 is the longest.
         */
        struct SDS_API CurrentShutter {
            uint8_t shutter_value; ///< current shutter value
        };

        /**
         * @struct GyroOffset
         * @brief A structure representing a single gyro calibration offset for a specific sensor.
         *
         * This structure contains information about gyro calibration offset for a specific sensor.
         */
        struct SDS_API GyroOffset {
            uint16_t sensorId { 0 };     ///< sensor id.
            std::string datetime { "" }; ///< date time in Y-m-d H:M:S format.
            int8_t temperature { 0 };    ///< temperature in degrees. This is the internal temperature of board, not the temperature of the external environment.
            int16_t gyro_x { 0 };        ///< bias in the x-coordinate of the IMU. Units are in gyro units.
            int16_t gyro_y { 0 };        ///< bias in the y-coordinate of the IMU. Units are in gyro units.
            int16_t gyro_z { 0 };        ///< bias in the z-coordinate of the IMU. Units are in gyro units.
            uint16_t standard_dev { 0 }; ///< standard deviation of IMU noise. Units are in gyro units.
        };

        /**
         * @struct FieldOfViewReport
         * @brief A report of the beacons in the field of view.
         *
         * This structure contains information about which beacons are in the field of view of the sensor, and if they are seen on 1, 2 or 3 optical sensors.
         */
        struct SDS_API FieldOfViewReport {
            std::map<uint16_t, int> seenBeacons; ///< Map of beacon ids to the number of optical sensors that the beacon was seen on.
        };

        /**
         * @struct ShutterSettings
         * @brief Specifies the shutter settings.
         *
         * This struct is used to control the sensors shutter. There are three shutter modes:
         * - Auto shutter - the shutter setting is detected automatically by the sensor.
         * - Fixed shutter -  the shutter is fixed at a specific value and will not be changed by the sensor.
         * - Range shutter - the shutter is set automatically by the sensor but it will be fixed to a specific range.
         * 
         * For typical usage Auto shutter is recommended, Fixed and Range shutter modes are used only in cases where there are abnormal optical conditions.
         * For example when the sun is directly in the field of view of the sensor.
         * The ShutterSettings struct should be instantiated by one of the following functions:
         *  getShutterSettingsAuto, getShutterSettingsFixed, or getShutterSettingsRange.
         * 
         * Shutter values are between 0 and 16. Additionally the value 63 is used to set the sensor into Auto shutter mode.
         */
        struct SDS_API ShutterSettings {
            uint8_t min_shutter;
            uint8_t max_shutter;
        };

        /**
         * @brief Get shutter settings for Auto shutter mode.
         * @return ShutterSettings
         */
        SDS_API ShutterSettings getShutterSettingsAuto();

        /**
         * @brief Get shutter settings for Fixed shutter mode.
         * @param shutter Shutter value, between 0 and 16 inclusive.
         * @return ShutterSettings
         */
        SDS_API ShutterSettings getShutterSettingsFixed(uint8_t shutter);

         /**
         * @brief Get shutter settings for Range shutter mode.
         * @param min_shutter Min shutter value, between 0 and 16 inclusive.
         * @param max_shutter Max shutter value, between 0 and 16 inclusive.
         * @return ShutterSettings
         */
        SDS_API ShutterSettings getShutterSettingsRange(uint8_t min_shutter, uint8_t max_shutter);

        /**
         * @struct Config6Dof
         * @brief A structure representing configuration settings for the Sixdof tracking mode.
         *
         * This structure contains configuration settings for the Sixdof tracking mode.
         * The only required data is the map, other fields can be left to defaults.
         */
        struct SDS_API Config6Dof {
            std::vector<Beacon> map;                  ///< vector of beacons for Sixdof tracking. (Required)
            std::vector<GyroOffset> gyroCalibration;  ///< vector of previously recorded gyro calibration data. (Optional)
            bool skip_gyro_calibration { true };     ///< set to true to skip gyro calibration, this will speed up the initialization process.
            ShutterSettings shutter_settings { getShutterSettingsAuto() };
            bool no_gyro_mode { false };              ///< flag to disable gyro sensor, this mode should only be used in select cases, please contact the Sixdof team for guidance. (Optional)
            std::string sixdof_dump_path { "" };      ///< full path to file to log data to, default value of empty string will not log data. (Optional)
        };

        /**
         * @struct ConfigRelativeBeacon
         * @brief A structure representing configuration settings for RelativeBeacon tracking mode.
         *
         * This structure contains configuration settings for RelativeBeacon tracking mode.
         * All members can be left to default values.
         */
        struct SDS_API ConfigRelativeBeacon {
            ShutterSettings shutter_settings { getShutterSettingsAuto() };
            std::vector<uint16_t> expected_beacon_ids;  ///< expected beacon ids for this mode, typically a large beacon. This information will be used to optimize the shutter for the beacons you are looking for. (Optional)
            float field_of_view_cutoff_deg { 45.0 };    ///< field of view cutoff for RelativeBeacon mode, in degrees. (Optional)
            std::string rel_lights_dump_path { "" };    ///< full path to file to log data to, default value of empty string will not log data. (Optional)
        };

        /**
         * @struct ConfigRelativeAngle
         * @brief A structure representing configuration settings for RelativeAngle tracking mode.
         *
         * This structure contains configuration settings for RelativeAngle tracking mode.
         * All members can be left to default values.
         */
        struct SDS_API ConfigRelativeAngle {
            ShutterSettings shutter_settings { getShutterSettingsAuto() };
            std::vector<uint16_t> expected_beacon_ids;  ///< expected beacon ids for this mode, typically a large beacon. This information will be used to optimize the shutter for the beacons you are looking for. (Optional)
            bool matching_mode_single_beacon { false }; ///< set the matching mode to match based on the assumption that there is only one beacon in the feild of view. It is possible to increase the tracking range with this mode.
            float field_of_view_cutoff_deg { 45.0 };    ///< field of view cutoff for RelativeAngle mode, in degrees. (Optional)
            std::string rel_angles_dump_path { "" };    ///< full path to file to log data to, default value of empty string will not log data. (Optional)
        };

        /**
         * @struct ConfigGyroCalibration
         * @brief A structure representing configuration settings for gyro calibration.
         *
         * This structure contains configuration settings for gyro calibration,
         * all members can be left to defaults.
         */
        struct SDS_API ConfigGyroCalibration {
            int duration_seconds { 30 }; ///< duration of gyro calibration capture in seconds.
        };

        /**
         * @typedef CallbackHandle
         * @brief A handle representing a registered callback function.
         *
         * This typedef represents a callback handle used to identify registered callback functions.
         * It is used to remove a specific callback.
         */
        typedef uint32_t CallbackHandle;

        /**
         * @class DroneTrackingManager
         * @brief Class for managing the Sixdof tracking system.
         *
         * This class provides an interface for managing drone tracking.
         * The correct order of function calls is first initializeNetwork,
         * then initialize6dof, initializeRelativeBeacon, initializeRelativeAngle or multiple.
         * 
         * Finally you can start the tracking with the setTrackingMode function.
         * 
         * There are two ways to get tracking data and status messages out of the DroneTrackingManager:\n
         *    1) Get functions. these will return the most recent data point. However the getStatusMessage function will return all messages in the order they appeared.\n
         *    2) Function callbacks. The callbacks provided will get called each time a new data point is provided.
         * 
         * It is recommended to use function callbacks for status messages to ensure all messages are received immediately. 
         * It is also recommended to register for status messages before initializing the network, this way we can get all messages from the initialization stage.
         * Each function callback is run on its own thread and it only gets updated once it has finished processing its current data point.
         * This means that long running function callbacks may miss data points that got overshadowed by new data, this is by design.
         * 
         * The DroneTrackingManager also allows for the client to do gyro calibration before flight, see  the getGyroCalibration() function.
         * 
         */
        class SDS_API DroneTrackingManager {
        public:
            virtual ~DroneTrackingManager() { }

            /**
             * @brief Initialize the network.
             *
             * This function initializes the network configuration for drone tracking.
             *
             * @param NetworkConfig The network configuration.
             * @return True if the network is successfully initialized, false otherwise.
             */
            virtual bool initializeNetwork(const NetworkConfig&) = 0;

            /**
             * @brief Initialize the network.
             *
             * This function initializes the network configuration for drone tracking.
             *
             * @param NetworkAdvancedConfig The network configuration.
             * @return True if the network is successfully initialized, false otherwise.
             */
            virtual bool initializeNetwork(const NetworkAdvancedConfig&) = 0;

            /**
             * @brief Initialize Sixdof tracking mode.
             *
             * This function initializes the Sixdof tracking mode. If you are not using the Sixdof tracking mode this is unnecessary.
             *
             * @param Config6Dof The configuration for the Sixdof tracking mode.
             * @return True if Sixdof tracking is successfully initialized, false otherwise.
             */
            virtual bool initialize6Dof(const Config6Dof&) = 0;

            /**
             * @brief Initialize RelativeBeacon tracking mode.
             *
             * This function initializes the RelativeBeacon tracking mode. If you are not using the RelativeBeacon tracking mode this is unnecessary.
             *
             * @param ConfigRelativeBeacon The configuration for RelativeBeacon tracking mode.
             * @return True if RelativeBeacon tracking is successfully initialized, false otherwise.
             */
            virtual bool initializeRelativeBeacon(const ConfigRelativeBeacon&) = 0;

            /**
             * @brief Initialize RelativeAngle tracking mode.
             *
             * This function initializes the RelativeAngle tracking mode. If you are not using the RelativeAngle tracking mode this is unnecessary.
             *
             * @param ConfigRelativeAngle The configuration for RelativeAngle tracking mode.
             * @return True if RelativeAngle tracking is successfully initialized, false otherwise.
             */
            virtual bool initializeRelativeAngle(const ConfigRelativeAngle&) = 0;

            /**
             * @brief Set the tracking mode.
             *
             * @param TrackingMode The desired TrackingMode.
             */
            virtual void setTrackingMode(const TrackingMode&) = 0;

             /**
             * @brief Gets the firmware version of the sensor.
             */
            virtual Version getFirmwareVersion() = 0;

            /**
             * @brief Do gyro calibration and retrieve the results.
             *
             * This function retrieves gyro calibration data for the sensor. 
             * This data should be saved and used at a later time by passing it into the gyroCalibration member for Config6Dof
             *
             * @param ConfigGyroCalibration The configuration for the gyro calibration procedure.
             * @return A vector of GyroOffset objects representing the gyro calibration data.
             */
            virtual std::vector<GyroOffset> getGyroCalibration(const ConfigGyroCalibration&) = 0;

            virtual Pose6Dof getPose6Dof() = 0; ///< get most recent Pose6Dof.

            virtual PoseRelativeBeaconCollection getPoseRelativeBeaconCollection() = 0; ///< get most recent PoseRelativeBeaconCollection.

            virtual RelativeAngleCollection getRelativeAngleCollection() = 0; ///< get most recent RelativeAngle.

            virtual StatusMessage getStatusMessage() = 0; ///< get first StatusMessage that was called but not received yet.

            virtual FieldOfViewReport getFieldOfViewReport() = 0; ///< get most recent field of view report.

            virtual CurrentShutter getCurrentShutter() = 0; ///< get most recent shutter value.

            virtual CallbackHandle registerPose6DofCallback(const std::function<void(Pose6Dof)>&) = 0; ///< register a function callback that takes Pose6Dof as an input.

            virtual CallbackHandle registerPoseRelativeBeaconCallback(const std::function<void(PoseRelativeBeaconCollection)>&) = 0; ///< register a function callback that takes PoseRelativeBeaconCollection as an input.

            virtual CallbackHandle registerRelativeAngleCallback(const std::function<void(RelativeAngleCollection)>&) = 0; ///< register a function callback that takes RelativeAngleCollection as an input.

            virtual CallbackHandle registerMessageCallback(const std::function<void(StatusMessage)>&) = 0; ///< register a function callback that takes StatusMessage as an input.

            virtual CallbackHandle registerFieldOfViewReportCallback(const std::function<void(FieldOfViewReport)>&) = 0; ///< register a function callback that takes FieldOfViewReport as an input.

            virtual CallbackHandle registerHeartbeatCallback(const std::function<void(Heartbeat)>&) = 0; ///< register a function callback that takes Heartbeat as an input.

            virtual CallbackHandle registerCurrentShutterCallback(const std::function<void(CurrentShutter)>&) = 0; ///< register a function callback that takes a CurrentShutter as an input.

            virtual void removePose6DofCallback(const CallbackHandle&) = 0; ///<< remove a Pose6Dof function callback.

            virtual void removePoseRelativeBeaconCallback(const CallbackHandle&) = 0; ///<< remove a PoseRelativeBeacon function callback.

            virtual void removeRelativeAngleCallback(const CallbackHandle&) = 0; ///<< remove a RelativeAngle function callback.

            virtual void removeMessageCallback(const CallbackHandle&) = 0; ///<< remove a StatusMessage function callback.

            virtual void removeFieldOfViewCallback(const CallbackHandle&) = 0; ///<< remove a FieldOfViewReport function callback.

            virtual void removeHeartbeatCallback(const CallbackHandle&) = 0; ///<< remove a Heartbeat function callback.
        
            virtual void removeCurrentShutterCallback(const CallbackHandle&) = 0; ///<< remove a Current Shutter function callback. 

            /**
             * @brief Dynamically swap the map.
             *
             * This enables the user to swap the map during execution.
             * It can only be called after initialize6Dof has already been called. 
             *
             * @param map The new beacon positions.
             */
            virtual void swapMap(const std::vector<Beacon>& map) = 0;
        };

        /**
         * @brief Create an instance of DroneTrackingManager.
         *
         * This function creates a new instance of the DroneTrackingManager.
         * When the DroneTrackingManger goes out of scope it will stop the sensor and be cleaned up automatically.
         *
         * @return A unique pointer to a new instance of DroneTrackingManager.
         */
        SDS_API std::unique_ptr<DroneTrackingManager> createDroneTrackingManager();
    }
} 
