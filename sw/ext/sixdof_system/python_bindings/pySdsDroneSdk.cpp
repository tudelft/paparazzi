//Copyright © 2017-2024 Six Degrees Space Ltd.  All rights reserved.
//Proprietary and Confidential.  Unauthorized use, disclosure or reproduction is strictly prohibited.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
namespace py = pybind11;

#include "SdsDroneSdk.h"

namespace Sds {
    namespace DroneSDK {

        PYBIND11_MODULE(pySdsDroneSdk, g) {

            py::class_<Version>(g, "Version")
                .def("getMajor", &Version::getMajor)
                .def("getMinor", &Version::getMinor)
                .def("getPatch", &Version::getPatch)
                .def("isEqual", &Version::isEqual)
                .def("isAtLeast", &Version::isAtLeast)
                .def("getString", &Version::getString);

            g.def("getVersion", [](){ return getVersion(); });

            py::class_<NetworkConfig>(g, "NetworkConfig")
                .def(py::init<>())
                .def_readwrite("sensor_id", &NetworkConfig::sensor_id)
                .def_readwrite("sensor_network_name", &NetworkConfig::sensor_network_name)
                .def_readwrite("udp_read_port", &NetworkConfig::udp_read_port);

            py::class_<NetworkAdvancedConfig>(g, "NetworkAdvancedConfig")
                .def(py::init<>())
                .def_readwrite("sensor_id", &NetworkAdvancedConfig::sensor_id)
                .def_readwrite("sensor_ip", &NetworkAdvancedConfig::sensor_ip)
                .def_readwrite("udp_read_port", &NetworkAdvancedConfig::udp_read_port);

            py::class_<Beacon>(g, "Beacon")
                .def(py::init<>())
                .def_readwrite("x", &Beacon::x)
                .def_readwrite("y", &Beacon::y)
                .def_readwrite("z", &Beacon::z)
                .def_readwrite("id", &Beacon::id);

            py::class_<Pose6Dof>(g, "Pose6Dof")
                .def_readonly("valid", &Pose6Dof::valid)
                .def_readonly("x", &Pose6Dof::x)
                .def_readonly("y", &Pose6Dof::y)
                .def_readonly("z", &Pose6Dof::z)
                .def_readonly("qw", &Pose6Dof::qw)
                .def_readonly("qx", &Pose6Dof::qx)
                .def_readonly("qy", &Pose6Dof::qy)
                .def_readonly("qz", &Pose6Dof::qz)
                .def_readonly("var_x", &Pose6Dof::var_x)
                .def_readonly("var_y", &Pose6Dof::var_y)
                .def_readonly("var_z", &Pose6Dof::var_z)
                .def_readonly("var_h", &Pose6Dof::var_h)
                .def_readonly("var_p", &Pose6Dof::var_p)
                .def_readonly("var_r", &Pose6Dof::var_r);

            py::class_<PoseRelativeBeacon>(g, "PoseRelativeBeacon")
                .def_readonly("x", &PoseRelativeBeacon::x)
                .def_readonly("y", &PoseRelativeBeacon::y)
                .def_readonly("z", &PoseRelativeBeacon::z)
                .def_readonly("id", &PoseRelativeBeacon::id);

            py::bind_vector<PoseRelativeBeaconCollection>(g, "PoseRelativeBeaconCollection");

            py::class_<RelativeAngle>(g, "RelativeAngle")
                .def_readonly("x_angle", &RelativeAngle::x_angle)
                .def_readonly("z_angle", &RelativeAngle::z_angle)
                .def_readonly("id", &RelativeAngle::id)
                .def_readonly("intensity", &RelativeAngle::intensity)
                .def_readonly("width", &RelativeAngle::width);

            py::bind_vector<RelativeAngleCollection>(g, "RelativeAngleCollection");
            
            py::enum_<Severity>(g, "Severity")
                .value("Exception", Severity::Exception)
                .value("Warning", Severity::Warning)
                .value("Informative", Severity::Informative);

            py::enum_<CallingLayer>(g, "CallingLayer")
                .value("SdsDroneSdk", CallingLayer::SdsDroneSdk)
                .value("Algorithm6Dof", CallingLayer::Algorithm6Dof)
                .value("AlgorithmRelativeBeacon", CallingLayer::AlgorithmRelativeBeacon)
                .value("SdsCommLib", CallingLayer::SdsCommLib)
                .value("NetworkConnection", CallingLayer::NetworkConnection)
                .value("GyroCalibration", CallingLayer::GyroCalibration)
                .value("AlgorithmRelativeAngle", CallingLayer::AlgorithmRelativeAngle);

            py::enum_<DroneSdkEventCode>(g, "DroneSdkEventCode")
                .value("UnexpectedAlgoInstance", DroneSdkEventCode::UnexpectedAlgoInstance)
                .value("TrackingModeNotInitalized", DroneSdkEventCode::TrackingModeNotInitalized)
                .value("NetworkNotInitalized", DroneSdkEventCode::NetworkNotInitalized)
                .value("GyroCalibrationRejected", DroneSdkEventCode::GyroCalibrationRejected)
                .value("CommunicationLoopNotClosed", DroneSdkEventCode::CommunicationLoopNotClosed);

            py::class_<StatusMessage>(g, "StatusMessage")
                .def_readonly("severity", &StatusMessage::severity)
                .def_readonly("layer", &StatusMessage::layer)
                .def_readonly("event_code", &StatusMessage::event_code)
                .def_readonly("message", &StatusMessage::message);

            py::enum_<TrackingMode>(g, "TrackingMode")
                .value("TrackingOFF", TrackingMode::TrackingOFF)
                .value("RelativeBeacon", TrackingMode::RelativeBeacon)
                .value("Sixdof", TrackingMode::Sixdof)
                .value("RelativeAngle", TrackingMode::RelativeAngle);

            g.def("to_string", py::overload_cast<TrackingMode>(&to_string));

            py::enum_<TrackingModeState>(g, "TrackingModeState")
                .value("Initializing", TrackingModeState::Initializing)
                .value("Running", TrackingModeState::Running)
                .value("Error", TrackingModeState::Error);

            g.def("to_string", py::overload_cast<TrackingModeState>(&to_string));

            py::class_<Heartbeat>(g, "Heartbeat")
                .def_readonly("sensor_communication_ok", &Heartbeat::sensor_communication_ok)
                .def_readonly("current_tracking_mode", &Heartbeat::current_tracking_mode)
                .def_readonly("current_tracking_mode_state", &Heartbeat::current_tracking_mode_state);

            py::class_<CurrentShutter>(g, "CurrentShutter")
                .def_readonly("shutter_value", &CurrentShutter::shutter_value);

            py::class_<GyroOffset>(g, "GyroOffset")
                .def_readonly("sensorId", &GyroOffset::sensorId)
                .def_readonly("datetime", &GyroOffset::datetime)
                .def_readonly("temperature", &GyroOffset::temperature)
                .def_readonly("gyro_x", &GyroOffset::gyro_x)
                .def_readonly("gyro_y", &GyroOffset::gyro_y)
                .def_readonly("gyro_z", &GyroOffset::gyro_z)
                .def_readonly("standard_dev", &GyroOffset::standard_dev);

            py::class_<FieldOfViewReport>(g, "FieldOfViewReport")
                .def_readonly("seenBeacons", &FieldOfViewReport::seenBeacons);

            py::class_<ShutterSettings>(g, "ShutterSettings")
                .def_readonly("max_shutter", &ShutterSettings::max_shutter)
                .def_readonly("min_shutter", &ShutterSettings::min_shutter);

            g.def("getShutterSettingsAuto", &getShutterSettingsAuto);
            g.def("getShutterSettingsFixed", &getShutterSettingsFixed);
            g.def("getShutterSettingsRange", &getShutterSettingsRange);

            py::class_<Config6Dof>(g, "Config6Dof")
                .def(py::init<>())
                .def_readwrite("map", &Config6Dof::map)
                .def_readwrite("gyroCalibration", &Config6Dof::gyroCalibration)
                .def_readwrite("skip_gyro_calibration", &Config6Dof::skip_gyro_calibration)
                .def_readwrite("shutter_settings", &Config6Dof::shutter_settings)
                .def_readwrite("no_gyro_mode", &Config6Dof::no_gyro_mode)
                .def_readwrite("sixdof_dump_path", &Config6Dof::sixdof_dump_path);

            py::class_<ConfigRelativeBeacon>(g, "ConfigRelativeBeacon")
                .def(py::init<>())
                .def_readwrite("shutter_settings", &ConfigRelativeBeacon::shutter_settings)
                .def_readwrite("expected_beacon_ids", &ConfigRelativeBeacon::expected_beacon_ids)
                .def_readwrite("field_of_view_cutoff_deg", &ConfigRelativeBeacon::field_of_view_cutoff_deg)
                .def_readwrite("rel_lights_dump_path", &ConfigRelativeBeacon::rel_lights_dump_path);

            py::class_<ConfigRelativeAngle>(g, "ConfigRelativeAngle")
                .def(py::init<>())
                .def_readwrite("shutter_settings", &ConfigRelativeAngle::shutter_settings)
                .def_readwrite("expected_beacon_ids", &ConfigRelativeAngle::expected_beacon_ids)
                .def_readwrite("field_of_view_cutoff_deg", &ConfigRelativeAngle::field_of_view_cutoff_deg)
                .def_readwrite("rel_angles_dump_path", &ConfigRelativeAngle::rel_angles_dump_path);

            py::class_<ConfigGyroCalibration>(g, "ConfigGyroCalibration")
                .def(py::init<>())
                .def_readwrite("duration_seconds", &ConfigGyroCalibration::duration_seconds);

            py::class_<DroneTrackingManager>(g, "DroneTrackingManager")
                .def("initializeNetwork", py::overload_cast<const NetworkConfig&>(&DroneTrackingManager::initializeNetwork))
                .def("initializeNetwork", py::overload_cast<const NetworkAdvancedConfig&>(&DroneTrackingManager::initializeNetwork))
                .def("initialize6Dof", &DroneTrackingManager::initialize6Dof)
                .def("initializeRelativeBeacon", &DroneTrackingManager::initializeRelativeBeacon)
                .def("initializeRelativeAngle", &DroneTrackingManager::initializeRelativeAngle)
                .def("setTrackingMode", &DroneTrackingManager::setTrackingMode)
                .def("getFirmwareVersion", &DroneTrackingManager::getFirmwareVersion)
                .def("getGyroCalibration", &DroneTrackingManager::getGyroCalibration)
                .def("getPose6Dof", &DroneTrackingManager::getPose6Dof)
                .def("getPoseRelativeBeaconCollection", &DroneTrackingManager::getPoseRelativeBeaconCollection)
                .def("getRelativeAngleCollection", &DroneTrackingManager::getRelativeAngleCollection)
                .def("getStatusMessage", &DroneTrackingManager::getStatusMessage)
                .def("getFieldOfViewReport", &DroneTrackingManager::getFieldOfViewReport)
                .def("getCurrentShutter", &DroneTrackingManager::getCurrentShutter)
                .def("registerPose6DofCallback", &DroneTrackingManager::registerPose6DofCallback)
                .def("registerPoseRelativeBeaconCallback", &DroneTrackingManager::registerPoseRelativeBeaconCallback)
                .def("registerRelativeAngleCallback", &DroneTrackingManager::registerRelativeAngleCallback)
                .def("registerMessageCallback", &DroneTrackingManager::registerMessageCallback)
                .def("registerFieldOfViewReportCallback", &DroneTrackingManager::registerFieldOfViewReportCallback)
                .def("registerHeartbeatCallback", &DroneTrackingManager::registerHeartbeatCallback)
                .def("registerCurrentShutterCallback", &DroneTrackingManager::registerCurrentShutterCallback)
                .def("removePose6DofCallback", &DroneTrackingManager::removePose6DofCallback)
                .def("removePoseRelativeBeaconCallback", &DroneTrackingManager::removePoseRelativeBeaconCallback)
                .def("removeRelativeAngleCallback", &DroneTrackingManager::removeRelativeAngleCallback)
                .def("removeMessageCallback", &DroneTrackingManager::removeMessageCallback)
                .def("removeFieldOfViewCallback", &DroneTrackingManager::removeFieldOfViewCallback)
                .def("removeHeartbeatCallback", &DroneTrackingManager::removeHeartbeatCallback)
                .def("removeCurrentShutterCallback", &DroneTrackingManager::removeCurrentShutterCallback)
                .def("swapMap", &DroneTrackingManager::swapMap);

            g.def("createDroneTrackingManager", &createDroneTrackingManager);

        }

    }
}
