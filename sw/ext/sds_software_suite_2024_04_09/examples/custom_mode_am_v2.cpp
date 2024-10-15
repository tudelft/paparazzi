#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include "SdsDroneSdk.h"
#include <string>
#include <sstream>
#include <glib.h>
#include <ivy.h>
#include <ivyglibloop.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include "custom_mode_am_v2.h"
#include <functional>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

//Library configuration parameters:
using namespace Sds::DroneSDK;
ConfigRelativeBeacon config_rel_beacon; // use defaults
ConfigRelativeAngle config_rel_angle; // use defaults
Config6Dof config6Dof; // use defaults
std::unique_ptr<DroneTrackingManager> droneManager;

#if CALIBRATE_GYROS
    std::vector<GyroOffset> gyroCalibration;
#endif

// Variables to set and check the behavior of the program:
int verbose_rx = 0; 
int verbose_tx = 0; 
int send_values_on_ivy = 1; 
int produce_log_file = 0; 

//Variable to store the current mode of the system, with default value of 1 (rel beacon pos):
int current_sixdof_mode = 1; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 

//Input map in the reference frame of the landing pad.
Beacon b1 {0.288, -0.222, -0.005, 1640};
Beacon b2 {0.265, 0, -0.03, 1636};
Beacon b3 {0.292, 0.224, -0.005, 1645};
Beacon b4 {-0.298, 0.23, -0.005, 1632};
Beacon b5 {-0.30, -0.222, -0.005, 1633};

//variables for the ivy bus messages: 
int sixdof_msg_available = 0, current_mode_msg_available = 0, beacon_pos_msg_available = 0, relative_angle_msg_available = 0; 
pthread_mutex_t mutex_ivy_bus = PTHREAD_MUTEX_INITIALIZER;

//Structure to store the sixdof packet
struct register_sixdof_packet my_sixdof_packet; 

//Structure to store beacon data
struct register_beacon_packet my_beacon_packet;

//Structure to store the Field of View report:
struct fov_report_packet my_fov_report_packet;

//Structure to store the relative angle data:
struct register_rel_angle_packet my_rel_angle_packet;

// File pointer for logging
FILE* log_file = nullptr;
int msg_id_fov = 1, msg_id_rel_pos = 2, msg_id_rel_angle = 3;
pthread_mutex_t mutex_logger = PTHREAD_MUTEX_INITIALIZER;

// Function to convert a quaternion to Euler angles
static void quaternion_to_euler(float q[4], float euler[3]) {
    // Extract the values from the quaternion
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // Calculate the Euler angles from the quaternion
    float ysqr = qy * qy;

    // Pitch (x-axis rotation) -> was Roll
    float t0 = +2.0 * (qw * qx + qy * qz);
    float t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    euler[1] = atan2f(t0, t1); // Pitch
    // Adjust Pitch by subtracting 90 degrees (π/2)
    euler[1] -= M_PI_2;
    // Clamp between pi/2 and -pi/2: 
    euler[1] = fmaxf(-M_PI_2, fminf(M_PI_2, euler[1]));

    // Roll (y-axis rotation) -> was Pitch, with sign swapped
    float t2 = +2.0 * (qw * qy - qz * qx);
    t2 = fmaxf(-1.0, fminf(1.0, t2)); // Clamp to prevent NaN
    euler[0] = -asinf(t2); // Roll with inverted sign

    // Yaw (z-axis rotation) with offset adjustment
    float t3 = +2.0 * (qw * qz + qx * qy);
    float t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    euler[2] = atan2f(t3, t4) - M_PI_2; // Adjust yaw by -90 degrees (π/2)

    // Normalize yaw to the range [-π, π]
    if (euler[2] < -M_PI) {
        euler[2] += 2.0 * M_PI;
    } else if (euler[2] > M_PI) {
        euler[2] -= 2.0 * M_PI;
    }
}

// Function to manage the ivy bus communication going out: 
void ivy_bus_out_handle() {
    while(true){
        
        //Check if new 6dof position message is available and send it to the ivy bus: 
        pthread_mutex_lock(&mutex_ivy_bus);
        int sixdof_msg_available_local = sixdof_msg_available;
        int current_mode_msg_available_local = current_mode_msg_available;
        int beacon_pos_msg_available_local = beacon_pos_msg_available;
        int relative_angle_msg_available_local = relative_angle_msg_available;
        sixdof_msg_available = 0;
        current_mode_msg_available = 0;
        beacon_pos_msg_available = 0;
        relative_angle_msg_available = 0;
        pthread_mutex_unlock(&mutex_ivy_bus);

        if(sixdof_msg_available_local){
            //Copy the struct values to a local one:
            static struct register_sixdof_packet my_sixdof_packet_local; 
            pthread_mutex_lock(&mutex_ivy_bus);
            memcpy(&my_sixdof_packet_local, &my_sixdof_packet, sizeof(struct register_sixdof_packet));
            pthread_mutex_unlock(&mutex_ivy_bus); 

            //Prepare message with snprintf: 
            char msg_ivy[256];
            snprintf(msg_ivy, sizeof(msg_ivy), "SIXDOF_TRACKING %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f", 
                my_sixdof_packet_local.timestamp_position,
                my_sixdof_packet_local.x_abs_pos, my_sixdof_packet_local.y_abs_pos, my_sixdof_packet_local.z_abs_pos,
                my_sixdof_packet_local.relative_phi_rad, my_sixdof_packet_local.relative_theta_rad,
                my_sixdof_packet_local.relative_psi_rad, 
                my_sixdof_packet_local.quat_w, my_sixdof_packet_local.quat_x, my_sixdof_packet_local.quat_y, my_sixdof_packet_local.quat_z,
                my_sixdof_packet_local.x_abs_pos_var, my_sixdof_packet_local.y_abs_pos_var, my_sixdof_packet_local.z_abs_pos_var,
                my_sixdof_packet_local.phi_rad_var, my_sixdof_packet_local.theta_rad_var,
                my_sixdof_packet_local.psi_rad_var);

            //Send vaues to the ivy bus:
            if(send_values_on_ivy) IvySendMsg("%s",msg_ivy);
                
            if(send_values_on_ivy && verbose_tx) printf("%s\n", msg_ivy);

        }

        //Check if new mode message is available and send it to the ivy bus: 
        if(current_mode_msg_available_local){
            struct timespec ts;
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 

            //Prepare message with snprintf:
            char msg_ivy[256];
            snprintf(msg_ivy, sizeof(msg_ivy), "SIXDOF_SYSTEM_CURRENT_MODE %.5f %d", current_timestamp, current_sixdof_mode);

            //Send values to the ivy bus:
            if(send_values_on_ivy) IvySendMsg("%s",msg_ivy);

            if(send_values_on_ivy && verbose_tx) printf("%s\n", msg_ivy);
        }       

        //Check if new beacon position message is available and send it to the ivy bus: 
        if(beacon_pos_msg_available_local){
            //Copy the struct values to a local one:
            static struct register_beacon_packet my_beacon_packet_local;
            pthread_mutex_lock(&mutex_ivy_bus);
            memcpy(&my_beacon_packet_local, &my_beacon_packet, sizeof(struct register_beacon_packet));
            pthread_mutex_unlock(&mutex_ivy_bus);
            for(int i = 0; i < N_BEACON; i++){
                if(send_values_on_ivy && my_beacon_packet_local.beacon_id[i] != 0){

                    //Prepare message with snprintf:
                    char msg_ivy[256];
                    snprintf(msg_ivy, sizeof(msg_ivy), "RELATIVE_BEACON_POS %.5f %d %.5f %.5f %.5f ", 
                        my_beacon_packet_local.timestamp_beacon[i], my_beacon_packet_local.beacon_id[i], 
                        my_beacon_packet_local.x_body_pos_beacon[i], my_beacon_packet_local.y_body_pos_beacon[i], 
                        my_beacon_packet_local.z_body_pos_beacon[i]);

                    //Send values to the ivy bus:
                    IvySendMsg("%s",msg_ivy);

                    if(verbose_tx) printf("%s\n", msg_ivy);
                }
            }
        }   
        
        //Check if new relative angle message is available and send it to the ivy bus:
        if(relative_angle_msg_available_local){
            //Copy the struct values to a local one:
            static struct register_rel_angle_packet my_rel_angle_packet_local;
            pthread_mutex_lock(&mutex_ivy_bus);
            memcpy(&my_rel_angle_packet_local, &my_rel_angle_packet, sizeof(struct register_rel_angle_packet));
            pthread_mutex_unlock(&mutex_ivy_bus);
            for(int i = 0; i < N_BEACON; i++){
                if(send_values_on_ivy && my_rel_angle_packet_local.beacon_id[i] != 0){
                    //Prepare message with snprintf:
                    char msg_ivy[256];
                    snprintf(msg_ivy, sizeof(msg_ivy), "RELATIVE_BEACON_ANGLE %.5f %d %.5f %.5f %.5f %.5f", 
                        my_rel_angle_packet_local.timestamp_rel_angle[i], my_rel_angle_packet_local.beacon_id[i], 
                        my_rel_angle_packet_local.x_angle_rad[i], my_rel_angle_packet_local.z_angle_rad[i], 
                        my_rel_angle_packet_local.intensity[i], my_rel_angle_packet_local.width[i]);
                    
                    //Send values to the ivy bus:
                    IvySendMsg("%s",msg_ivy);

                    if(verbose_tx) printf("%s\n", msg_ivy);

                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(5)); // Sleep for 5 microseconds to avoid overloading the CPU 
    } 
}

static void ivy_set_rel_beacon_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
    if(current_sixdof_mode != 1){
        bool sixdofSuccess = droneManager->initializeRelativeBeacon(config_rel_beacon);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed relative beacon setup" << std::endl;
            current_sixdof_mode = -1;
        }
        else{
            current_sixdof_mode = 1;
        }

        #if CALIBRATE_GYROS
        ConfigGyroCalibration config;
        config.duration_seconds = 10;
        std::cout << "Doing Gyro Calibration, please keep the sensor still" << std::endl;
        gyroCalibration = droneManager->getGyroCalibration(config);
        std::cout << "Finished Gyro Calibration, now you can move the sensor" << std::endl;
        for (const GyroOffset& c : gyroCalibration) {
            std::cout << std::endl;
            std::cout  << "sensor_id: " << c.sensorId << std::endl 
                        << "datetime: " << c.datetime << std::endl 
                        << "temperature: " << (int)c.temperature << std::endl
                        << "gyro_bias: " << c.gyro_x << " " << c.gyro_y << " " << c.gyro_z << " " << std::endl
                        << "std: " <<c.standard_dev << std::endl;
        }
        #endif

        // Start tracking
        std::cout << "TrackingMode RelativeBeacon" << std::endl;
        droneManager->setTrackingMode(TrackingMode::RelativeBeacon);
        pthread_mutex_lock(&mutex_ivy_bus);
        current_mode_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 
    }
}

static void ivy_set_rel_angle_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
    if(current_sixdof_mode != 2){
        std::cout << "Switching to tracking mode = RelativeAngle" << std::endl;
        bool sixdofSuccess = droneManager->initializeRelativeAngle(config_rel_angle);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed relative angle setup" << std::endl;
            current_sixdof_mode = -1;
        }
        else{
            current_sixdof_mode = 2;
        }
        droneManager->setTrackingMode(TrackingMode::RelativeAngle);
        pthread_mutex_lock(&mutex_ivy_bus);
        current_mode_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 

        #if CALIBRATE_GYROS
        ConfigGyroCalibration config;
        config.duration_seconds = 10;
        std::cout << "Doing Gyro Calibration, please keep the sensor still" << std::endl;
        gyroCalibration = droneManager->getGyroCalibration(config);
        std::cout << "Finished Gyro Calibration, now you can move the sensor" << std::endl;
        for (const GyroOffset& c : gyroCalibration) {
            std::cout << std::endl;
            std::cout  << "sensor_id: " << c.sensorId << std::endl 
                        << "datetime: " << c.datetime << std::endl 
                        << "temperature: " << (int)c.temperature << std::endl
                        << "gyro_bias: " << c.gyro_x << " " << c.gyro_y << " " << c.gyro_z << " " << std::endl
                        << "std: " <<c.standard_dev << std::endl;
        }
        #endif
    }
}

static void ivy_set_sixdof_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{

    if(current_sixdof_mode != 3){
        config6Dof.map.push_back(b1);
        #if INCLUDE_HIGH_POW_BEC_IN_SIXDOF_MAP
            config6Dof.map.push_back(b2);   
        #endif
        config6Dof.map.push_back(b3);
        config6Dof.map.push_back(b4);
        config6Dof.map.push_back(b5);

        #if DISABLE_GYRO_SIXDOF
            config6Dof.no_gyro_mode = true;
        #endif

        #if DISABLE_GYRO_SIXDOF != 1
            // Manually setting the gyro calibration data based on the values you provided
            GyroOffset predefinedCalibration;
            predefinedCalibration.sensorId = 158;
            predefinedCalibration.datetime = "2024-10-14 12:00:00"; 
            predefinedCalibration.temperature = 58;  
            predefinedCalibration.gyro_x = 41;     
            predefinedCalibration.gyro_y = -29;     
            predefinedCalibration.gyro_z = -13;   
            predefinedCalibration.standard_dev = 1;  
            // Store the calibration in a vector
            std::vector<GyroOffset> gyroCalibration = { predefinedCalibration };
            // Initialize the 6DoF configuration with the predefined gyro calibration
            config6Dof.gyroCalibration = gyroCalibration;  // Assign the manually set calibration data
        #endif

        bool sixdofSuccess = droneManager->initialize6Dof(config6Dof);
        if (!sixdofSuccess) {
            std::cout << "Drone Manager failed sixdof setup" << std::endl;
            current_sixdof_mode = -1;
        }
        else{
            current_sixdof_mode = 3;
        }

        // Start tracking
        std::cout << "TrackingMode Sixdof" << std::endl;
        droneManager->setTrackingMode(TrackingMode::Sixdof);
        
        pthread_mutex_lock(&mutex_ivy_bus);
        current_mode_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 

        #if CALIBRATE_GYROS
        ConfigGyroCalibration config;
        config.duration_seconds = 10;
        std::cout << "Doing Gyro Calibration, please keep the sensor still" << std::endl;
        gyroCalibration = droneManager->getGyroCalibration(config);
        std::cout << "Finished Gyro Calibration, now you can move the sensor" << std::endl;
        for (const GyroOffset& c : gyroCalibration) {
            std::cout << std::endl;
            std::cout  << "sensor_id: " << c.sensorId << std::endl 
                        << "datetime: " << c.datetime << std::endl 
                        << "temperature: " << (int)c.temperature << std::endl
                        << "gyro_bias: " << c.gyro_x << " " << c.gyro_y << " " << c.gyro_z << " " << std::endl
                        << "std: " <<c.standard_dev << std::endl;
        }
        #endif
    }
}

// Function to generate log file name based on the current time of day
std::string generateLogFileName() {
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "Sixdof_log_%Y%m%d_%H%M%S.txt", timeinfo);
    return std::string(buffer);
}

// Initialize the log file in the "logs" subfolder
void initializeLogFile() {
    // Create the "logs" subfolder if it doesn't exist
    const char* folder_name = "logs";
    if(produce_log_file){
        mkdir(folder_name, 0777);
        std::string log_file_name = std::string(folder_name) + "/" + generateLogFileName();
        log_file = fopen(log_file_name.c_str(), "a");
        if (log_file == nullptr) {
            perror("Failed to open log file");
        }
    }

}

/* This is an example of Relative Beacon tracking */
int main(int ac, const char *av[]) {

    std::cout << "Init ivy bus" << std::endl;
    //Initialize the Ivy bus address based on the operating system

    const char* ivy_bus;
    #ifdef __APPLE__
        ivy_bus = "224.255.255.255";
    #else
        ivy_bus = "127.255.255.255";
    #endif

    // Initialize the GLib main loop
    GMainLoop *ml = g_main_loop_new(NULL, FALSE);

    // Initialize log file
    initializeLogFile();

    Version version = getVersion();
    std::cout << "Using SdsDroneSdk version: " << version.getString() << std::endl;

    // Initialize DroneTrackingManager
    droneManager = createDroneTrackingManager();
    std::cout << "Drone Manager created" << std::endl;

    // Register callbacks before doing anything else so we can get error messages printed out
    droneManager->registerMessageCallback([](const StatusMessage& msg){
        std::cout << msg.message << std::endl; 
    });

    droneManager->registerHeartbeatCallback([](const Heartbeat& hb){
        //TODO: populate the heartbeat message like mode etc..
        pthread_mutex_lock(&mutex_ivy_bus);
        current_mode_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 
    });

    droneManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        //Prepare the structure to store the relative angle data:
        struct register_rel_angle_packet my_rel_angle_packet_local;
        //Zero all the indexes for the ivy comm:
        for (int i = 0; i < N_BEACON; i++){
            my_rel_angle_packet_local.beacon_id[i] = 0;
        }
        int j = 0;
        for (const RelativeAngle& ra : col) { 
            struct timespec ts;
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 

            my_rel_angle_packet_local.timestamp_rel_angle[j] =(float) current_timestamp;
            my_rel_angle_packet_local.beacon_id[j] = (int) ra.id;
            my_rel_angle_packet_local.x_angle_rad[j] =(float) ra.x_angle;
            my_rel_angle_packet_local.z_angle_rad[j] =(float) ra.z_angle;
            my_rel_angle_packet_local.intensity[j] =(float) ra.intensity;
            my_rel_angle_packet_local.width[j] =(float) ra.width;
            
            //Log the data to the log file if needed:
            if (log_file != nullptr && produce_log_file && my_rel_angle_packet_local.beacon_id[j] != 0) {
                pthread_mutex_lock(&mutex_logger);
                fprintf(log_file, "Msg_id: %d, Timestamp: %f, Beacon ID: %d, x_angle_rad: %f, z_angle_rad: %f, Intensity: %f, Width: %f\n",
                        msg_id_rel_angle, my_rel_angle_packet_local.timestamp_rel_angle[j], my_rel_angle_packet_local.beacon_id[j], 
                        my_rel_angle_packet_local.x_angle_rad[j], my_rel_angle_packet_local.z_angle_rad[j], my_rel_angle_packet_local.intensity[j], my_rel_angle_packet_local.width[j]);
                fflush(log_file);
                pthread_mutex_unlock(&mutex_logger);
            }
            
            j++;
        }
        
        //Save the data to the global variable:
        pthread_mutex_lock(&mutex_ivy_bus);
        memcpy(&my_rel_angle_packet, &my_rel_angle_packet_local, sizeof(struct register_rel_angle_packet));
        relative_angle_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus);
    });

    droneManager->registerFieldOfViewReportCallback([](const FieldOfViewReport& report) {

        // Process the FieldOfViewReport
        int j = 0;
        for (const auto& entry : report.seenBeacons) {
            struct timespec ts;
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec * 1e-9;

            my_fov_report_packet.timestamp_fov[j] = current_timestamp;
            my_fov_report_packet.fov_beacon_id[j] = entry.first;    // Beacon ID
            my_fov_report_packet.sensor_seen_count[j] = entry.second;      // Number of optical sensors that saw the beacon

            // Log the data to the log file if needed:
            if (log_file != nullptr && produce_log_file) {
                pthread_mutex_lock(&mutex_logger);
                fprintf(log_file, "Msg_id: %d, Timestamp: %f, Beacon ID: %d, Seen Count: %d\n",
                        msg_id_fov, my_fov_report_packet.timestamp_fov[j], my_fov_report_packet.fov_beacon_id[j], my_fov_report_packet.sensor_seen_count[j]);
                fflush(log_file);
                pthread_mutex_unlock(&mutex_logger);
            }
            j++;
        }
    });

    droneManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        //Prepare the structure to store the beacon data:
        struct register_beacon_packet my_beacon_packet_local;

        //zeros all the indexes for the ivy comm: 
        for (int i = 0; i < N_BEACON; i++){
            my_beacon_packet_local.beacon_id[i] = 0;
        }
        //copy values on structure
        int j = 0;
        for (const PoseRelativeBeacon& b : col) {
            struct timespec ts;
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 

            my_beacon_packet_local.timestamp_beacon[j] =(float) current_timestamp; 
            my_beacon_packet_local.beacon_id[j] = (int) b.id; 
            my_beacon_packet_local.x_body_pos_beacon[j] = (float) b.z; 
            my_beacon_packet_local.y_body_pos_beacon[j] = (float) b.x;
            my_beacon_packet_local.z_body_pos_beacon[j] = (float) b.y;

            //Log the data to the log file if needed:
            if (log_file != nullptr && produce_log_file && my_beacon_packet_local.beacon_id[j] != 0) {
                pthread_mutex_lock(&mutex_logger);
                fprintf(log_file, "Msg_id: %d, Timestamp: %f, Beacon ID: %d, x_body_pos_beacon: %f, y_body_pos_beacon: %f, z_body_pos_beacon: %f\n",
                        msg_id_rel_pos, my_beacon_packet_local.timestamp_beacon[j], 
                        my_beacon_packet_local.beacon_id[j], my_beacon_packet_local.x_body_pos_beacon[j],
                        my_beacon_packet_local.y_body_pos_beacon[j], my_beacon_packet_local.z_body_pos_beacon[j]);
                fflush(log_file);
                pthread_mutex_unlock(&mutex_logger);
            }

            j++;
        }    
        //Save the data to the global variable:
        pthread_mutex_lock(&mutex_ivy_bus);
        memcpy(&my_beacon_packet, &my_beacon_packet_local, sizeof(struct register_beacon_packet));
        beacon_pos_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus);
    });

    droneManager->registerPose6DofCallback([](const Pose6Dof& sd){
        if(sd.valid){
            struct timespec ts;
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9;
            
            float quat_array[4] = {(float) sd.qw, (float) sd.qx, (float) sd.qy, (float) sd.qz}; //should be in the order qw, qz, qx, -qy
            float euler_angles[3];
            quaternion_to_euler(quat_array, euler_angles);

            static struct register_sixdof_packet my_sixdof_packet_local; 

            my_sixdof_packet_local.timestamp_position = current_timestamp; 
            my_sixdof_packet_local.x_abs_pos = (float) sd.x; 
            my_sixdof_packet_local.y_abs_pos = (float) sd.y; 
            my_sixdof_packet_local.z_abs_pos = (float) sd.z; 
            my_sixdof_packet_local.relative_phi_rad = (float) euler_angles[0];
            my_sixdof_packet_local.relative_theta_rad = (float) euler_angles[1];
            my_sixdof_packet_local.relative_psi_rad = (float) euler_angles[2];
            my_sixdof_packet_local.quat_w = (float) sd.qw;
            my_sixdof_packet_local.quat_x = (float) sd.qx;
            my_sixdof_packet_local.quat_y = (float) sd.qy;
            my_sixdof_packet_local.quat_z = (float) sd.qz;
            my_sixdof_packet_local.x_abs_pos_var = (float) sd.var_x; 
            my_sixdof_packet_local.y_abs_pos_var = (float) sd.var_y; 
            my_sixdof_packet_local.z_abs_pos_var = (float) sd.var_y; 
            my_sixdof_packet_local.phi_rad_var = (float) sd.var_h;
            my_sixdof_packet_local.theta_rad_var = (float) sd.var_p;
            my_sixdof_packet_local.psi_rad_var = (float) sd.var_r;

            pthread_mutex_lock(&mutex_ivy_bus);
            memcpy(&my_sixdof_packet, &my_sixdof_packet_local, sizeof(struct register_sixdof_packet));
            sixdof_msg_available = 1;
            pthread_mutex_unlock(&mutex_ivy_bus); 
        }
    });

    // Initialize network, this can only be done once
    NetworkConfig networkConfiguration { 158, "enx000000000600" };
    bool networkSuccess = droneManager->initializeNetwork(networkConfiguration);
    if (!networkSuccess) {
        std::cout << "Drone Manager failed network setup" << std::endl;
        return 1;
    }

    //Default state
    bool sixdofSuccess = droneManager->initializeRelativeBeacon(config_rel_beacon);
    if (!sixdofSuccess) {
        std::cout << "Drone Manager failed relative beacon setup" << std::endl;
        return 1;
    }
    // Start tracking
    std::cout << "TrackingMode RelativeBeacon" << std::endl;
    droneManager->setTrackingMode(TrackingMode::RelativeBeacon);

    IvyInit ("SixDofSysSensor", "SixDofSysSensor READY", NULL, NULL, NULL, NULL);
    IvyBindMsg(ivy_set_rel_beacon_mode, NULL, "SET_SIXDOF_SYS_MODE %d",1);
    IvyBindMsg(ivy_set_rel_angle_mode, NULL, "SET_SIXDOF_SYS_MODE %d",2);
    IvyBindMsg(ivy_set_sixdof_mode, NULL, "SET_SIXDOF_SYS_MODE %d",3);
    IvyStart(ivy_bus);

    // Create a thread that manages the ivybus messages going out: 
    std::thread newThread(ivy_bus_out_handle);

    g_main_loop_run(ml);

    std::cout << "Done" << std::endl;
    return 0;
}