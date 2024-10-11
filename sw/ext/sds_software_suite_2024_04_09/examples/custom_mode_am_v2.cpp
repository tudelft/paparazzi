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


using namespace Sds::DroneSDK;

ConfigRelativeBeacon config_rel_beacon; // use defaults
ConfigRelativeAngle config_rel_angle; // use defaults
Config6Dof config6Dof; // use defaults

float euler_angles[3], UAV_ned_pos[3]; 

std::unique_ptr<DroneTrackingManager> droneManager;

int verbose_rx = 0; 
int verbose_tx = 0; 
int send_values_on_ivy = 1; 
int produce_log_file = 1; 

int current_sixdof_mode = 1; //1 -->rel beacon pos; 2 -->rel beacon angle; 3-->sixdof mode. Default is 1. 

struct timespec ts;
#define N_BEACON 5

// input map
Beacon b1 {0.288, -0.222, -0.005, 1640};
Beacon b2 {0.265, 0, -0.03, 1636};
Beacon b3 {0.292, 0.224, -0.005, 1645};
Beacon b4 {-0.298, 0.23, -0.005, 1632};
Beacon b5 {-0.30, -0.222, -0.005, 1633};

//Sixdof messages: 
struct register_sixdof_packet my_sixdof_packet; 
int sixdof_msg_available = 0, current_mode_msg_available = 0, beacon_pos_msg_available = 0; 

double timestamp_beacon[N_BEACON];
int beacon_id[N_BEACON]; 
float x_body_pos_beacon[N_BEACON];
float y_body_pos_beacon[N_BEACON];
float z_body_pos_beacon[N_BEACON];

// Arrays to store beacon data for logging
int fov_beacon_id[N_BEACON]; // Array to store beacon IDs
double fov_timestamp_beacon[N_BEACON]; // Array to store timestamps
int seen_count[N_BEACON]; // Array to store the count of seen beacons

// File pointer for logging
FILE* log_file = nullptr;
int msg_id_fov = 1, msg_id_rel_pos = 2;

pthread_mutex_t mutex_ivy_bus = PTHREAD_MUTEX_INITIALIZER, mutex_logger = PTHREAD_MUTEX_INITIALIZER;

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
        if(sixdof_msg_available){
            static struct register_sixdof_packet my_sixdof_packet_local; 
            pthread_mutex_lock(&mutex_ivy_bus);
            memcpy(&my_sixdof_packet_local, &my_sixdof_packet, sizeof(struct register_sixdof_packet));
            sixdof_msg_available = 0;
            pthread_mutex_unlock(&mutex_ivy_bus); 

            if(send_values_on_ivy){
                IvySendMsg("SIXDOF_TRACKING %f %f %f %f %f %f %f %f %f %f %f %f %f", my_sixdof_packet_local.timestamp_position, my_sixdof_packet_local.x_body_pos, my_sixdof_packet_local.y_body_pos, my_sixdof_packet_local.z_body_pos, my_sixdof_packet_local.relative_phi_rad, my_sixdof_packet_local.relative_theta_rad, my_sixdof_packet_local.relative_psi_rad, my_sixdof_packet_local.x_body_pos_var, my_sixdof_packet_local.y_body_pos_var, my_sixdof_packet_local.z_body_pos_var, my_sixdof_packet_local.phi_rad_var, my_sixdof_packet_local.theta_rad_var, my_sixdof_packet_local.psi_rad_var);
            }
                
            if(verbose_tx){
                printf("SIXDOF_TRACKING %f %f %f %f %f %f %f %f %f %f %f %f %f", my_sixdof_packet_local.timestamp_position, my_sixdof_packet_local.x_body_pos, my_sixdof_packet_local.y_body_pos, my_sixdof_packet_local.z_body_pos, my_sixdof_packet_local.phi_rad_var, my_sixdof_packet_local.theta_rad_var, my_sixdof_packet_local.psi_rad_var, my_sixdof_packet_local.x_body_pos_var, my_sixdof_packet_local.y_body_pos_var, my_sixdof_packet_local.z_body_pos_var, my_sixdof_packet_local.phi_rad_var, my_sixdof_packet_local.theta_rad_var, my_sixdof_packet_local.psi_rad_var);
                printf("\n");
            } 

        }

        //Check if new mode message is available and send it to the ivy bus: 
        if(current_mode_msg_available){
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 

            if(send_values_on_ivy){
                IvySendMsg("SIXDOF_SYSTEM_CURRENT_MODE %f %d", current_timestamp, current_sixdof_mode);  
            }
            
           if(verbose_tx){
                printf("SIXDOF_SYSTEM_CURRENT_MODE %f %d", current_timestamp, current_sixdof_mode);
                printf("\n");
            }
            pthread_mutex_lock(&mutex_ivy_bus);
            current_mode_msg_available = 0;
            pthread_mutex_unlock(&mutex_ivy_bus); 
        }       

        //Check if new beacon position message is available and send it to the ivy bus: 
        if(beacon_pos_msg_available){
            for(int i = 0; i < N_BEACON; i++){
                if(send_values_on_ivy && beacon_id[i] != 0){
                    //Send values over IVYBUS
                    IvySendMsg("RELATIVE_BEACON_POS %f %d %f %f %f ", timestamp_beacon[i], beacon_id[i], x_body_pos_beacon[i], y_body_pos_beacon[i], z_body_pos_beacon[i]);
                }
                if(verbose_tx){
                    printf("RELATIVE_BEACON_POS %f %d %f %f %f ", timestamp_beacon[i], beacon_id[i], x_body_pos_beacon[i], y_body_pos_beacon[i], z_body_pos_beacon[i]);
                    printf("\n");
                }
            }
            pthread_mutex_lock(&mutex_ivy_bus);
            beacon_pos_msg_available = 0;
            pthread_mutex_unlock(&mutex_ivy_bus); 
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

    }
}

static void ivy_set_sixdof_mode(IvyClientPtr app, void *user_data, int argc, char *argv[])
{

    if(current_sixdof_mode != 3){
        config6Dof.map.push_back(b1);
        config6Dof.map.push_back(b2);
        config6Dof.map.push_back(b3);
        config6Dof.map.push_back(b4);
        config6Dof.map.push_back(b5);

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
    mkdir(folder_name, 0777);

    std::string log_file_name = std::string(folder_name) + "/" + generateLogFileName();
    log_file = fopen(log_file_name.c_str(), "a");
    if (log_file == nullptr) {
        perror("Failed to open log file");
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
        pthread_mutex_lock(&mutex_ivy_bus);
        current_mode_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 
    });

    droneManager->registerRelativeAngleCallback([](const RelativeAngleCollection& col){
        for (const RelativeAngle& ra : col) { 
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 
            if(verbose_tx){
                std::cout << "ID: " << ra.id
                        << std::fixed << std::setprecision(5)
                        << "Timestamp: " << current_timestamp << std::setprecision(0)
                        << " XAng: " << std::setw(3) << ra.x_angle * 180.0 / 3.14159
                        << " ZAng: " << std::setw(3) << ra.z_angle * 180.0 / 3.14159 
                        << " Intensity: " << std::setw(4) << ra.intensity 
                        << std::fixed << std::setprecision(2)
                        << " Width: " << std::setw(5) << ra.width << std::endl;
            }
            // if(send_values_on_ivy){
            //     //Send values over IVYBUS
            //     IvySendMsg("RELATIVE_BEACON_ANGLE %f %d %f %f %f %f", current_timestamp, ra.id, ra.x_angle, ra.z_angle, ra.intensity, ra.width);
            // }
        }
    });

    droneManager->registerFieldOfViewReportCallback([](const FieldOfViewReport& report) {

        // Process the FieldOfViewReport
        int j = 0;
        for (const auto& entry : report.seenBeacons) {
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec * 1e-9;

            fov_timestamp_beacon[j] = current_timestamp;
            fov_beacon_id[j] = entry.first;    // Beacon ID
            seen_count[j] = entry.second;      // Number of optical sensors that saw the beacon

            if (log_file != nullptr && produce_log_file) {
                pthread_mutex_lock(&mutex_logger);
                fprintf(log_file, "Msg_id: %d, Timestamp: %f, Beacon ID: %d, Seen Count: %d\n",
                        msg_id_fov, fov_timestamp_beacon[j], fov_beacon_id[j], seen_count[j]);
                fflush(log_file);
                pthread_mutex_unlock(&mutex_logger);
            }
            j++;
        }
    });

    droneManager->registerPoseRelativeBeaconCallback([](const PoseRelativeBeaconCollection& col){
        //zeros all the indexes for the ivy comm: 
        for (int i = 0; i < N_BEACON; i++){
            beacon_id[i] = 0;
        }
        //copy values on structure
        int j = 0;
        for (const PoseRelativeBeacon& b : col) {
            clock_gettime(CLOCK_BOOTTIME, &ts);
            double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9; 

            timestamp_beacon[j] = current_timestamp; 
            beacon_id[j] = (int) b.id; 
            x_body_pos_beacon[j] = (float) b.z; 
            y_body_pos_beacon[j] = (float) b.x;
            z_body_pos_beacon[j] = (float) b.y;

            if (log_file != nullptr && produce_log_file && beacon_id[j] != 0) {
                pthread_mutex_lock(&mutex_logger);
                fprintf(log_file, "Msg_id: %d, Timestamp: %f, Beacon ID: %d, x_body_pos_beacon: %f, y_body_pos_beacon: %f, z_body_pos_beacon: %f\n",
                        msg_id_rel_pos, timestamp_beacon[j], beacon_id[j], x_body_pos_beacon[j], y_body_pos_beacon[j], z_body_pos_beacon[j]);
                fflush(log_file);
                pthread_mutex_unlock(&mutex_logger);
            }

            j++;
        }    
        //Update flag:
        pthread_mutex_lock(&mutex_ivy_bus);
        beacon_pos_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 
    });

    droneManager->registerPose6DofCallback([](const Pose6Dof& sd){
        clock_gettime(CLOCK_BOOTTIME, &ts);
        double current_timestamp = ts.tv_sec + ts.tv_nsec*1e-9;
        
        float quat_array[4] = {(float) sd.qw, (float) sd.qx, (float) sd.qy, (float) sd.qz};
        float euler_angles[3];
        quaternion_to_euler(quat_array, euler_angles);

        static struct register_sixdof_packet my_sixdof_packet_local; 

        my_sixdof_packet_local.timestamp_position = current_timestamp; 
        my_sixdof_packet_local.x_body_pos = (float) -sd.x; 
        my_sixdof_packet_local.y_body_pos = (float) -sd.y; 
        my_sixdof_packet_local.z_body_pos = (float) -sd.z; 
        my_sixdof_packet_local.relative_phi_rad = (float) euler_angles[0];
        my_sixdof_packet_local.relative_theta_rad = (float) euler_angles[1];
        my_sixdof_packet_local.relative_psi_rad = (float) euler_angles[2];
        my_sixdof_packet_local.x_body_pos_var = (float) sd.var_x; 
        my_sixdof_packet_local.y_body_pos_var = (float) sd.var_y; 
        my_sixdof_packet_local.z_body_pos_var = (float) sd.var_y; 
        my_sixdof_packet_local.phi_rad_var = (float) -1;
        my_sixdof_packet_local.theta_rad_var = (float) -1;
        my_sixdof_packet_local.psi_rad_var = (float) -1;

        pthread_mutex_lock(&mutex_ivy_bus);
        memcpy(&my_sixdof_packet, &my_sixdof_packet_local, sizeof(struct register_sixdof_packet));
        sixdof_msg_available = 1;
        pthread_mutex_unlock(&mutex_ivy_bus); 
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