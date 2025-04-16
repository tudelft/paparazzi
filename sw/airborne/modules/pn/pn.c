#include <stdio.h>
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/ins/ins_int.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"

void pn_init(void);
void pn_run(void);
void pn_start(void);
void pn_stop(void);
float eucld(struct FloatVect3 v);
static void saturate(struct FloatVect3 *vector, float max_val);
static float time_s = 0.0f;
uint8_t flag = 1;
const float dt = 1.0f / 100.0f; // NOTE THIS MUST MATCH THE IMU DATASTREAM??!?

void pn_init(void)
{
    printf("[pn] pn_init() called\n");
}

void pn_run(void)
{
    time_s += dt;

    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        printf("[pn] Not in GUIDED mode, skipping pursuit.\n");
        return;
    }

    // === Configurable parameters ===
    float lambda = 50.0f;
    float pp_weight = 0.03f;
    float max_accel = 5.0f;
    float epsilon = 1e-3f;
    float heading_sp = 0.0f;

    float radius = 2.0f;
    float angular_speed = 0.5f;  // rad/s

    struct FloatVect3 pos_target = {
        .x = radius * sinf(angular_speed * time_s),
        .y = radius * cosf(angular_speed * time_s),
        .z = -1.0f
    };

    // === Get current position and velocity ===
    struct NedCoor_f *pos_now = stateGetPositionNed_f();
    struct NedCoor_f *vel_now = stateGetSpeedNed_f();

    printf("Pos Drone: [%.2f, %.2f, %.2f]\n", pos_now->x, pos_now->y, pos_now->z);


    struct FloatVect3 r = {
        .x = pos_target.x - pos_now->x,
        .y = pos_target.y - pos_now->y,
        .z = pos_target.z - pos_now->z
    };

    struct FloatVect3 vel_target = {
        .x = radius * angular_speed * sinf(angular_speed * time_s),
        .y = -radius * angular_speed * cosf(angular_speed * time_s),
        .z = 0.0f
    };
    
    struct FloatVect3 r_dot = {
        .x = vel_target.x - vel_now->x,
        .y = vel_target.y - vel_now->y,
        .z = vel_target.z - vel_now->z
    };

    float r_norm = eucld(r);
    float r_dot_norm = eucld(r_dot);
    float t_go = r_norm / (r_dot_norm + epsilon);

    printf("[pn] Time: %.2fs | Target Pos: [%.2f, %.2f, %.2f]\n", time_s, pos_target.x, pos_target.y, pos_target.z);
    // printf("[pn] Relative r = [%.2f, %.2f, %.2f], norm = %.2f\n", r.x, r.y, r.z, r_norm);
    // printf("[pn] Relative r_dot = [%.2f, %.2f, %.2f], norm = %.2f\n", r_dot.x, r_dot.y, r_dot.z, r_dot_norm);
    // printf("[pn] Time-to-go t_go = %.3f s\n", t_go);

    struct FloatVect3 term1 = {
        .x = (r.x + r_dot.x * t_go) / (t_go * t_go),
        .y = (r.y + r_dot.y * t_go) / (t_go * t_go),
        .z = (r.z + r_dot.z * t_go) / (t_go * t_go)
    };

    struct FloatVect3 acc_cmd = {
        .x = lambda * ((1 - pp_weight) * term1.x + pp_weight * r.x),
        .y = lambda * ((1 - pp_weight) * term1.y + pp_weight * r.y),
        .z = lambda * ((1 - pp_weight) * term1.z + pp_weight * r.z)
    };

    printf("[pn] Unconstrained FRPN accel = [%.2f, %.2f, %.2f]\n", acc_cmd.x, acc_cmd.y, acc_cmd.z);
    saturate(&acc_cmd, max_accel);
    printf("[pn] Saturated accel = [%.2f, %.2f, %.2f]\n", acc_cmd.x, acc_cmd.y, acc_cmd.z);

    // guidance_h_set_acc(acc_cmd.x, acc_cmd.y); // horizontal

    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &acc_cmd);

    // struct StabilizationSetpoint sp = guidance_indi_run(&acc_cmd, heading_sp);

    // struct Int32Quat q_des = stab_sp_to_quat_i(&sp);
    // bool in_flight = autopilot_in_flight();
    // stabilization_indi_attitude_run(q_des, in_flight);

    
    printf("Distance: %.3f m\n", r_norm);

}


void pn_start(void)
{
    printf("[pn] pn_start() called\n");
}

void pn_stop(void)
{
    printf("[pn] pn_stop() called\n");
}

float eucld(struct FloatVect3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Saturate the vector by its magnitude while preserving its direction.
static void saturate(struct FloatVect3 *vector, float max_val)
{
    float mag = eucld(*vector);
    if (mag > max_val) {
        float scale = max_val / mag;
        vector->x *= scale;
        vector->y *= scale;
        vector->z *= scale;
    }
}