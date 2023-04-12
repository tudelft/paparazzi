/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/cfd_wind_data_importer/cfd_wind_data_importer.c"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Sending IVY request to import CFD wind data periodically
 */

#include "modules/cfd_wind_data_importer/cfd_wind_data_importer.h"
//#include "nps_ivy.h"

#include <glib.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "nps_fdm.h"
#include "nps_atmosphere.h"
#include "state.h"

#include "modules/datalink/downlink.h"
#include "modules/datalink/telemetry.h"
//#include "subsystems/abi.h"

#include "math/pprz_geodetic_float.h"

extern int wind_speed = 12;

static void cfd_wind_importer_send_position(void)
{
    struct NpsFdm fdm_ivy;
    memcpy(&fdm_ivy, &fdm, sizeof(struct NpsFdm));
    float _pos_x = fdm_ivy.ltpprz_pos.x;
    float _pos_y = fdm_ivy.ltpprz_pos.y;
    float _pos_z = fdm_ivy.ltpprz_pos.z;
    uint8_t ac_id = AC_ID;
//    printf("%f %f %f\n", _pos_x, _pos_y, _pos_z);
    DOWNLINK_SEND_LTP_POSITION(DefaultChannel, DefaultDevice, &ac_id,
                               &_pos_x, &_pos_y, &_pos_z, &wind_speed);
//    pprz_msg_send_LTP_POSITION(DefaultChannel, DefaultDevice, AC_ID,
//                               (&fdm_ivy.ltpprz_pos.x),
//                               (&fdm_ivy.ltpprz_pos.y),
//                               (&fdm_ivy.ltpprz_pos.z));
}

// Variables that are send through IVY
//static void send_ltp_position(struct transport_tx *trans, struct link_device *dev){
//    struct NpsFdm fdm_ivy;
//    memcpy(&fdm_ivy, &fdm, sizeof(struct NpsFdm));
//    pprz_msg_send_LTP_POSITION(trans, dev, AC_ID,
//                               &fdm_ivy.ltpprz_pos.x,
//                               &fdm_ivy.ltpprz_pos.y,
//                               &fdm_ivy.ltpprz_pos.z);
//}

//static void ivy_send_CFD_WIND_REQ(void)
//{
//    // First unbind from previous request if needed
////    if (ivyPtr != NULL) {
////        IvyUnbindMsg(ivyPtr);
////        ivyPtr = NULL;
////    }
//
//    int pid = (int)getpid();
//
//    // Bind to the reply;
////    ivyPtr = IvyBindMsg(on_CFD_WIND, NULL, "^%d_%d (\\S*) CFD_WIND (\\S*) (\\S*) (\\S*)", pid, seq);
//
//    // Send actual request
//    struct NpsFdm fdm_ivy;
//    memcpy(&fdm_ivy, &fdm, sizeof(struct NpsFdm));
//
//    IvySendMsg("CFD %d_%d CFD_WIND_REQ %f %f %f %f %f %f",
//               pid, seq,
//               DegOfRad(fdm_ivy.lla_pos_pprz.lat),
//               DegOfRad(fdm_ivy.lla_pos_pprz.lon),
//               (fdm_ivy.hmsl),
//               (fdm_ivy.ltpprz_pos.x),
//               (fdm_ivy.ltpprz_pos.y),
//               (fdm_ivy.ltpprz_pos.z));
//    seq++;
//
////    return TRUE;
//}

void cfd_wind_importer_parse_wind_msg(uint8_t *buf)
{
    // wind speed in m/s
    struct FloatVect3 wind;
    wind.x = DL_CFD_WIND_DATA_wind_east(buf); //east
    wind.y = DL_CFD_WIND_DATA_wind_north(buf); //north
    wind.z = DL_CFD_WIND_DATA_wind_up(buf); //up

//    printf("%f %f %f\n", wind.x, wind.y, wind.z);

    /* set wind speed in NED */
    nps_atmosphere_set_wind_ned(wind.y, wind.x, -wind.z);
}

void cfd_wind_importer_move_waypoint_msg_cb(uint8_t *buf)
{
    // receive ltp position
    int wp_id;
    struct NedCoor_f ltp_pos;

    wp_id = DL_MOVE_WAYPOINT_LTP_wp_id(buf);
    ltp_pos.x = DL_MOVE_WAYPOINT_LTP_ltp_x(buf);
    ltp_pos.y = DL_MOVE_WAYPOINT_LTP_ltp_y(buf);
    ltp_pos.z = DL_MOVE_WAYPOINT_LTP_ltp_z(buf);

    printf("%d: %f, %f, %f, move wp\n", wp_id, ltp_pos.x, ltp_pos.y, ltp_pos.z);

    struct UtmCoor_f pos_utm;

    if(!state.utm_initialized_f){
        printf("utm origin not initialized\n");
        return;
    }
    UTM_OF_NED_ADD(pos_utm, ltp_pos, state.utm_origin_f);
    printf("moved wp %d utm: %f, %f, %f\n", wp_id, pos_utm.east, pos_utm.north, pos_utm.alt);

    nav_move_waypoint(wp_id, pos_utm.east, pos_utm.north, pos_utm.alt);
}


//void* ivy_main_loop(void* data __attribute__((unused)));
//
//int find_launch_index(void);
//
//
//void* ivy_main_loop(void* data __attribute__((unused)))
//{
//    IvyMainLoop();
//
//    return NULL;
//}
//
//void nps_ivy_init(char *ivy_bus)
//{
//    const char *agent_name = AIRFRAME_NAME"_NPS";
//    const char *ready_msg = AIRFRAME_NAME"_NPS Ready";
//    IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);
//
//    // bind on a general WORLD_ENV (not a reply to request)
//    IvyBindMsg(on_WORLD_ENV, NULL, "^(\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
//
//    // to be able to change datalink_enabled setting back on
//    IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
//
//#ifdef __APPLE__
//    const char *default_ivy_bus = "224.255.255.255";
//#else
//    const char *default_ivy_bus = "127.255.255.255";
//#endif
//    if (ivy_bus == NULL) {
//        IvyStart(default_ivy_bus);
//    } else {
//        IvyStart(ivy_bus);
//    }
//
//    nps_ivy_send_world_env = false;
//
//    ap_launch_index = find_launch_index();
//
//    // Launch separate thread with IvyMainLoop()
//    pthread_create(&th_ivy_main, NULL, ivy_main_loop, NULL);
//
//}
//
///*
// * Parse WORLD_ENV message from gaia.
// *
// */
//static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
//                         void *user_data __attribute__((unused)),
//                         int argc __attribute__((unused)), char *argv[])
//{
//    // wind speed in m/s
//    struct FloatVect3 wind;
//    wind.x = atof(argv[1]); //east
//    wind.y = atof(argv[2]); //north
//    wind.z = atof(argv[3]); //up
//
//    /* set wind speed in NED */
//    nps_atmosphere_set_wind_ned(wind.y, wind.x, -wind.z);
//
//    /* not used so far */
//    //float ir_contrast = atof(argv[4]);
//
//    /* set new time factor */
//    nps_set_time_factor(atof(argv[5]));
//
//#if USE_GPS
//    // directly set gps fix in subsystems/gps/gps_sim_nps.h
//  gps_has_fix = atoi(argv[6]); // gps_availability
//#endif
//}
//
///*
// * Send a WORLD_ENV_REQ message
// */
//
//
//void nps_ivy_send_WORLD_ENV_REQ(void)
//{
//    // First unbind from previous request if needed
//    if (ivyPtr != NULL) {
//        IvyUnbindMsg(ivyPtr);
//        ivyPtr = NULL;
//    }
//
//    int pid = (int)getpid();
//
//    // Bind to the reply
//    ivyPtr = IvyBindMsg(on_WORLD_ENV, NULL, "^%d_%d (\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", pid, seq);
//
//    // Send actual request
//    struct NpsFdm fdm_ivy;
//    memcpy(&fdm_ivy, &fdm, sizeof(struct NpsFdm));
//
//    IvySendMsg("nps %d_%d WORLD_ENV_REQ %f %f %f %f %f %f",
//               pid, seq,
//               DegOfRad(fdm_ivy.lla_pos_pprz.lat),
//               DegOfRad(fdm_ivy.lla_pos_pprz.lon),
//               (fdm_ivy.hmsl),
//               (fdm_ivy.ltpprz_pos.x),
//               (fdm_ivy.ltpprz_pos.y),
//               (fdm_ivy.ltpprz_pos.z));
//    seq++;
//
//    nps_ivy_send_world_env = false;
//}


//static void init_ivy_cfd_wind_data_importer(void) {
//
//    IvyInit ("CFD_WIND_DATA_IMPORTER", "CFD_WIND_DATA_IMPORTER READY", NULL, NULL, NULL, NULL);
//    IvyStart("127.255.255.255");
//    IvyBindMsg(on_CFD_WIND, NULL, "^(\\S*) CFD_WIND (\\S*) (\\S*) (\\S*)");
//    init_ivy = 0;
//}



//gboolean cfd_wind_timeout_cb(gpointer data){
//    return TRUE;
//}

//void init_cfd_wind_data_importer_ivy(void) {
//    GMainLoop *ml = g_main_loop_new(NULL, FALSE);
//
//    IvyInit("CFD_WIND_IMPORTER", "CFD_WIND_IMPORTER READY", NULL, NULL, NULL, NULL);
//    IvyStart("127.255.255.255");
//
//    g_timeout_add(100, ivy_send_CFD_WIND_REQ, NULL);
//
//    IvyBindMsg(on_CFD_WIND, NULL, "^(\\S*) CFD_WIND (\\S*) (\\S*) (\\S*)");
//
//    g_main_loop_run(ml);
//}

void init_cfd_wind_data_importer(void)
{
  // your init code here
//    IvyBindMsg(on_CFD_WIND, NULL, "^(\\S*) CFD_WIND (\\S*) (\\S*) (\\S*)");
//    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LTP_POSITION, send_ltp_position);
}

void cfd_wind_data_importer_periodic(void)
{
//    if (init_ivy){
//        init_ivy_cfd_wind_data_importer();
//    }
  // your periodic code here.
//    ivy_send_CFD_WIND_REQ();
    cfd_wind_importer_send_position();
//    printf("init: %d, %d, %d\n", state.ned_initialized_i, state.ned_initialized_f, state.utm_initialized_f);

}

