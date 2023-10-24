/*
 * Copyright (C) 2023 Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
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

/** @file "firmwares/rotorcraft/oneloop/navigation_oneloop.c"
 * @author Tomaso De Ponti <t.m.l.deponti@tudelft.nl>
 * A collection of Navigation functions to test the oneloop controller
 */

#include "firmwares/rotorcraft/oneloop/navigation_oneloop.h"
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"


void init_nav_oneloop(void){

}

/**
 * Navigation speed controller function.
 *
 * This function calculates the control signal for the navigation speed controller based on the desired
 * speed (x_d_des) and the actual speed (x_d_actual) of a system. It updates the reference values
 * for velocity (x_d_ref), acceleration (x_2d_ref), jerk (x_3d_ref) and position (x_ref) using a proportional controller.
 *
 * @param dt          Time step between control updates.
 * @param x_ref       Pointer to the reference position.
 * @param x_d_ref     Pointer to the reference velocity.
 * @param x_2d_ref    Pointer to the reference acceleration.
 * @param x_3d_ref    Pointer to the reference jerk.
 * @param x_d_des     Desired velocity.
 * @param x_d_actual  Actual velocity.
 * @param x_2d_bound  Upper bound for the control acceleration.
 * @param k1_rm       Proportional control gain velocity.
 * @param k2_rm       Proportional control gain acceleration.
 */

void nav_speed_controller(float dt, float* x_ref, float* x_d_ref, float* x_2d_ref, float* x_3d_ref, float x_d_des, float x_d_actual, float x_2d_bound, float k1_rm, float k2_rm){
  float e_x_d      = k1_rm * (x_d_des- x_d_actual);
  float e_x_2d     = k2_rm * (e_x_d- *x_2d_ref);
  float x_3d_bound = (x_2d_bound - fabs(*x_2d_ref)) / dt;
  BoundAbs(e_x_2d, x_3d_bound);
  *x_3d_ref = e_x_2d;
  *x_2d_ref = (*x_2d_ref + dt * (*x_3d_ref));
  *x_d_ref  = (*x_d_ref  + dt * (*x_2d_ref));
  *x_ref    = (*x_ref    + dt * (*x_d_ref ));  
}
/**
 * @brief the position, velocity, acceleration, jerk, and lap count for a point moving along a straight oval path.
 *
 * @param s         The distance along the path.
 * @param r         The radius of the corners of the oval path.
 * @param l         The length of the straight sections of the oval path.
 * @param psi_i     The heading angle of the oval path.
 * @param v_route   The velocity magnitude along the path.
 * @param a_route   The acceleration magnitude along the path.
 * @param j_route   The jerk magnitude along the path.
 * @param p         Array of length 3 representing the position vector [x, y, z] of the point on the path.
 * @param v         Array of length 3 representing the velocity vector [Vx, Vy, Vz] of the point on the path.
 * @param a         Array of length 3 representing the acceleration vector [Ax, Ay, Az] of the point on the path.
 * @param j         Array of length 3 representing the jerk vector [Jx, Jy, Jz] of the point on the path.
 * @param lap       Pointer to a variable that stores the number of laps completed around the oval path.
 */

//void straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float* p, float* v, float* a, float* j, float* lap)
void straight_oval(float s, float r, float l, float psi_i, float v_route, float a_route, float j_route, float p[3], float pi[3], float v[3], float a[3], float j[3], float psi_vec[4], float* lap) {
    //printf("psi_oval = %f\n",psi_i);
    float cd = cosf(psi_i);
    float sd = sinf(psi_i);
    float head_vec[4] = {0.0, 0.0, 0.0, 0.0};
    float p0[3] = {0.0, 0.0, 0.0};
    //float pi[3] = {0.0,0.0,0.0};
    float p1[3] = {p0[0] + r, p0[1]-r, p0[2]};
    float p2[3] = {p1[0] + l, p0[1]-r, p0[2]};

    // Calculate inner vertices
    float p1i[3] = {p1[0], p1[1] + r, p1[2]};
    float p2i[3] = {p2[0], p2[1] + r, p2[2]};
    float p3i[3] = {p2[0], p2[1] - r, p2[2]};
    float p4i[3] = {p1[0], p1[1] - r, p1[2]};
    float l01 = 1.0; 
    float l12 = 1.0;
    float l34 = 1.0;
    
    float tmp_1[3] = {0.,0.,0.};
    float tmp_2[3] = {0.,0.,0.};
    float tmp_3[3] = {0.,0.,0.};
    float_vect_diff(tmp_1,p1i,p0,3);
    float_vect_diff(tmp_2,p2i,p1i,3);
    float_vect_diff(tmp_3,p4i,p3i,3);
    l01 = float_vect_norm(tmp_1,3);
    l12 = float_vect_norm(tmp_2,3);
    l34 = float_vect_norm(tmp_3,3);
    
    float u01[3] = {0.,0.,0.};  
    float u12[3] = {0.,0.,0.};
    float u34[3] = {0.,0.,0.};

    float_vect_sdiv(u01,tmp_1,l01,3);
    float_vect_sdiv(u12,tmp_2,l12,3);
    float_vect_sdiv(u34,tmp_3,l34,3);
    float runway = r;
    int8_t i;
    if (s < runway) {
      for (i = 0; i < 3; i++){
        p[i] = p0[i] + s * u01[i];
        v[i] = v_route * u01[i];
        a[i] = a_route * u01[i];
        j[i] = j_route * u01[i];
      }
    } else {
        s = s - runway;

        // Calculate length of route
        float L = 2 * M_PI * r + l12 + l34;
        float s1 = l12;
        float s2 = s1 + M_PI * r;
        float s3 = s2 + l34;
        float s4 = s3 + M_PI * r;
        float dist = fmodf(s, L);
        *lap = floorf(s / (1.05f * L));
        if (dist < s1) {
            float sector = dist;
              for (i = 0; i < 3; i++){
                p[i] = p1i[i] + sector * u12[i];
                v[i] = v_route * u12[i];
                a[i] = a_route * u12[i];
                j[i] = j_route * u12[i];
              }
        } else if (dist < s2) {
            float sector = dist - s1;
            float beta = sector / r;
            float sb = sinf(beta);
            float cb = cosf(beta);
            float v2 = v_route * v_route;
            float v3 = v2 * v_route;
            float r2 = r * r;
            head_vec[0] = beta;
            head_vec[1] = v_route / r;
            head_vec[2] = a_route / r;
            head_vec[3] = j_route / r; 

            p[0] = r * sb + p2[0];
            p[1] = r * cb + p2[1];
            p[2] = p2[2];
            v[0] =  cb * v_route;
            v[1] = -sb * v_route;
            v[2] = 0; 
            a[0] =  cb * a_route - sb * v2 /  r;
            a[1] = -sb * a_route - cb * v2 / r;
            a[2] = 0 ;
            j[0] = cb * j_route - cb * v3 / r2 - 3 * sb * a_route * v_route / r;
            j[1] = sb * v3 / r2 - sb * j_route - 3 * cb * a_route * v_route / r;
            j[2] = 0;
        } else if (dist < s3) {
            float sector = dist - s2;
               for (i = 0; i < 3; i++){
                p[i] = p3i[i] + sector * u34[i];
                v[i] = v_route * u34[i];
                a[i] = a_route * u34[i];
                j[i] = j_route * u34[i];
                head_vec[0] = M_PI;
              }           
        } else if (dist < s4) {
            float sector = dist - s3;
            float beta = M_PI + sector / r;
            float sb = sinf(beta);
            float cb = cosf(beta);
            float v2 = v_route * v_route;
            float v3 = v2 * v_route;
            float r2 = r * r;
            head_vec[0] = beta;
            head_vec[1] = v_route / r;
            head_vec[2] = a_route / r;
            head_vec[3] = j_route / r;

            p[0] = r * sb + p1[0];
            p[1] = r * cb + p1[1];
            p[2] = p1[2];
            v[0] =  cb * v_route;
            v[1] = -sb * v_route;
            v[2] = 0; 
            a[0] =  cb * a_route - sb * v2 / r;
            a[1] = -sb * a_route - cb * v2 / r;
            a[2] = 0 ;
            j[0] = cb * j_route - cb * v3 / r2 - 3 * sb * a_route * v_route / r;
            j[1] = sb * v3 / r2 - sb * j_route - 3 * cb * a_route * v_route / r;
            j[2] = 0;            
        }
    }

    float temp_p[3] = {
        cd * p[0] - sd * p[1],
        sd * p[0] + cd * p[1],
        p[2]
    };
    float temp_v[3] = {
        cd * v[0] - sd * v[1],
        sd * v[0] + cd * v[1],
        v[2]
    };
    float temp_a[3] = {
        cd * a[0] - sd * a[1],
        sd * a[0] + cd * a[1],
        a[2]
    };
    float temp_j[3] = {
        cd * j[0] - sd * j[1],
        sd * j[0] + cd * j[1],
        j[2]
    };
    for (i = 0; i < 3; i++){
      p[i] = temp_p[i] + pi[i];
      v[i] = temp_v[i];
      a[i] = temp_a[i];
      j[i] = temp_j[i];
    }
    psi_vec[0] = convert_angle(-head_vec[0] + psi_i);
    psi_vec[1] = -head_vec[1];
    psi_vec[2] = -head_vec[2];
    psi_vec[3] = -head_vec[3];
}

