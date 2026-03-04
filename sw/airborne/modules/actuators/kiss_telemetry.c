/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>, 2026 OpenUAS
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
/**
 * @file "modules/actuators/kiss_telemetry.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * @brief Retrieve live telemetry data from a KISS or KISS compatible ESCs via the ESC datapin output a serial byte stream.
 */

#include "modules/actuators/kiss_telemetry.h"
#include "modules/actuators/actuators_pwm.h"
#include "mcu_periph/uart.h"
#include "modules/datalink/telemetry.h"
#include "modules/energy/electrical.h" // For getting battery voltage, a value not from ESC

#ifndef KISS_TELEMETRY_MOTOR_POLES
#define KISS_TELEMETRY_MOTOR_POLES 14
#endif

//FIXME:
// When using multiple ESC's then enhance, fix and validate current code.
// #define ACTUATORS_KISS_NB ACTUATORS_NB
// For now only one(1) KISS ESC with telemetry is supported
#ifndef ACTUATORS_KISS_NB
#define ACTUATORS_KISS_NB 1
#endif

#ifndef ACTUATORS_KISS_SERVO_IDX
#define ACTUATORS_KISS_SERVO_IDX 0
#endif

#define KISS_TELEMETRY_REQ_PULSE_US 840

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static struct kiss_telemetry_t kiss_telemetry;

void kiss_telemetry_init(void) {
    kiss_telemetry.dev = &((KISS_TELEMETRY_PORT).device);
    kiss_telemetry.servo_idx = ACTUATORS_KISS_SERVO_IDX;//FIXME: This is only for one ESC
    kiss_telemetry.buf_idx = 0;
}

void kiss_telemetry_periodic(void) {
    uint32_t __attribute__ ((unused)) cnt;

    /* Check if we are allowed to send the pulse for a telemetry request */
#if PWM_USE_TIM1
    cnt = timer_get_counter(TIM1);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM2
    cnt = timer_get_counter(TIM2);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM3
    cnt = timer_get_counter(TIM3);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM4
    cnt = timer_get_counter(TIM4);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM5
    cnt = timer_get_counter(TIM5);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM6
    cnt = timer_get_counter(TIM6);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM7
    cnt = timer_get_counter(TIM7);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM8
    cnt = timer_get_counter(TIM8);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM9
    cnt = timer_get_counter(TIM9);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM10
    cnt = timer_get_counter(TIM10);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM11
    cnt = timer_get_counter(TIM11);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM12
    cnt = timer_get_counter(TIM12);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM13
    cnt = timer_get_counter(TIM13);
    if(cnt <= 2400) {
        return;
    }
#endif
#if PWM_USE_TIM14
    cnt = timer_get_counter(TIM14);
    if(cnt <= 2400) {
        return;
    }
#endif

    /* Send a pulse to request telemetry data */
    ActuatorPwmSet(kiss_telemetry.servo_idx, KISS_TELEMETRY_REQ_PULSE_US);
    ActuatorsPwmCommit();

    //struct act_feedback_t feedback[ACTUATORS_KISS_NB] = { 0 };

    /* Move to next servo for next time */
    kiss_telemetry.servo_idx = (kiss_telemetry.servo_idx + 1) % ACTUATORS_KISS_NB;
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    return (crc_u);
}

void kiss_telemetry_event(void) {
    if (kiss_telemetry.dev->char_available(kiss_telemetry.dev->periph)) {
        while (kiss_telemetry.dev->char_available(kiss_telemetry.dev->periph)) {

            uint8_t c = kiss_telemetry.dev->get_byte(kiss_telemetry.dev->periph);
            kiss_telemetry.buffer[kiss_telemetry.buf_idx] = c;

            /* Update CRC if needed */
            if(kiss_telemetry.buf_idx < (KISS_FRAME_LENGTH - 1)) kiss_telemetry.crc = update_crc8(c, kiss_telemetry.crc);

            kiss_telemetry.buf_idx++;

            /* We received a full telemetry message */
            if(kiss_telemetry.buf_idx >= KISS_FRAME_LENGTH) {

            //DEBUG ONLY
            float xa,xb,xc,xd,xe,xf,xg,xh,xi,xj;
            xa = (float)rand();
            xb = 11.0;//(float)kiss_telemetry.buffer[0];
            xc = 22.0;//(float)((kiss_telemetry.buffer[1] << 8) + kiss_telemetry.buffer[2]) / 100.0;
            xd = 33.0;//(float)((kiss_telemetry.buffer[3] << 8) + kiss_telemetry.buffer[4]) / 100.0;
            xe = 44.0;//(float)((kiss_telemetry.buffer[5] << 8) + kiss_telemetry.buffer[6]) / 1000.0;
            xf = 2400.0;//(float)((kiss_telemetry.buffer[7] << 8) + kiss_telemetry.buffer[8]) * 200 / KISS_TELEMETRY_MOTOR_POLES;
            xg = 66.0;//electrical.vsupply / 10.0;
            xh = 77.0;//current * motor_volts;
            xi = 88.0;//node_id;
            xj = 99.0;//temperature_dev;
            pprz_msg_send_ESC(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID, &xa,&xb,&xc,&xd,&xe,&xf,&xg,&xh,&xi,&xj);
            //DEBUG ONLY END


                /* If the CRC matches parse the message */
                if(kiss_telemetry.crc == kiss_telemetry.buffer[KISS_FRAME_LENGTH - 1]) {

//DOES not arrive here

                    /* From KISS telemetry stream */
                    float temperature = kiss_telemetry.buffer[0];
                    float motor_volts = ((kiss_telemetry.buffer[1] << 8) + kiss_telemetry.buffer[2]) / 100.0;
                    float current = ((kiss_telemetry.buffer[3] << 8) + kiss_telemetry.buffer[4]) / 100.0;
                    float consumption = ((kiss_telemetry.buffer[5] << 8) + kiss_telemetry.buffer[6]) / 1000.0;
                    // RPM in telemetry is need to be real RPM with a scale of 100, not electrical RPM, therefore value is 2*100 / motor_poles
                    float rpm = ((kiss_telemetry.buffer[7] << 8) + kiss_telemetry.buffer[8]) * 200 / KISS_TELEMETRY_MOTOR_POLES;

                    /* NOT comming from the KISS telemetry stream, but additional values needed to fill ESC message */
                    float bat_volts = electrical.vsupply / 10.0; // Supply voltage from battery
                    float power = current * motor_volts;// Electrical power
                    uint8_t node_id = 0;     // Unused since KISS ESCs do not send a node ID
                    float temperature_dev = 0; // Extra value from a thermistor on ADC port mounted in e.g. Motor

                    // pprz_msg_send_ESC(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID,
                    //                   &current, &bat_volts, &power, &rpm, &motor_volts, &consumption, &temperature, &temperature_dev, &node_id, &kiss_telemetry.servo_idx);
                }
                kiss_telemetry.buf_idx = 0;
                kiss_telemetry.crc = 0;
            }
        }
    }
    //FIXME: If you fancy it one can enhance all with feedback and Abi AbiSendMsgACT_FEEDBACK(ACT_FEEDBACK_ACTUATOR_ID, feedback, ACTUATORS_NB);
}

