#ifndef TEST_SDP3X_ABI_H
#define TEST_SDP3X_ABI_H

#include <stdint.h>

#define SDP3X_SENDER_ID 41U
#define AIRSPEED_SDP3X_ID 3U

void test_abi_baro_diff(uint8_t sender_id, float pressure);
void test_abi_temperature(uint8_t sender_id, float temperature);
void test_abi_airspeed(uint8_t sender_id, float airspeed);

#define AbiSendMsgBARO_DIFF(sender_id, pressure) test_abi_baro_diff((sender_id), (pressure))
#define AbiSendMsgTEMPERATURE(sender_id, temperature) test_abi_temperature((sender_id), (temperature))
#define AbiSendMsgAIRSPEED(sender_id, airspeed) test_abi_airspeed((sender_id), (airspeed))

#endif
