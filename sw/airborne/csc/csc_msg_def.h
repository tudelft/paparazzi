#ifndef CSC_MSG_DEF_H
#define CSC_MSG_DEF_H

#define CSC_SERVO_CMD_ID    0
#define CSC_MOTOR_CMD_ID    1
#define CSC_MOTOR_STATUS_ID 2
#define CSC_BOARD_STATUS_ID 3
#define CSC_BOARD_ADCVOLTS_ID 4

/* Received from the autopilot */
struct CscServoCmd {
  uint16_t servos[4];
} __attribute__((packed));

/* Send and Received between autopilot and csc */
struct CscMotorMsg {
  uint8_t  cmd_id;
  uint16_t arg1;
  uint16_t arg2;
} __attribute__((packed));

struct CscStatusMsg {
  uint32_t loop_count;
  uint32_t msg_count;
} __attribute__((packed));

struct CscADCMsg {
  float ADCVolts1;
  float ADCVolts2;
} __attribute__((packed));

#endif
