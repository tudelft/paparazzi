/*
 * Copyright (C) 2017 Steven van der Helm, C. DW 
 * Copyright (C) 2026 OpenUAS
 *
 * This file is prt of paparazzi
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
 * @file "modules/uwb_swarm.h"
 * @author Steven van der Helm, C. DW , OpenUAS
 *
  * This module is for communication between drones in a swarm using UWB. 
  * It is designed to be used together with an intermediate MCU such as an Arduino Pro Mini icw a Decawave DWM1000 UWB module, 
  * This device which receives and sends UWB messages to other drones in the swarm, and communicates with the flight controller.
  * The code for the intermediate MCU can be found at 
  * http://github.com/tudelft/uwb_swarm_extender
  * The main idea is that the flight controller sends its state data (distance, velocity, relative position, acceleration, yaw rate) to the intermediate MCU over the bus, which then sends this data over UWB to the other drones in the swarm. 
  * The intermediate MCU also receives UWB messages FROM all the other drones, which contain their state data, and sends this data over bus to the flight controller, which can then use this data for various swarming behavior solutions
 */

//TODO: nodeAddress mapped to AC_ID, better even module dynamic config if connected to FC
//Even a dynamic update of NODES in the network would be good, but for now we can just assume a fixed number of nodes and fixed AC_IDs for simplicity

#include "uwb_swarm.h"
#include "modules/datalink/downlink.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"

#include "modules/datalink/telemetry.h"

#define UWB_SWARM_SERIAL_PORT (&((UWB_SWARM_UART).device))
struct link_device *external_device = UWB_SWARM_SERIAL_PORT;

#define UWB_SWARM_I2C_PORT i2c2  //

// Some meta data for on the bus communication
#define UWB_SWARM_COMM_MAX_MESSAGE 20
#define UWB_SWARM_COMM_END_MARKER 255
#define UWB_SWARM_COMM_SPECIAL_BYTE 253
#define UWB_SWARM_COMM_START_MARKER 254
#define UWB_SWARM_COMM_NODE_STATE_SIZE 7

#define UWB_SWARM_COMM_NUM_NODES 3 // How many total of devices we can maximum expect are in the network
#define UWB_SWARM_COMM_DIST_NUM_NODES UWB_SWARM_COMM_NUM_NODES-1  // How many distant nodes are in the network (one less than the total number of nodes)

// On the bus message types
#define UWB_SWARM_COMM_RANGE 0 // Distance to other drone
#define UWB_SWARM_COMM_VX 1    // Velocity in X direction
#define UWB_SWARM_COMM_VY 2    // Velocity in Y direction
#define UWB_SWARM_COMM_Z 3     // Altitude
#define UWB_SWARM_COMM_AX 4    // Acceleration in X direction
#define UWB_SWARM_COMM_AY 5    // Acceleration in Y direction
#define UWB_SWARM_COMM_YAWR 6  // Yaw rate

struct nodeState {
  uint8_t nodeAddress;
  float r;
  float vx;
  float vy;
  float z;
  float ax;
  float ay;
  float yawr;
  bool state_updated[UWB_SWARM_COMM_NODE_STATE_SIZE];
};

static struct nodeState states[UWB_SWARM_COMM_DIST_NUM_NODES];

/**
 * Function that is called when over the serial a new state value from a remote node is received
 */
static void handleNewStateValue(uint8_t nodeIndex, uint8_t msg_type, float value)
{
  struct nodeState *node = &states[nodeIndex];
  switch (msg_type) {
    case UWB_SWARM_COMM_RANGE :
      node->r = value;
      node->state_updated[UWB_SWARM_COMM_RANGE] = true;
      break;
    case UWB_SWARM_COMM_VX :
      node->vx = value;
      node->state_updated[UWB_SWARM_COMM_VX] = true;
      break;
    case UWB_SWARM_COMM_VY :
      node->vy = value;
      node->state_updated[UWB_SWARM_COMM_VY] = true;
      break;
    case UWB_SWARM_COMM_Z :
      node->z = value;
      node->state_updated[UWB_SWARM_COMM_Z] = true;
      break;
    case UWB_SWARM_COMM_AX :
      node->ax = value;
      node->state_updated[UWB_SWARM_COMM_AX] = true;
      break;
    case UWB_SWARM_COMM_AY :
      node->ay = value;
      node->state_updated[UWB_SWARM_COMM_AY] = true;
      break;
    case UWB_SWARM_COMM_YAWR :
      node->yawr = value;
      node->state_updated[UWB_SWARM_COMM_YAWR] = true;
      break;
  }
}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
static void decodeHighBytes(uint8_t bytes_received, uint8_t *received_message)
{
  static uint8_t receive_buffer[4];
  float tempfloat;
  uint8_t data_received_count = 0;
  uint8_t this_address = received_message[1];
  uint8_t msg_from = received_message[2];
  uint8_t msg_type = received_message[3];
  uint8_t nodeIndex = msg_from - 1 - (uint8_t)(this_address < msg_from);
  for (uint8_t i = 4; i < bytes_received - 1; i++) {
    // Skip the begin marker (0), this address (1), remote address (2), message type (3), and end marker (bytes_received-1)
    uint8_t var_byte = received_message[i];
    if (var_byte == UWB_SWARM_COMM_SPECIAL_BYTE) {
      i++;
      var_byte = var_byte + received_message[i];
    }
    if (data_received_count < 4) {
      receive_buffer[data_received_count] = var_byte;
    }
    data_received_count++;
  }
  if (data_received_count == 4) {
    // Move memory from integer buffer to float variable
    memcpy(&tempfloat, &receive_buffer, 4);
    
    // Ensure the nodeAddress matches the sender we received this float from
    states[nodeIndex].nodeAddress = msg_from;
    
    // Set the variable to the appropriate type and store it in state
    handleNewStateValue(nodeIndex, msg_type, tempfloat);
  }
}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
static void encodeHighBytes(uint8_t *send_data, uint8_t msg_size, uint8_t *data_send_buffer, uint8_t *data_total_send)
{
  uint8_t data_send_count = msg_size;
  *data_total_send = 0;
  for (uint8_t i = 0; i < data_send_count; i++) {
    if (send_data[i] >= UWB_SWARM_COMM_SPECIAL_BYTE) {
      data_send_buffer[*data_total_send] = UWB_SWARM_COMM_SPECIAL_BYTE;
      (*data_total_send)++;
      data_send_buffer[*data_total_send] = send_data[i] - UWB_SWARM_COMM_SPECIAL_BYTE;
    } else {
      data_send_buffer[*data_total_send] = send_data[i];
    }
    (*data_total_send)++;
  }
}

/**
 * Function that will send a float over the bus. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
static void sendFloat(uint8_t msg_type, float data)
{
  static uint8_t data_send_buffer[UWB_SWARM_COMM_MAX_MESSAGE];
  static uint8_t data_total_send = 0;

  // Make bytes of the float
  uint8_t floatbyte[4];
  memcpy(floatbyte, &data, 4);
  encodeHighBytes(floatbyte, 4, data_send_buffer, &data_total_send);

  UWB_SWARM_SERIAL_PORT->put_byte(UWB_SWARM_SERIAL_PORT->periph, 0, UWB_SWARM_COMM_START_MARKER);
  UWB_SWARM_SERIAL_PORT->put_byte(UWB_SWARM_SERIAL_PORT->periph, 0, msg_type);

  for (uint8_t i = 0; i < data_total_send; i++) {
    UWB_SWARM_SERIAL_PORT->put_byte(UWB_SWARM_SERIAL_PORT->periph, 0, data_send_buffer[i]);
  }

  UWB_SWARM_SERIAL_PORT->put_byte(UWB_SWARM_SERIAL_PORT->periph, 0, UWB_SWARM_COMM_END_MARKER);
}

/**
 * Helper function that sets the boolean that tells whether a remote drone has a new state update to false.
 */
static void setNodeStatesFalse(uint8_t index)
{
  for (uint8_t j = 0; j < UWB_SWARM_COMM_NODE_STATE_SIZE; j++) {
    states[index].state_updated[j] = false;
  }
}

/**
 * This function checks if all the states of all the distant nodes have at least once been updated.
 * If all the states are updated, then do something with it! AKA CALLBACK TO MARIO
 */
static void checkStatesUpdated(void)
{
  bool checkbool;
  for (uint8_t i = 0; i < UWB_SWARM_COMM_DIST_NUM_NODES; i++) {
    checkbool = true;
    for (uint8_t j = 0; j < UWB_SWARM_COMM_NODE_STATE_SIZE; j++) {
      checkbool = checkbool && states[i].state_updated[j];
    }
    if (checkbool) {
      AbiSendMsgUWB_COMMUNICATION(UWB_COMM_ID, i, states[i].r, states[i].vx, states[i].vy, states[i].z, states[i].ax, states[i].ay, states[i].yawr);
      setNodeStatesFalse(i);
    }
  }
}

/**
 * Function for receiving datastream from the bus, regardless of being I2C, UART, CAN.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in received_message, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */

 //FIXME: this function is currently only implemented for UART, but it should be implemented for I2C as well, and the periodic function should be adapted to call the appropriate getBusData function based on the communication type used.
static void getBusData(uint8_t *bytes_received)
{
  static bool in_progress = false;
  static uint8_t var_byte;
  static uint8_t received_message[UWB_SWARM_COMM_MAX_MESSAGE];

  while (external_device->char_available(external_device->periph)) {
    var_byte = UWB_SWARM_SERIAL_PORT->get_byte(UWB_SWARM_SERIAL_PORT->periph);

    if (var_byte == UWB_SWARM_COMM_START_MARKER) {
      (*bytes_received) = 0;
      in_progress = true;
    }

    if (in_progress) {
      if ((*bytes_received) < UWB_SWARM_COMM_MAX_MESSAGE - 1) {
        received_message[*bytes_received] = var_byte;
        (*bytes_received)++;
      } else {
        in_progress = false;
      }
    }

    if (var_byte == UWB_SWARM_COMM_END_MARKER) {
      in_progress = false;
      decodeHighBytes(*bytes_received, received_message);
    }
  }
}

/**
 * Initialization function that sets all the states of the distant nodes to false, meaning that no new state update has been received yet.
 * And sends also the AC_ID so a table can be made on the Arduino side to match the received UWB messages to the correct drone in the swarm based on AC_ID
 */
void uwb_swarm_init(void)
{
  // Set all nodes to false
  for (uint8_t i = 0; i < UWB_SWARM_COMM_DIST_NUM_NODES; i++) {
    setNodeStatesFalse(i);
  }

  //Send AC_ID so a table can be made on the Arduino side to match the received UWB messages to the correct drone in the swarm based on AC_ID. Since we can only send floats, we encode the AC_ID as a float by multiplying it with 0x01010101, which means that when decoded back into bytes, all 4 bytes of the float will have the value of AC_ID, which makes it easy to decode back into the original AC_ID on the Arduino side.
  //The uncommon scenrio that AC_ID is larger than 253 (the special byte value) is not handled, but in that case the AC_ID can simply be set to 253 on the Arduino side as well, since it is only used for matching the received UWB messages to the correct drone in the swarm, and it does not matter if multiple drones have the same AC_ID as long as they are different from the AC_ID of the drone itself.
  //also sending the AC_ID as a float with all bytes the same value makes it easy to identify the messages from this drone on the Arduino side, since they will have a unique value that is different from the messages received from the other drones in the swarm, which can be used for debugging and testing purposes.
/*
  uint8_t v = AC_ID;
  union { uint32_t i; float f; } u = { v * 0x01010101 };

  sendFloat(UWB_SWARM_COMM_RANGE, u.f);
  sendFloat(UWB_SWARM_COMM_VX, u.f);
  sendFloat(UWB_SWARM_COMM_VY, u.f);
  sendFloat(UWB_SWARM_COMM_Z, u.f);
  sendFloat(UWB_SWARM_COMM_AX, u.f);
  sendFloat(UWB_SWARM_COMM_AY, u.f);
  sendFloat(UWB_SWARM_COMM_YAWR, u.f);
*/
}

/**
 * This function periodically sends own state data over the bus, which in turn is received by the MiniTag PCB MCU, 
 * which then sends it over UWB to the other drones in the swarm. 
 */
void uwb_swarm_periodic(void)
{
  // TODO: Right now floats are sent individually, but it would be nice to send all at once (requires integrating with UWB/Arduino side as well).
  sendFloat(UWB_SWARM_COMM_VX, stateGetSpeedEnu_f()->y);
  sendFloat(UWB_SWARM_COMM_VY, stateGetSpeedEnu_f()->x);
  sendFloat(UWB_SWARM_COMM_Z, stateGetPositionEnu_f()->z);
  sendFloat(UWB_SWARM_COMM_AX, stateGetAccelNed_f()->x);
  sendFloat(UWB_SWARM_COMM_AY, stateGetAccelNed_f()->y);
  sendFloat(UWB_SWARM_COMM_YAWR, stateGetBodyRates_f()->r);

}

/**
 * Event function currently checks for serial data and whether an update of states is available for a distant drone.
 * If these cases are true, then actions are taken.
 */
void uwb_swarm_event(void)
{
//#if !SITL
  static uint8_t bytes_received;
  getBusData(&bytes_received);
  checkStatesUpdated();
//#endif
}

void uwb_swarm_report(void)
{
/* Add this to messages.xml data for downlink message: */
/*
    <message name="UWB_SWARM" id="51">
      <description>Relative localization data from current aircraft to be picked up by other Aircrafts in a swarm</description>
      <field name="id_tracked"    type="uint8" unit="">ID of the MAV this data refers to</field>
      <field name="r_tracked"     type="float" unit="m">Distance of other MAV</field>
      <field name="vx_tracked"    type="float" unit="m/s">X speed of other MAV</field>
      <field name="vy_tracked"    type="float" unit="m/s">Y speed of other MAV</field>
      <field name="z_tracked"     type="float" unit="m">Height position of other MAV</field>
      <field name="ax_tracked"    type="float" unit="m/s^2">X acceleration of other MAV</field>
      <field name="ay_tracked"    type="float" unit="m/s^2">Y acceleration of other MAV</field>
      <field name="yawr_tracked"  type="float" unit="rad/s">Yaw rate of other MAV</field>
    </message>
*/

  static uint8_t report_node = 0;

  if (states[report_node].nodeAddress != 0) {
      DOWNLINK_SEND_UWB_SWARM(DefaultChannel, DefaultDevice,
                              &states[report_node].nodeAddress, 
                              &states[report_node].r,
                              &states[report_node].vx, 
                              &states[report_node].vy,
                              &states[report_node].z, 
                              &states[report_node].ax, 
                              &states[report_node].ay,
                              &states[report_node].yawr); 
  }
  
  report_node++;
  if (report_node >= UWB_SWARM_COMM_DIST_NUM_NODES) {
      report_node = 0;
  }
}
