#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <generated/airframe.h>
#include "modules/sensors/rangefinder_i2c.h"
#include "modules/core/abi.h"
#include "modules/datalink/telemetry.h"

#ifndef RANGEFINDER_I2C_SYNC_SEND
#define RANGEFINDER_I2C_SYNC_SEND 0
#endif

struct i2c_periph test_bus;
static bool accept_request = true;
static unsigned reads, writes, publications;
static float last_distance;
static unsigned debug_messages, telemetry_messages;
static uint16_t debug_raw;
static float debug_distance;
static test_telemetry_callback_t telemetry_callback;
uint32_t get_sys_time_usec(void) { return 1000; }

void register_periodic_telemetry(int periodic, int message, test_telemetry_callback_t callback)
{
  assert(periodic == DefaultPeriodic && message == PPRZ_MSG_ID_RANGEFINDER);
  assert(telemetry_callback == NULL);
  telemetry_callback = callback;
}

void pprz_msg_send_RANGEFINDER(struct transport_tx *trans, struct link_device *device, uint8_t aircraft,
                             const uint8_t *address, const uint16_t *raw, const float *distance)
{
  (void)trans;
  (void)device;
  assert(aircraft == AC_ID && *address == 0xA4 && *raw == 3000);
  assert(fabsf(*distance - 3.f) < 0.0001f);
  telemetry_messages++;
}

void test_rangefinder_debug_send(const uint8_t *address, const uint16_t *raw, const float *distance)
{
  assert(*address == 0xA4);
  debug_messages++;
  debug_raw = *raw;
  debug_distance = *distance;
}

bool i2c_receive(struct i2c_periph *bus, struct i2c_transaction *trans, uint8_t address, uint16_t length)
{
  assert(bus == &test_bus && address == 0xA4 && length == 2);
  if (!accept_request) { return false; }
  reads++;
  trans->status = I2CTransPending;
  return true;
}

bool i2c_transmit(struct i2c_periph *bus, struct i2c_transaction *trans, uint8_t address, uint8_t length)
{
  assert(bus == &test_bus && address == 0xA4 && length == 1);
  assert(trans->buf[0] == RANGEFINDER_I2C_READ_MODE_SINGLE);
  if (!accept_request) { return false; }
  writes++;
  trans->status = I2CTransPending;
  return true;
}

void AbiSendMsgAGL(uint8_t sender, uint32_t stamp, float distance)
{
  assert(sender == AGL_RANGEFINDER_I2C_ID && stamp == 1000);
  publications++;
  last_distance = distance;
}

static void request_read(void)
{
  rangefinder_i2c_periodic();
#if RANGEFINDER_I2C_READ_MODE_SINGLE != 0
  assert(rangefinder_i2c.status == RANGEFINDER_I2C_WAIT_REQUEST);
  rangefinder_i2c.trans.status = I2CTransSuccess;
  rangefinder_i2c_event();
  rangefinder_i2c_periodic();
#endif
  assert(rangefinder_i2c.status == RANGEFINDER_I2C_WAIT_DATA);
}

int main(void)
{
  rangefinder_i2c_init();
  assert(isnan(rangefinder_i2c.distance));
  accept_request = false;
  rangefinder_i2c_periodic();
  rangefinder_i2c_event();
  assert(reads == 0 && writes == 0 && publications == 0);
  assert(debug_messages == 0 && telemetry_messages == 0);
  assert((telemetry_callback != NULL) == (PERIODIC_TELEMETRY != 0));
  telemetry_callback = NULL;
  rangefinder_i2c_init();
  accept_request = true;
  request_read();
  unsigned old_reads = reads;
  for (unsigned step = 0; step < 1000; step++) {
    rangefinder_i2c_periodic();
    rangefinder_i2c_event();
  }
  assert(reads == old_reads && publications == 0);
  assert(debug_messages == 0 && telemetry_messages == 0);
  rangefinder_i2c.trans.buf[0] = 0x0B;
  rangefinder_i2c.trans.buf[1] = 0xB8;
  rangefinder_i2c.trans.status = I2CTransSuccess;
  rangefinder_i2c_event();
  assert(debug_messages == 0);
  rangefinder_i2c_periodic();
  assert(publications == 1 && fabsf(last_distance - 3.f) < 0.0001f);
  assert(debug_messages == RANGEFINDER_I2C_SYNC_SEND && telemetry_messages == 0);
  if (RANGEFINDER_I2C_SYNC_SEND) {
    assert(debug_raw == 3000 && fabsf(debug_distance - 3.f) < 0.0001f);
  }
  if (telemetry_callback != NULL) {
    telemetry_callback(NULL, NULL);
    assert(telemetry_messages == 1);
    assert(debug_messages == RANGEFINDER_I2C_SYNC_SEND);
  }
  rangefinder_i2c_event();
  assert(debug_messages == RANGEFINDER_I2C_SYNC_SEND);
  request_read();
  rangefinder_i2c.trans.status = I2CTransFailed;
  rangefinder_i2c_event();
  assert(publications == 1 && isnan(rangefinder_i2c.distance));
  assert(debug_messages == RANGEFINDER_I2C_SYNC_SEND);
  request_read();
  rangefinder_i2c.trans.buf[0] = 0xFF;
  rangefinder_i2c.trans.buf[1] = 0xFF;
  rangefinder_i2c.trans.status = I2CTransSuccess;
  rangefinder_i2c_event();
  rangefinder_i2c_periodic();
  assert(publications == 1 && isnan(rangefinder_i2c.distance));
  assert(debug_messages == 2 * RANGEFINDER_I2C_SYNC_SEND);
  if (RANGEFINDER_I2C_SYNC_SEND) {
    assert(debug_raw == UINT16_MAX && isnan(debug_distance));
  }
  request_read();
  rangefinder_i2c.trans.buf[0] = 0;
  rangefinder_i2c.trans.buf[1] = 0;
  rangefinder_i2c.trans.status = I2CTransSuccess;
  rangefinder_i2c_event();
  rangefinder_i2c_periodic();
  assert(publications == 1 && isnan(rangefinder_i2c.distance));
  assert(debug_messages == 3 * RANGEFINDER_I2C_SYNC_SEND);
  rangefinder_i2c_report();
  assert(debug_messages == 4 * RANGEFINDER_I2C_SYNC_SEND);
  assert(telemetry_messages == (PERIODIC_TELEMETRY ? 1U : 0U));
  puts("Rangefinder async transaction and two-byte buffer checks passed");
  return 0;
}