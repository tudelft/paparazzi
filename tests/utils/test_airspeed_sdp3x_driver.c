#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mcu_periph/i2c.h"
#include "modules/sensors/airspeed_sdp3x.h"

#define SDP3X_STOP_COMMAND 0x3ff9U
#define SDP3X_START_COMMAND 0x3615U

struct i2c_periph i2c2;

static struct i2c_transaction *submitted_transaction;
static unsigned submit_count;
static bool submitted_read;
static uint16_t submitted_length;
static uint16_t submitted_command;

static unsigned pressure_publish_count;
static unsigned temperature_publish_count;
static unsigned airspeed_publish_count;
static float published_pressure;
static float published_temperature;
static float published_airspeed;
static uint8_t pressure_sender_id;
static uint8_t temperature_sender_id;
static uint8_t airspeed_sender_id;

static unsigned test_number;

static void expect(bool condition, const char *name)
{
  test_number++;
  if (!condition) {
    fprintf(stderr, "not ok %u - %s\n", test_number, name);
    exit(EXIT_FAILURE);
  }
  printf("ok %u - %s\n", test_number, name);
}

static bool submit(struct i2c_transaction *transaction, uint8_t address,
                   uint16_t length, bool read)
{
  submitted_transaction = transaction;
  submitted_read = read;
  submitted_length = length;
  submitted_command = read ? 0U :
                      (uint16_t)(((uint16_t)transaction->buf[0] << 8) | transaction->buf[1]);
  transaction->slave_addr = address;
  transaction->status = I2CTransPending;
  submit_count++;
  return true;
}

bool i2c_transmit(struct i2c_periph *periph, struct i2c_transaction *transaction,
                  uint8_t address, uint8_t length)
{
  (void)periph;
  transaction->len_w = length;
  transaction->len_r = 0;
  return submit(transaction, address, length, false);
}

bool i2c_receive(struct i2c_periph *periph, struct i2c_transaction *transaction,
                 uint8_t address, uint16_t length)
{
  (void)periph;
  transaction->len_w = 0;
  transaction->len_r = length;
  return submit(transaction, address, length, true);
}

void test_abi_baro_diff(uint8_t sender_id, float pressure)
{
  pressure_sender_id = sender_id;
  published_pressure = pressure;
  pressure_publish_count++;
}

void test_abi_temperature(uint8_t sender_id, float temperature)
{
  temperature_sender_id = sender_id;
  published_temperature = temperature;
  temperature_publish_count++;
}

void test_abi_airspeed(uint8_t sender_id, float airspeed)
{
  airspeed_sender_id = sender_id;
  published_airspeed = airspeed;
  airspeed_publish_count++;
}

static uint8_t crc8(const uint8_t data[2])
{
  for (uint16_t checksum = 0; checksum <= UINT8_MAX; checksum++) {
    if (sdp3x_crc_valid(data, 2, (uint8_t)checksum)) {
      return (uint8_t)checksum;
    }
  }
  abort();
}

static void put_word(uint8_t *buffer, unsigned offset, uint16_t word)
{
  buffer[offset] = (uint8_t)(word >> 8);
  buffer[offset + 1] = (uint8_t)word;
  buffer[offset + 2] = crc8(&buffer[offset]);
}

static void complete_success(void)
{
  submitted_transaction->status = I2CTransSuccess;
  sdp3x_event();
}

static void complete_scale(uint16_t scale)
{
  put_word((uint8_t *)submitted_transaction->buf, 0, 0U);
  put_word((uint8_t *)submitted_transaction->buf, 3, 4000U);
  put_word((uint8_t *)submitted_transaction->buf, 6, scale);
  complete_success();
}

static void complete_sample(int16_t pressure_counts, int16_t temperature_counts, bool valid_crc)
{
  put_word((uint8_t *)submitted_transaction->buf, 0, (uint16_t)pressure_counts);
  put_word((uint8_t *)submitted_transaction->buf, 3, (uint16_t)temperature_counts);
  if (!valid_crc) {
    submitted_transaction->buf[2] ^= 1U;
  }
  complete_success();
}

static void expect_command(uint16_t command, const char *name)
{
  expect(!submitted_read && submitted_length == 2U && submitted_command == command, name);
}

static void start_to_scale_read(void)
{
  sdp3x_periodic();
  expect_command(SDP3X_STOP_COMMAND, "STOP command submitted");
  complete_success();

  const unsigned after_stop = submit_count;
  sdp3x_periodic();
  expect(submit_count == after_stop, "full callback wait after STOP");
  sdp3x_periodic();
  expect_command(SDP3X_START_COMMAND, "START command submitted");
  complete_success();

  const unsigned after_start = submit_count;
  sdp3x_periodic();
  expect(submit_count == after_start, "full callback wait after START");
  sdp3x_periodic();
  expect(submitted_read && submitted_length == 9U, "nine-byte scale read submitted");
}

int main(void)
{
#if SDP3X_TEST_FILTER_TRANSIENT
  puts("1..26");
#else
  puts("1..25");
#endif
  sdp3x_init();

  start_to_scale_read();
  complete_scale(60U);
  expect(pressure_publish_count == 0U && airspeed_publish_count == 0U,
         "wrong configured scale publishes nothing");
  sdp3x_periodic();
  expect_command(SDP3X_STOP_COMMAND, "scale mismatch restarts at STOP");
  complete_success();

  sdp3x_init();
  start_to_scale_read();
  complete_scale(SDP3X_SCALE_PRESSURE_SDP33);
  sdp3x_periodic();
  expect(submitted_read && submitted_length == 6U, "six-byte sample read submitted");
  complete_sample(-1764, 5000, true);
  expect(pressure_publish_count == 1U && temperature_publish_count == 1U &&
         airspeed_publish_count == 1U, "valid sample publishes all ABI values once");
    expect(pressure_sender_id == 41U, "BARO_DIFF sender ID");
    expect(temperature_sender_id == 41U, "TEMPERATURE sender ID");
    expect(airspeed_sender_id == 3U, "AIRSPEED sender ID");
  expect(fabsf(published_pressure - 88.2f) < 1e-5f, "ABI pressure is in Pa");
  expect(fabsf(published_temperature - 25.f) < 1e-5f, "ABI temperature is in Celsius");
  expect(fabsf(published_airspeed - 12.f) < 1e-5f, "ABI airspeed is EAS in m/s");

  sdp3x_periodic();
  complete_sample(1764, 5000, true);
#if SDP3X_ENABLE_BIDIRECTIONAL
  expect(fabsf(published_pressure + 88.2f) < 1e-5f, "reverse pressure remains signed");
  expect(fabsf(published_airspeed + 12.f) < 1e-5f, "reverse EAS remains signed");
#elif SDP3X_TEST_FILTER_TRANSIENT
  expect(published_pressure >= 0.f, "filtered reverse pressure remains nonnegative");
  expect(published_airspeed >= 0.f, "filtered reverse EAS remains nonnegative");
#else
  expect(fabsf(published_pressure) < 1e-6f, "reverse pressure is clamped for ABI");
  expect(fabsf(published_airspeed) < 1e-6f, "reverse EAS is clamped for ABI");
#endif

#if SDP3X_TEST_FILTER_TRANSIENT
  bool transient_nonnegative = true;
  for (unsigned sample = 0; sample < 300U; sample++) {
    sdp3x_periodic();
    complete_sample(1764, 5000, true);
    transient_nonnegative = transient_nonnegative && published_pressure >= 0.f && published_airspeed >= 0.f;
  }
  expect(transient_nonnegative, "abrupt stop never publishes negative filtered airspeed");
#endif

  const unsigned pressure_count_before_crc = pressure_publish_count;
  const unsigned airspeed_count_before_crc = airspeed_publish_count;
  for (unsigned failure = 0; failure < 5U; failure++) {
    sdp3x_periodic();
    complete_sample(1764, 5000, false);
  }
    expect(pressure_publish_count == pressure_count_before_crc &&
      airspeed_publish_count == airspeed_count_before_crc,
         "CRC failures publish no samples");
  sdp3x_periodic();
  expect_command(SDP3X_STOP_COMMAND, "five CRC failures restart at STOP");
  complete_success();

  sdp3x_init();
  sdp3x_periodic();
  submitted_transaction->status = I2CTransFailed;
  sdp3x_event();
  sdp3x_periodic();
  expect_command(SDP3X_STOP_COMMAND, "I2C failure retries from STOP");

  return EXIT_SUCCESS;
}
