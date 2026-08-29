#ifndef TEST_SDP3X_I2C_H
#define TEST_SDP3X_I2C_H

#include <stdbool.h>
#include <stdint.h>

enum I2CTransactionStatus {
  I2CTransPending,
  I2CTransRunning,
  I2CTransSuccess,
  I2CTransFailed,
  I2CTransDone
};

struct i2c_transaction {
  uint16_t len_r;
  uint8_t len_w;
  uint8_t slave_addr;
  volatile uint8_t buf[32];
  volatile enum I2CTransactionStatus status;
};

struct i2c_periph {
  uint8_t unused;
};

extern struct i2c_periph i2c2;

bool i2c_transmit(struct i2c_periph *periph, struct i2c_transaction *transaction,
                  uint8_t address, uint8_t length);
bool i2c_receive(struct i2c_periph *periph, struct i2c_transaction *transaction,
                 uint8_t address, uint16_t length);

#endif
