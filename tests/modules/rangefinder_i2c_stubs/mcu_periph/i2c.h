#pragma once
#include "std.h"
enum I2CTransactionStatus { I2CTransPending, I2CTransRunning, I2CTransSuccess, I2CTransFailed, I2CTransDone };
struct i2c_transaction { enum I2CTransactionStatus status; uint8_t buf[2]; };
struct i2c_periph { int unused; };
extern struct i2c_periph test_bus;
extern bool i2c_receive(struct i2c_periph *, struct i2c_transaction *, uint8_t, uint16_t);
extern bool i2c_transmit(struct i2c_periph *, struct i2c_transaction *, uint8_t, uint8_t);