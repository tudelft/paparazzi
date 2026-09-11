#ifndef CATIA_SERIAL_TX_H
#define CATIA_SERIAL_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int serial_tx_start(int descriptor);
void serial_tx_stop(void);
int serial_tx_send(uint8_t message, const uint8_t *payload, size_t length);
int serial_tx_send_if_idle(uint8_t message, const uint8_t *payload, size_t length);
int serial_tx_flush(void);
bool serial_tx_pending(void);

#endif