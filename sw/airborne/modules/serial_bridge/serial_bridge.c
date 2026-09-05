#include "modules/serial_bridge/serial_bridge.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/udp.h"

#ifndef SERIAL_BRIDGE_UART
#error "SERIAL_BRIDGE_UART not defined"
#endif

#ifndef SERIAL_BRIDGE_UDP
#error "SERIAL_BRIDGE_UDP not defined"
#endif

void serial_bridge_init(void) {
  // Initialization is handled by the mcu_periph subsystem for UART and UDP
}

void serial_bridge_periodic(void) {
  // Forward UART to UDP
  int uart_avail = uart_char_available(&(SERIAL_BRIDGE_UART));
  if (uart_avail > 0) {
    while (uart_char_available(&(SERIAL_BRIDGE_UART))) {
      uint8_t c = uart_getch(&(SERIAL_BRIDGE_UART));
      udp_put_byte(&(SERIAL_BRIDGE_UDP), 0, c);
    }
    udp_send_message(&(SERIAL_BRIDGE_UDP), 0);
  }

  // Forward UDP to UART
  int udp_avail = udp_char_available(&(SERIAL_BRIDGE_UDP));
  if (udp_avail > 0) {
    while (udp_char_available(&(SERIAL_BRIDGE_UDP))) {
      uint8_t c = udp_getch(&(SERIAL_BRIDGE_UDP));
      uart_put_byte(&(SERIAL_BRIDGE_UART), 0, c);
    }
    uart_send_message(&(SERIAL_BRIDGE_UART), 0);
  }
}
