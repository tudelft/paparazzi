#ifndef AP_WATCHDOG_H
#define AP_WATCHDOG_H

void watchdog_capture_reset_cause(void);
void watchdog_init(void);
void watchdog_start(void);
void watchdog_periodic(void);
void watchdog_feed(void);

#endif