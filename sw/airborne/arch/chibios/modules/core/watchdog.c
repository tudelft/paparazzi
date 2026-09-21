#include "modules/core/watchdog.h"
#include "hal.h"
#include "modules/datalink/telemetry.h"

#if !defined(STM32F4XX)
#error "AP watchdog reset diagnostics currently require STM32F4"
#endif

#if USE_HARD_FAULT_RECOVERY
#error "AP watchdog has not been integrated with hard-fault recovery"
#endif

#ifndef WATCHDOG_TIMEOUT_MS
#define WATCHDOG_TIMEOUT_MS 2000U
#endif

#define WATCHDOG_RELOAD_TICKS ((WATCHDOG_TIMEOUT_MS * STM32_LSICLK + 63999U) / 64000U)

#if WATCHDOG_TIMEOUT_MS < 250 || WATCHDOG_TIMEOUT_MS > 8192
#error "WATCHDOG_TIMEOUT_MS must be between 250 and 8192"
#endif

#if WATCHDOG_RELOAD_TICKS < 1 || WATCHDOG_RELOAD_TICKS > 4096
#error "Watchdog timeout does not fit the IWDG reload register"
#endif

static const WDGConfig watchdog_config = {
  .pr = STM32_IWDG_PR_64,
  .rlr = STM32_IWDG_RL(WATCHDOG_RELOAD_TICKS - 1U)
};

static uint32_t watchdog_reset_flags;
static bool watchdog_started;
static bool watchdog_progress;

#if PERIODIC_TELEMETRY
static void send_watchdog(struct transport_tx *trans, struct link_device *dev)
{
  float values[] = {watchdog_reset_flags >> 24, watchdog_started, WATCHDOG_TIMEOUT_MS};
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID, 8, "watchdog", 3, values);
}
#endif

void watchdog_capture_reset_cause(void)
{
  watchdog_reset_flags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
}

void watchdog_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, send_watchdog);
#endif
}

void watchdog_start(void)
{
  watchdog_progress = false;
  wdgStart(&WDGD1, &watchdog_config);
  watchdog_started = true;
}

void watchdog_periodic(void)
{
  watchdog_progress = true;
}

void watchdog_feed(void)
{
  if (watchdog_started && watchdog_progress) {
    watchdog_progress = false;
    wdgReset(&WDGD1);
  }
}