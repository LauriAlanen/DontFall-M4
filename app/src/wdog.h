/*
 * M4 watchdog: WWDG1 resets the M4 when the firmware hangs (DontFall
 * docs/archive/failsafe-plan.md step 5). Linux sees that reset as a crash
 * (stm32_rproc's "wdg" interrupt, EXTI 68): it writes a core dump and
 * restarts the firmware.
 *
 * Who kicks it: a thread at the lowest application priority, and only
 * when every source below has checked in since the last kick (a task
 * monitor). So the watchdog fires when
 *  - any thread or ISR spins: everything above the kicker starves it,
 *  - interrupts stay locked or the kernel timers stop: no check-ins,
 *  - a check-in source stops.
 *
 * Not kicked from the IMU sampling path: a lost INT1 stops that path
 * while the M4 is fine, and the supervisor already handles it (disarmed,
 * reason "imu"); a reset would only lose that reason.
 *
 * WWDG1 keeps counting while a debugger halts the M4: freezing it needs
 * DBGMCU, which Linux owns.
 */
#ifndef DONTFALL_M4_WDOG_H_
#define DONTFALL_M4_WDOG_H_

enum wdog_source {
	WDOG_CONTROL_TICK, /* control.c's no-sample timer */
	WDOG_SOURCE_COUNT,
};

/* Start WWDG1 and the kicker. Once started it can't be stopped. Returns
 * 0 or the watchdog API's error.
 */
int wdog_start(void);

/* source is alive. Any context, also ISRs. */
void wdog_checkin(enum wdog_source source);

#endif /* DONTFALL_M4_WDOG_H_ */
