/*
 * A7 liveness watchdog.
 *
 * Every valid PING from the A7 kicks it. While kicks keep arriving the
 * orange LED LD7 blinks; if none arrives for HEARTBEAT_TIMEOUT_MS the link
 * counts as lost and LD7 goes off. A blinking LD7 therefore proves the
 * whole chain: remoteproc loaded and started the M4, rpmsg is up, and the
 * A7 application is alive and talking.
 *
 * The same pattern later guards motion commands: the M4 must not keep
 * acting on stale commands when the A7 goes quiet (VISION.md, fail safe).
 */
#ifndef DONTFALL_M4_HEARTBEAT_H_
#define DONTFALL_M4_HEARTBEAT_H_

/* A7 sends a PING every 500 ms; three missed in a row = link lost. */
#define HEARTBEAT_TIMEOUT_MS 1500

/* Configure LD7 (off). Returns 0, or a negative errno. */
int heartbeat_init(void);

/* A heartbeat arrived from the A7. Thread context. */
void heartbeat_kick(void);

#endif /* DONTFALL_M4_HEARTBEAT_H_ */
