/*
 * micro-ROS node on the M4. See uros.h.
 *
 * Runs in its own preemptible thread, below the coop ipc threads: rclc's
 * executor blocks in the transport read while it waits for data.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uros.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <sensor_msgs/msg/imu.h>

#include "imu.h"
#include "uros_transport.h"

#define UROS_STACK_SIZE   16384
#define UROS_PRIORITY     K_PRIO_PREEMPT(5)
/* 50 Hz: telemetry for the PC, the IMU itself samples at ~850 Hz */
#define PUBLISH_PERIOD_MS 20
/* Orientation variance, rad^2. Roll and pitch: (1 deg)^2, the accuracy
 * aimed for at rest (docs/archive/imu-plan.md step 6); worse while
 * accelerating.
 * Yaw: not estimated (always 0), so "anything".
 */
#define TILT_VARIANCE     3e-4
#define YAW_VARIANCE      1e6
#define AGENT_PING_MS     100
/* how often to look for the Agent while it's away */
#define AGENT_SEARCH_MS   500
/* how often to check a connected Agent is still there */
#define AGENT_CHECK_MS    1000
/* time to wait for the Agent's clock when syncing the session */
#define TIME_SYNC_MS      1000

K_THREAD_STACK_DEFINE(uros_stack, UROS_STACK_SIZE);
static struct k_thread uros_thread;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_timer_t timer;
static rclc_executor_t executor;
static sensor_msgs__msg__Imu msg;
static char frame_id[] = "imu_link";

/* Fields that never change: set once, the timer only fills in the data. */
static void imu_msg_init(void)
{
	memset(&msg, 0, sizeof(msg));
	msg.header.frame_id.data = frame_id;
	msg.header.frame_id.size = strlen(frame_id);
	msg.header.frame_id.capacity = sizeof(frame_id);
	/* row-major 3x3 about x, y, z; the other covariances stay all zero =
	 * unknown (REP 145)
	 */
	msg.orientation_covariance[0] = TILT_VARIANCE;
	msg.orientation_covariance[4] = TILT_VARIANCE;
	msg.orientation_covariance[8] = YAW_VARIANCE;
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	struct imu_sample s;

	ARG_UNUSED(last_call_time);

	if (!timer || imu_latest(&s) != IMU_OK) {
		return;
	}

	/* Agent's clock (synced in create_entities), so PC tools see real
	 * times. Publish time, not sample time: at most one sample (~1.2 ms)
	 * apart.
	 */
	int64_t now = rmw_uros_epoch_nanos();

	msg.header.stamp.sec = (int32_t)(now / 1000000000);
	msg.header.stamp.nanosec = (uint32_t)(now % 1000000000);
	msg.orientation.x = s.orientation[0];
	msg.orientation.y = s.orientation[1];
	msg.orientation.z = s.orientation[2];
	msg.orientation.w = s.orientation[3];
	msg.angular_velocity.x = s.gyro[0];
	msg.angular_velocity.y = s.gyro[1];
	msg.angular_velocity.z = s.gyro[2];
	msg.linear_acceleration.x = s.accel[0];
	msg.linear_acceleration.y = s.accel[1];
	msg.linear_acceleration.z = s.accel[2];

	rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);

	if (rc != RCL_RET_OK) {
		printk("uros: publish failed: %d\n", (int)rc);
	}
}

/* print the failing call and bail out of create_entities() */
#define UROS_CHECK(fn)                                                         \
	do {                                                                   \
		rcl_ret_t rc_ = (fn);                                          \
		if (rc_ != RCL_RET_OK) {                                       \
			printk("uros: %s failed: %d\n", #fn, (int)rc_);       \
			return false;                                          \
		}                                                              \
	} while (0)

/* Open a session with the Agent and create the node and its entities. */
static bool create_entities(void)
{
	UROS_CHECK(rclc_support_init(&support, 0, NULL, &allocator));
	UROS_CHECK(rclc_node_init_default(&node, "dontfall_m4", "", &support));
	/* best effort, like ROS's sensor data QoS: a lost sample doesn't
	 * matter, and nothing should wait for acks
	 */
	UROS_CHECK(rclc_publisher_init_best_effort(&publisher, &node,
						   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
						   "~/imu"));
	/* not fatal: without it stamps are M4 uptime instead of wall time */
	if (rmw_uros_sync_session(TIME_SYNC_MS) != RMW_RET_OK) {
		printk("uros: time sync with the Agent failed\n");
	}
	UROS_CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
					    timer_callback, true));
	UROS_CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	UROS_CHECK(rclc_executor_add_timer(&executor, &timer));
	return true;
}

/*
 * Free everything create_entities() made. The Agent is gone (or never
 * answered), so don't wait for it to confirm the deletes. Also safe after
 * a partial create: the fini calls skip zero-initialized entities.
 */
static void destroy_entities(void)
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);

	if (rmw_context) {
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	}

	/* errors are expected here (entities never made, no Agent to answer) */
	rcl_ret_t rc;

	rc = rclc_executor_fini(&executor);
	rc = rcl_timer_fini(&timer);
	rc = rcl_publisher_fini(&publisher, &node);
	rc = rcl_node_fini(&node);
	rc = rclc_support_fini(&support);
	(void)rc;

	executor = rclc_executor_get_zero_initialized_executor();
	timer = rcl_get_zero_initialized_timer();
	publisher = rcl_get_zero_initialized_publisher();
	node = rcl_get_zero_initialized_node();
	memset(&support, 0, sizeof(support));
	/* fini on half-made entities sets errors nobody reads */
	rcl_reset_error();
}

static bool agent_alive(void)
{
	return rmw_uros_ping_agent(AGENT_PING_MS, 1) == RMW_RET_OK;
}

static void uros_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	allocator = rcl_get_default_allocator();
	executor = rclc_executor_get_zero_initialized_executor();
	timer = rcl_get_zero_initialized_timer();
	publisher = rcl_get_zero_initialized_publisher();
	node = rcl_get_zero_initialized_node();
	imu_msg_init();

	if (uros_transport_init()) {
		printk("uros: transport init failed\n");
		return;
	}

	/*
	 * Find the Agent, create everything, spin until a ping goes
	 * unanswered, then tear down and start over. A restarted Agent has
	 * forgotten our session, so everything has to be created again.
	 */
	for (;;) {
		printk("uros: waiting for the micro-ROS Agent\n");
		while (!agent_alive()) {
			k_sleep(K_MSEC(AGENT_SEARCH_MS));
		}
		printk("uros: Agent found\n");

		if (!create_entities()) {
			destroy_entities();
			k_sleep(K_MSEC(AGENT_SEARCH_MS));
			continue;
		}
		printk("uros: publishing /dontfall_m4/imu\n");

		int64_t next_check = k_uptime_get() + AGENT_CHECK_MS;

		for (;;) {
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			if (k_uptime_get() < next_check) {
				continue;
			}
			next_check = k_uptime_get() + AGENT_CHECK_MS;
			if (!agent_alive()) {
				break;
			}
		}

		printk("uros: Agent lost\n");
		destroy_entities();
		uros_transport_reset();
	}
}

int uros_start(void)
{
	k_thread_create(&uros_thread, uros_stack, K_THREAD_STACK_SIZEOF(uros_stack), uros_task,
			NULL, NULL, NULL, UROS_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uros_thread, "uros");
	return 0;
}
