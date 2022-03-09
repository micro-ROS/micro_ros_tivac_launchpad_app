#include "./config.h"
#include "./microros_usbcdc_transport.h"
#include "./microros_allocators.h"
#include "./microros_parameters.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/timing.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <rosrider_firmware/include/rosrider.h>
#include <rosrider_firmware/ros_motors/ros_motors.h>

// #include "rosrider.h"
// #include "ros_encoders.h"
// #include "ros_motors.h"
// #include "ros_leds.h"
// #include "ros_monitor.h"
// #include "parameters.h"
// #include "rosrider/SysCtrlService.h"
// #include "ros_simple.h"


#define CHECK_AND_CONTINUE(X) if (!ret || !(X)){ret = false;}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;

rcl_node_t node;
rclc_executor_t executor;

const rclc_parameter_options_t parameter_options = {.notify_changed_over_dds = true, .max_params = PARAMETERS_SIZE};
rclc_parameter_server_t param_server;

rcl_timer_t state_pub_timer;

rcl_publisher_t left_state_pub;
rcl_publisher_t right_state_pub;
rcl_publisher_t left_position_pub;
rcl_publisher_t right_position_pub;

std_msgs__msg__Int32 recv_msgs[2];
rcl_subscription_t left_control_sub;
rcl_subscription_t right_control_sub;

#ifdef DIAGNOSTIC_PUB
rcl_timer_t diagnostic_pub_timer;
rcl_publisher_t diagnostic_pub;

uint32_t max_used_stack();
void diagnostic_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static char buf[300];

	snprintf(buf, sizeof(buf), "max_stack: %lu / %u B\n max_heap: %lu / %u B\n current %d\n voltage %d\n",
		max_used_stack(),
		STACK_SIZE * sizeof(uint32_t),
		max_used_heap(),
		HEAP_SIZE,
		0,
		0);

	std_msgs__msg__String msg;
	msg.data.data = buf;
	msg.data.size = strlen(buf);
	msg.data.capacity = sizeof(buf);

	(void)! rcl_publish(&diagnostic_pub, &msg, NULL);
}
#endif

void state_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	std_msgs__msg__Int32 msg;

	msg.data = get_velocity(&encoder_left);
	(void)! rcl_publish(&left_state_pub, &msg, NULL);

	msg.data = get_velocity(&encoder_right);
	(void)! rcl_publish(&right_state_pub, &msg, NULL);

	msg.data = get_position(&encoder_left);
	(void)! rcl_publish(&left_position_pub, &msg, NULL);

	msg.data = get_position(&encoder_right);
	(void)! rcl_publish(&right_position_pub, &msg, NULL);
}

void left_control_sub_callback(const void * msg_in)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msg_in;
	int32_t pwm_left = msg->data;

	if(pwm_left >= 0) { left_phase(FORWARD); } else if(pwm_left < 0) { left_phase(REVERSE); }
	left_pwm(abs(pwm_left));
}

void right_control_sub_callback(const void * msg_in)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msg_in;
	int32_t pwm_right = msg->data;

	if(pwm_right >= 0) { right_phase(FORWARD); } else if(pwm_right < 0) { right_phase(REVERSE); }
    right_pwm(abs(pwm_right));
}


bool init_microros_entites() {
	bool ret = true;

	allocator = rcl_get_default_allocator();

	init_options = rcl_get_zero_initialized_init_options();
	CHECK_AND_CONTINUE(RCL_RET_OK == rcl_init_options_init(&init_options, allocator));

	// create init_options
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	(void)! rcl_init_options_fini(&init_options);

	// sync time
	CHECK_AND_CONTINUE(RCL_RET_OK == rmw_uros_sync_session(1000.0));

	// create node
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_node_init_default(&node, "rosrider", "", &support));

	// create publishers
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&left_state_pub, 	 &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_wheel/state"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&right_state_pub, 	 &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_wheel/state"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&left_position_pub,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_wheel/position"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&right_position_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_wheel/position"));

	// create timer,
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&state_pub_timer, &support, RCL_MS_TO_NS(20), state_pub_timer_callback));

	// create subscribers
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_subscription_init_default(&left_control_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_wheel/control_effort"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_subscription_init_default(&right_control_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_wheel/control_effort"));

	// create parameter server
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_parameter_server_init_with_option(&param_server, &node, &parameter_options));

	// create executor
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_init(&executor, &support.context, 1 /* timer */ + 2 /* subs */ + RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER + DIAGNOSTIC_PUB_HANDLE, &allocator));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &state_pub_timer));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_subscription(&executor, &left_control_sub, &recv_msgs[0], &left_control_sub_callback, ON_NEW_DATA));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_subscription(&executor, &left_control_sub, &recv_msgs[1], &right_control_sub_callback, ON_NEW_DATA));
 	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

	initialize_parameters(&param_server);

#ifdef DIAGNOSTIC_PUB
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&diagnostic_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/diagnostic"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&diagnostic_pub_timer, &support, RCL_MS_TO_NS(1000), diagnostic_pub_timer_callback));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &diagnostic_pub_timer));
#endif

	return ret;
}

bool fini_microros_entities() {
	bool ret = true;

 	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	(void)! rcl_publisher_fini(&left_state_pub, &node);
	(void)! rcl_publisher_fini(&right_state_pub, &node);
	(void)! rcl_publisher_fini(&left_position_pub, &node);
	(void)! rcl_publisher_fini(&right_position_pub, &node);

	(void)! rcl_timer_fini(&state_pub_timer);

	(void)! rcl_subscription_fini(&left_control_sub, &node);
	(void)! rcl_subscription_fini(&right_control_sub, &node);

	(void)! rclc_parameter_server_fini(&param_server, &node);

#ifdef DIAGNOSTIC_PUB
	(void)! rcl_publisher_fini(&diagnostic_pub, &node);
	(void)! rcl_timer_fini(&diagnostic_pub_timer);
#endif

	(void)! rclc_executor_fini(&executor);
	(void)! rcl_node_fini(&node);
	(void)! rclc_support_fini(&support);

	free_all_heap();

	return ret;
}

void microros_app(void * arg)
{
	// Rosrider init
	init_rosrider_system();
	init_motors(false, false);
	init_encoders(MAP_SysCtlClockGet() / 10, false, false, true, true);

	allocator = rcutils_get_zero_initialized_allocator();
	allocator.allocate = custom_allocate;
	allocator.deallocate = custom_deallocate;
	allocator.reallocate = custom_reallocate;
	allocator.zero_allocate = custom_zero_allocate;

  	(void)! rcutils_set_default_allocator(&allocator);

    rmw_uros_set_custom_transport(
		true,
		(void *) NULL,
		tivac_usbcdc_transport_open,
		tivac_usbcdc_transport_close,
		tivac_usbcdc_transport_write,
		tivac_usbcdc_transport_read
	);

	enum states {
		WAITING_AGENT,
		AGENT_AVAILABLE,
		AGENT_CONNECTED,
		AGENT_DISCONNECTED
	} state = WAITING_AGENT;

	while(1) {
		switch (state) {
		case WAITING_AGENT:
			EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == init_microros_entites()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				fini_microros_entities();
			};
			break;
		case AGENT_CONNECTED:
			EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			}
			break;
		case AGENT_DISCONNECTED:
			fini_microros_entities();
			state = WAITING_AGENT;
			break;
		default:
			break;
    	}
	}
}
