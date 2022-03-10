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
#include <sensor_msgs/msg/battery_state.h>

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

rcl_timer_t diagnostic_pub_timer;
rcl_publisher_t diagnostic_pub;

rcl_timer_t dead_line_timer;

void diagnostic_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void state_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void left_control_sub_callback(const void * msg_in);
void right_control_sub_callback(const void * msg_in);
void dead_line_timer_callback(rcl_timer_t * timer, int64_t last_call_time);


bool init_microros_entites() {
	bool ret = true;
	uint8_t executor_handles = 0;

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
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&state_pub_timer, &support, RCL_MS_TO_NS(100), state_pub_timer_callback));
	executor_handles += 1;

	// create subscribers
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_subscription_init_best_effort(&left_control_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_wheel/control_effort"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_subscription_init_best_effort(&right_control_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/right_wheel/control_effort"));
	executor_handles += 2;

	// create diagnostic
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_best_effort(&diagnostic_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/diagnostic"));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&diagnostic_pub_timer, &support, RCL_MS_TO_NS(1000), diagnostic_pub_timer_callback));
	executor_handles += 1;

	// create parameter server
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_parameter_server_init_with_option(&param_server, &node, &parameter_options));
	executor_handles += RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER;

	// deadline timer
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&dead_line_timer, &support, RCL_MS_TO_NS(100), dead_line_timer_callback));
	executor_handles += 1;

	// create executor
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_init(&executor, &support.context, executor_handles, &allocator));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &state_pub_timer));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_subscription(&executor, &left_control_sub, &recv_msgs[0], &left_control_sub_callback, ON_NEW_DATA));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_subscription(&executor, &left_control_sub, &recv_msgs[1], &right_control_sub_callback, ON_NEW_DATA));
 	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &diagnostic_pub_timer));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &dead_line_timer));

	initialize_parameters(&param_server);

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

	(void)! rcl_publisher_fini(&diagnostic_pub, &node);
	(void)! rcl_timer_fini(&diagnostic_pub_timer);

	(void)! rcl_timer_fini(&dead_line_timer);

	(void)! rclc_executor_fini(&executor);
	(void)! rcl_node_fini(&node);
	(void)! rclc_support_fini(&support);

	free_all_heap();

	return ret;
}