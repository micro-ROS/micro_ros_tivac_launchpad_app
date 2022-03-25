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

#include <rosrider_firmware/include/rosrider.h>
#include <rosrider_firmware/ros_encoders/ros_encoders.h>
#include <rosrider_firmware/ros_motors/ros_motors.h>
#include <rosrider_firmware/ros_leds/ros_leds.h>
#include <rosrider_firmware/ros_monitor/ros_monitor.h>
#include <rosrider_firmware/include/parameters.h>
#include <rosrider_firmware/ros_simple/ros_simple.h>

typedef enum {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} microros_state_t;

bool init_microros_entites();
bool fini_microros_entities();

extern rclc_executor_t executor;
extern rcl_publisher_t left_state_pub;
extern rcl_publisher_t right_state_pub;
extern rcl_publisher_t left_position_pub;
extern rcl_publisher_t right_position_pub;
extern rcl_publisher_t diagnostic_pub;

extern rcl_timer_t dead_line_timer;

extern "C" uint32_t max_used_stack();
void diagnostic_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	static std_msgs__msg__String msg = {0};

	static char buffer[1000] = {0};
	snprintf(buffer, 1000, "Stack usage: %d/%d B   Heap usage: %d/%d B   Bus voltage: %d mV   Bus current %d mA", max_used_stack(), STACK_SIZE*4, max_used_heap(), HEAP_SIZE, (int) bus_voltage * 1000, (int) bus_current);
	msg.data.data = buffer;
	msg.data.size = strlen(buffer);
	msg.data.capacity = 1000;

	(void)! rcl_publish(&diagnostic_pub, &msg, NULL);
}

void state_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	std_msgs__msg__Int32 msg;

	measure_encoders();

	int32_t c_left = vel_left * encdir_left;
	int32_t c_right = vel_right * encdir_right;

	msg.data = c_left * ROUNDS_PER_MINUTE;
	(void)! rcl_publish(&left_state_pub, &msg, NULL);

	msg.data = c_right * ROUNDS_PER_MINUTE;
	(void)! rcl_publish(&right_state_pub, &msg, NULL);

	msg.data = (int32_t) pos_left;
	(void)! rcl_publish(&left_position_pub, &msg, NULL);

	msg.data = (int32_t) pos_right;
	(void)! rcl_publish(&right_position_pub, &msg, NULL);
}

void left_control_sub_callback(const void * msg_in)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msg_in;
	pwm_left = (int16_t) msg->data;

	if(pwm_left >= 0.0) {
		left_phase(FORWARD);
	} else if(pwm_left < 0.0) {
		left_phase(REVERSE);
	}

	left_pwm(abs(pwm_left));

	(void)! rcl_timer_reset(&dead_line_timer);

	// Set LEDs to green
	program_leds(0x00FF00FF,0x00FF00FF,0x00FF00FF,0x00FF00FF,0x00FF00FF, 2.0);
}

void right_control_sub_callback(const void * msg_in)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msg_in;
	pwm_right = (int16_t) msg->data;

	if(pwm_right >= 0.0) {
		right_phase(FORWARD);
	} else if(pwm_right < 0.0) {
		right_phase(REVERSE);
	}
	right_pwm(abs(pwm_right));

	(void)! rcl_timer_reset(&dead_line_timer);

	// Set LEDs to green
	program_leds(0x00FF00FF,0x00FF00FF,0x00FF00FF,0x00FF00FF,0x00FF00FF, 2.0);

}

void dead_line_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	left_pwm(0);
	right_pwm(0);

	// Set LEDs to red
	program_leds(0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF, 2.0);

}

extern "C" void microros_app(void * arg)
{
	// ---- RosRider init ----
	init_system();
	init_I2C2();
    init_monitor();

	// self power diagnostics
    self_power_diagnostics();

    // // do not start system, and go to hibernate after 100 seconds
    // if(power_status > 1) { self_power_fail(); }

    // initialize eeprom
    initialize_eeprom();

    // if default_status > 0, read eeprom parameters
    if(DEFAULT_STATUS > 0) {
        read_EEPROM_PARAMETERS();
        recalculate_params();
    }

	// start initializing system
    init_motors(LEFT_REVERSE, RIGHT_REVERSE);
    init_encoders(ui32SysClkFreq / UPDATE_RATE, LEFT_SWAP, RIGHT_SWAP, LEFT_ENC_AB, RIGHT_ENC_AB);

    // init_leds must be last, or it alters pwm characteristics
    init_leds();

    // main timer init, with default update rate
    Timer0Init();

    // led timer init @ 1.0Hz
    Timer1Init(1.0f);

	program_leds(0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF,0xFF0000FF, 2.0);

	// ---- micro-ROS init ----
	rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
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

	// ---- main loop ----
	microros_state_t state = WAITING_AGENT;

	while(1) {
		handle_leds();
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
			EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
