#include <ServoInput.h>
#include <Servo.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
/*
  GITMRG Nova V1.1 Custom BLDC Thruster Driver

  v1.1 RoboBoat Testing

  Resources
  https://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide
  https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void error_cb(){
  delay(10);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



// PINS -------------------------------------------------------------
// BALL SHOOTER MOTORS
// MOTOR ALFA
const int A_SIG_PIN = 3;
Servo motor_a;
// MOTOR BRAVO
const int B_SIG_PIN = 4;
Servo motor_b;
// BEAM BREAKS
const int BB_A_PIN = 5;
const int BB_B_PIN = 6;
const int BB_C_PIN = 7;

// WATER SHOOTER MOTOR
const int BP_SIG_PIN = A1;
Servo motor_bp; //bilge pump

// VARS -------------------------------------------------------------
unsigned long loop_time = 0;
unsigned long last_time = 0;

// DEVICES ----------------------------------------------------------
// MOTORS

// FUNCTIONS --------------------------------------------------------

//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_publisher/micro-ros_publisher.ino
//https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_subscriber/micro-ros_subscriber.ino
//rcl_publisher_t vehicle_state_pub;
rcl_subscription_t motor_a_sub; // A (left_rear)
rcl_subscription_t motor_b_sub; //Nick Code  :C // B (left_front)
rcl_subscription_t motor_bp_sub; // C (right_front)

rclc_executor_t executor;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msg_a;
std_msgs__msg__Int32 msg_b;
std_msgs__msg__Int32 msg_bp;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

//Nick Code :^(

float limit_coefficient = -1 * .9;

void motor_a_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_a = int(val * 100 * limit_coefficient);
//  Serial.print("ros_left_rear_thrust: ");
//  Serial.println(ros_left_rear_thrust);
}

void motor_b_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_b = 1 * val * 100 * limit_coefficient;
}

void motor_bp_callback(const void * msgin) 
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float val = msg->data;
  ros_cmd_bp = val * 100 * limit_coefficient;
}

bool ros_create_entities() {
  // Initialize micro-ROS allocator
  
  delay(1000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = (size_t)(12);
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));
  // create publisher
//  RCCHECK(rclc_publisher_init_default(
//    &vehicle_state_pub,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//    "/wamv/nova/mode"));
  // create thrust subscribers
  RCCHECK(rclc_subscription_init_default(
    &motor_a_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ball_shooter/throttle_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_b_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ball_shooter/throttle_cmd"));
  RCCHECK(rclc_subscription_init_default(
    &motor_bp_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/water_shooter/throttle_cmd"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); // Increment this for more subs
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg0, &vehicle_state_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(&executor, &motor_a_sub, &msg_a, &motor_a_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_b_sub, &msg_b, &motor_b_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_bp_sub, &msg_bp, &motor_bp_callback, ON_NEW_DATA));

  msg.data = 0;
  msg_a.data = 0;
  msg_b.data = 0;
  msg_bp.data = 0;
  return true;
}

void ros_destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //rcl_publisher_fini(&vehicle_state_pub, &node);
  rcl_subscription_fini(&motor_a_sub, &node);
  rcl_subscription_fini(&motor_b_sub, &node);
  rcl_subscription_fini(&motor_bp_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void exec_mode(int mode, bool killed) {
  // Publish Vehicle State
  else {
    if (mode == 0){ // AUTONOMOUS
      ros_handler();
      motor_a.writeMicroseconds(ros_cmd_a*4 + 1500);
      motor_b.writeMicroseconds(ros_cmd_b*4 + 1500);
      motor_bp.writeMicroseconds(ros_cmd_bp*4 + 1500);
      delay(20);
//      msg_x.data = 2;
    }
    else if (mode == 1) { // TEST
      motor_a.writeMicroseconds(50*4 + 1500);
      motor_b.writeMicroseconds(50*4 + 1500);
      motor_bp.writeMicroseconds(100*4 + 1500);
    }
    else {
      Serial.println("Error, mode not supported");
    }
  }
}

void setup() {
  Serial.begin(9600);
  loop_time = millis();
  
  delay(100);
  Serial.println("AUX SYSTEM STARTING...");

  Serial.println("MOTOR SETUP");
  motor_a.attach(A_SIG_PIN);
  motor_b.attach(B_SIG_PIN);
  motor_bp.attach(BP_SIG_PIN);

  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_bp.writeMicroseconds(1500);

  Serial.println("============= CALIBRATION COMPLETE ===============");
  delay(500);
  set_microros_transports();
  
  state = WAITING_AGENT;

  Serial.println("==================================================");
  Serial.println("============ NOVA MOTOR INIT COMPLETE ============");
  Serial.println("==================================================");
}

void zero_all_motors() {
  motor_a.writeMicroseconds(1500);
  motor_b.writeMicroseconds(1500);
  motor_bp.writeMicrosecodns(1500;)
}
void zero_ros_cmds() {
  ros_cmd_a = 0;
  ros_cmd_b = 0;
  ros_cmd_bp = 0;
}


void ros_handler() {
  bool created = false;
  switch (state) {
    case WAITING_AGENT:
      cfg_lt(0, 0, 3, 0);
      zero_ros_cmds();
      zero_all_motors();
      EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      
      break;
    case AGENT_AVAILABLE:
      cfg_lt(0, 0, 2, 0);
      zero_ros_cmds();
      zero_all_motors();
      created = ros_create_entities();
      state = (true == created) ? AGENT_CONNECTED : WAITING_AGENT;
      delay(100);
      if (state == WAITING_AGENT) {
        ros_destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      cfg_lt(0, 0, 1, 0);
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      break;
    case AGENT_DISCONNECTED:
      cfg_lt(3, 0, 3, 0);
      zero_ros_cmds();
      zero_all_motors();
      ros_destroy_entities();
      delay(100);
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

// TODO: Refactor to use control_state
bool boat_killed = false;
int cmd_ctr = 1;  // 0 if auto, 1 if test

void loop() {
  // Get loop time
  loop_time = millis();

  boat_killed = false;
  exec_mode(cmd_ctr, boat_killed);

  if (state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}
