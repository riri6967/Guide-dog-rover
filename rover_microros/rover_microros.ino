/*
 * rover_microros.ino — the rover as a ROS 2 node
 *
 * Board:   ESP32 Dev Module, Arduino core 2.0.2
 * Library: micro_ros_arduino v2.0.8-jazzy
 * Agent:   Raspberry Pi 5, ROS 2 Jazzy, over USB serial
 *
 * Subscribes  /cmd_vel   geometry_msgs/msg/Twist
 * Publishes   /odom      nav_msgs/msg/Odometry   at 20 Hz
 *
 * Drive it from the Pi:
 *     ~/start_agent.sh
 *     ros2 run teleop_twist_keyboard teleop_twist_keyboard
 *
 * NOTE: the USB port is the micro-ROS transport. Serial.print does not work
 * here — anything you print corrupts the protocol. Debug via ROS topics or
 * the onboard LED.
 */

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>

#include "pins.h"
#include "motors.h"
#include "encoders.h"

// ===================================================================
//  SET THESE FROM YOUR ROVER — everything downstream depends on them
// ===================================================================

// Pulses per ONE full wheel revolution. Get this from encoder_test.ino:
// press 'c', turn one wheel exactly 10 turns by hand, divide the count by 10.
// Do not trust the slot count printed on the disc.
const float TICKS_PER_REV = 20.0f;          // <-- MEASURE

// Wheel diameter in metres. 65 mm = 0.065
const float WHEEL_DIAMETER_M = 0.065f;      // <-- MEASURE

// Track width: centre of the left wheel to centre of the right wheel, metres.
// This sets how fast the rover thinks it is turning. Get it wrong and the
// robot will spiral in RViz while driving straight in real life.
const float TRACK_WIDTH_M = 0.200f;         // <-- MEASURE

// Top speed of one wheel at full PWM, metres/second. Used to convert the
// m/s that ROS asks for into a PWM duty. Rough is fine to start:
// drive flat out across a measured 2 m and time it.
const float MAX_WHEEL_SPEED_MS = 0.55f;     // <-- MEASURE

// ===================================================================

// Motors stop if no /cmd_vel arrives within this window. Every way of losing
// control — agent killed, cable pulled, Pi crashed, teleop closed — looks the
// same from here: commands stop. Silence means stop, never "carry on".
const unsigned long CMD_TIMEOUT_MS = 900;

const unsigned long ODOM_PERIOD_MS = 50;    // 20 Hz

// ------------------------------------------------------------ ROS entities
rcl_subscription_t   cmd_vel_sub;
rcl_publisher_t      odom_pub;
geometry_msgs__msg__Twist    cmd_vel_msg;
nav_msgs__msg__Odometry      odom_msg;

// Raw cumulative encoder counts, published alongside odometry. These are the
// unprocessed truth: they need no calibration to be useful, so you can watch
// them climb and know the wheels are actually turning long before the
// geometry constants above are measured.
rcl_publisher_t      ticks_l_pub;
rcl_publisher_t      ticks_r_pub;
std_msgs__msg__Int32 ticks_l_msg;
std_msgs__msg__Int32 ticks_r_msg;

// Signed lifetime totals — they count down when the wheel is driven backwards,
// so a forward-then-reverse run should return close to where it started.
int32_t totalTicksL = 0;
int32_t totalTicksR = 0;

rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_timer_t       odom_timer;

// ------------------------------------------------------------ state
unsigned long lastCmdMs = 0;

// Odometry pose, integrated from wheel travel.
double pose_x = 0.0;
double pose_y = 0.0;
double pose_theta = 0.0;

// Single-channel encoders cannot sense direction, so we remember which way we
// last told each side to turn and sign the ticks with that.
int dirLeft  = 1;
int dirRight = 1;

unsigned long lastOdomMs = 0;

// Agent connection state machine. The stock examples give up permanently when
// the agent is missing; a robot must reconnect instead.
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState agentState = WAITING_AGENT;

static char odom_frame[]  = "odom";
static char child_frame[] = "base_link";

// ------------------------------------------------------------ helpers

#define RCCHECK(fn)      { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn)  { rcl_ret_t rc = fn; (void)rc; }

#define EVERY_N_MS(ms, body) { \
  static volatile unsigned long _last = 0; \
  if (millis() - _last >= (ms)) { body; _last = millis(); } }

// Convert a wheel speed in m/s to an L298N duty, -255..255.
int speedToDuty(float metresPerSec) {
  float ratio = metresPerSec / MAX_WHEEL_SPEED_MS;
  int duty = (int)(ratio * Motors::MAX_SPEED);
  return constrain(duty, -Motors::MAX_SPEED, Motors::MAX_SPEED);
}

// ------------------------------------------------------------ callbacks

// ROS hands us a body velocity; differential drive turns that into two wheel
// speeds. Positive angular.z is counter-clockwise, so the right wheel speeds
// up and the left slows down.
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x;     // m/s forward
  float w = msg->angular.z;    // rad/s, CCW positive

  float vLeft  = v - (w * TRACK_WIDTH_M / 2.0f);
  float vRight = v + (w * TRACK_WIDTH_M / 2.0f);

  dirLeft  = (vLeft  < 0) ? -1 : 1;
  dirRight = (vRight < 0) ? -1 : 1;

  Motors::drive(speedToDuty(vLeft), speedToDuty(vRight));
  lastCmdMs = millis();
}

// Integrate wheel travel into a pose and publish it.
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  uint32_t dL, dR;
  Encoders::takeDelta(dL, dR);

  // Sign the raw counts by the direction we last commanded. Single-channel
  // sensors cannot tell direction themselves, so this is the best available
  // answer — correct unless something drags a wheel against the motor.
  totalTicksL += (int32_t)dL * dirLeft;
  totalTicksR += (int32_t)dR * dirRight;

  ticks_l_msg.data = totalTicksL;
  ticks_r_msg.data = totalTicksR;
  RCSOFTCHECK(rcl_publish(&ticks_l_pub, &ticks_l_msg, NULL));
  RCSOFTCHECK(rcl_publish(&ticks_r_pub, &ticks_r_msg, NULL));

  unsigned long now = millis();
  float dt = (now - lastOdomMs) / 1000.0f;
  lastOdomMs = now;
  if (dt <= 0.0f) return;

  const float circumference = WHEEL_DIAMETER_M * PI;
  const float metresPerTick = circumference / TICKS_PER_REV;

  float distL = dL * metresPerTick * dirLeft;
  float distR = dR * metresPerTick * dirRight;

  float dCentre = (distL + distR) / 2.0f;
  float dTheta  = (distR - distL) / TRACK_WIDTH_M;

  // Integrate about the midpoint of the arc rather than its start — markedly
  // better than the naive version when turning.
  pose_x     += dCentre * cos(pose_theta + dTheta / 2.0);
  pose_y     += dCentre * sin(pose_theta + dTheta / 2.0);
  pose_theta += dTheta;

  // Keep theta in -pi..pi so it never drifts into float mush.
  while (pose_theta >  PI) pose_theta -= 2.0 * PI;
  while (pose_theta < -PI) pose_theta += 2.0 * PI;

  // Timestamp from the agent's clock, so the Pi's TF tree lines up.
  int64_t nanos = rmw_uros_epoch_nanos();
  odom_msg.header.stamp.sec     = (int32_t)(nanos / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);

  odom_msg.pose.pose.position.x = pose_x;
  odom_msg.pose.pose.position.y = pose_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Yaw as a quaternion. Rotation about Z only, so x and y stay zero.
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(pose_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(pose_theta / 2.0);

  odom_msg.twist.twist.linear.x  = dCentre / dt;
  odom_msg.twist.twist.angular.z = dTheta / dt;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

// ------------------------------------------------------------ ROS setup

bool createEntities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_base", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_publisher_init_default(
    &ticks_l_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ticks_left"));

  RCCHECK(rclc_publisher_init_default(
    &ticks_r_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ticks_right"));

  // _default2 takes an explicit clock. The old rclc_timer_init_default is
  // deprecated in this version and warns at compile time.
  RCCHECK(rclc_timer_init_default2(
    &odom_timer, &support, RCL_MS_TO_NS(ODOM_PERIOD_MS),
    odom_timer_callback, true));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  // Line the ESP32's clock up with the agent's, so odom timestamps make sense
  // to the rest of the ROS graph.
  rmw_uros_sync_session(1000);

  return true;
}

// Tear every entity down so a reconnect starts clean, rather than leaking
// publishers and subscriptions on the agent side each time the link drops.
// Return values are deliberately discarded — if teardown fails there is
// nothing useful left to do about it.
void destroyEntities() {
  (void)rcl_publisher_fini(&odom_pub, &node);
  (void)rcl_publisher_fini(&ticks_l_pub, &node);
  (void)rcl_publisher_fini(&ticks_r_pub, &node);
  (void)rcl_subscription_fini(&cmd_vel_sub, &node);
  (void)rcl_timer_fini(&odom_timer);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

// ------------------------------------------------------------ setup

void setup() {
  // Motors first, so the wheels are dead before anything else can run.
  Motors::begin();
  Encoders::begin();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  set_microros_transports();

  lastCmdMs  = millis();
  lastOdomMs = millis();
}

// ------------------------------------------------------------ loop

void loop() {
  switch (agentState) {

    case WAITING_AGENT:
      // Ping rather than blocking: the rover stays responsive and the motors
      // stay stopped while the agent is missing.
      EVERY_N_MS(500, agentState =
        (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      agentState = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agentState == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EVERY_N_MS(500, agentState =
        (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agentState == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      Motors::stopNow();
      agentState = WAITING_AGENT;
      break;
  }

  // Failsafe. Runs regardless of connection state — if commands stop arriving
  // for any reason, the wheels stop.
  if (agentState != AGENT_CONNECTED || millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    Motors::drive(0, 0);
  }

  // LED on = connected to the agent. The only debugging output available,
  // since the USB port belongs to micro-ROS.
  digitalWrite(PIN_LED, agentState == AGENT_CONNECTED ? HIGH : LOW);

  Motors::update();
}
