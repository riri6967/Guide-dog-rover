/*
 * rover_microros.ino — the rover as a ROS 2 node
 *
 * Board:   ESP32 Dev Module, Arduino core 2.0.2
 * Library: micro_ros_arduino v2.0.8-jazzy
 * Agent:   Raspberry Pi 5, ROS 2 Jazzy, over USB serial
 *
 * Subscribes  /cmd_vel       geometry_msgs/msg/Twist
 * Publishes   /ticks_left    std_msgs/msg/Int32   at 20 Hz, cumulative, signed
 *             /ticks_right   std_msgs/msg/Int32   at 20 Hz
 *
 * Odometry is computed on the Pi (ros/rover_bringup, wheel_odometry) from the
 * tick counts, not here: INTEGRATION.md decision 2.
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
const float TICKS_PER_REV = 140.0f;        // MEASURED: 1392/1402 pulses over 10 wheel turns

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

// Closed-loop wheel speed (PI controller on the encoder ticks). OFF until the
// 100 nF motor capacitors are fitted: motor EMI adds false encoder pulses, the
// controller reads them as extra speed and throttles the motors down. With it
// off, /cmd_vel maps straight to PWM as before.
const bool  USE_SPEED_PID = false;
const float PID_KP = 300.0f;        // duty per m/s of speed error
const float PID_KI = 600.0f;        // duty per (m/s x s) of accumulated error
const float SPEED_FILTER = 0.5f;    // weight of the newest speed sample, 0..1

// Commands slower than this are treated as stop. Below it the motor would
// only run at its minimum speed anyway, which is faster than asked.
const float MIN_WHEEL_SPEED_MS = 0.02f;

// ===================================================================

// Motors stop if no /cmd_vel arrives within this window. Every way of losing
// control — agent killed, cable pulled, Pi crashed, teleop closed — looks the
// same from here: commands stop. Silence means stop, never "carry on".
const unsigned long CMD_TIMEOUT_MS = 900;

const unsigned long TICK_PERIOD_MS = 50;    // 20 Hz: tick publishing and the speed loop

// ------------------------------------------------------------ ROS entities
rcl_subscription_t   cmd_vel_sub;
geometry_msgs__msg__Twist    cmd_vel_msg;

// Raw cumulative encoder counts. These are the unprocessed truth: they need no
// calibration to be useful, and the Pi computes odometry from them.
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
rcl_timer_t       tick_timer;

// ------------------------------------------------------------ state
unsigned long lastCmdMs = 0;

// Speed loop state, per side: target and filtered measured speed in m/s, and
// the PI controller's accumulated error.
float targetL = 0.0f, targetR = 0.0f;
float measL = 0.0f,   measR = 0.0f;
float integL = 0.0f,  integR = 0.0f;

// Single-channel encoders cannot sense direction, so we remember which way we
// last told each side to turn and sign the ticks with that.
int dirLeft  = 1;
int dirRight = 1;

unsigned long lastTickMs = 0;

// Agent connection state machine. The stock examples give up permanently when
// the agent is missing; a robot must reconnect instead.
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState agentState = WAITING_AGENT;

// ------------------------------------------------------------ helpers

#define RCCHECK(fn)      { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn)  { rcl_ret_t rc = fn; (void)rc; }

#define EVERY_N_MS(ms, body) { \
  static volatile unsigned long _last = 0; \
  if (millis() - _last >= (ms)) { body; _last = millis(); } }

// Convert a wheel speed in m/s to an L298N duty, -255..255. The range starts
// at the driver's deadband instead of 0, so a slow command still turns the
// wheels instead of vanishing below Motors::DEADBAND (the L298N's ~23 %).
float speedToDuty(float metresPerSec) {
  float speed = fabsf(metresPerSec);
  if (speed < MIN_WHEEL_SPEED_MS) return 0.0f;
  float ratio = constrain(speed / MAX_WHEEL_SPEED_MS, 0.0f, 1.0f);
  float duty = Motors::DEADBAND + ratio * (Motors::MAX_SPEED - Motors::DEADBAND);
  return metresPerSec < 0 ? -duty : duty;
}

// One PI step for one side. speedToDuty() is the feed-forward; the controller
// only corrects what it gets wrong (load, battery voltage, a stiffer motor).
int speedControl(float target, float measured, float &integ, float dt) {
  if (fabsf(target) < MIN_WHEEL_SPEED_MS) { integ = 0.0f; return 0; }
  float err = target - measured;
  float out = speedToDuty(target) + PID_KP * err + PID_KI * integ;
  // Anti-windup: stop accumulating while the output is pinned at full scale,
  // unless the error is pulling it back.
  bool saturated = fabsf(out) >= Motors::MAX_SPEED;
  if (!saturated || (out > 0) != (err > 0)) integ += err * dt;
  out = speedToDuty(target) + PID_KP * err + PID_KI * integ;
  // Never drive against the commanded direction: single-channel encoders
  // cannot see which way the wheel turns, so a reversed wheel looks forward.
  if ((target > 0 && out < 0) || (target < 0 && out > 0)) out = 0;
  return constrain((int)out, -Motors::MAX_SPEED, Motors::MAX_SPEED);
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

  // A change of direction starts the speed loop afresh on that side.
  if ((vLeft  < 0) != (targetL < 0)) { integL = 0.0f; measL = 0.0f; }
  if ((vRight < 0) != (targetR < 0)) { integR = 0.0f; measR = 0.0f; }
  targetL = vLeft;
  targetR = vRight;

  dirLeft  = (vLeft  < 0) ? -1 : 1;
  dirRight = (vRight < 0) ? -1 : 1;

  if (!USE_SPEED_PID) {
    Motors::drive((int)speedToDuty(vLeft), (int)speedToDuty(vRight));
  }
  lastCmdMs = millis();
}

// Publish the tick counts and, with USE_SPEED_PID, run the speed loop.
void tick_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
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
  float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;
  if (!USE_SPEED_PID || dt <= 0.0f) return;

  // Silence means stop: the same timeout as the failsafe in loop(), so the
  // speed loop cannot keep a wheel turning after commands stop.
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    targetL = targetR = 0.0f;
  }

  const float metresPerTick = WHEEL_DIAMETER_M * PI / TICKS_PER_REV;
  measL += SPEED_FILTER * (dL * metresPerTick / dt * dirLeft  - measL);
  measR += SPEED_FILTER * (dR * metresPerTick / dt * dirRight - measR);

  Motors::drive(speedControl(targetL, measL, integL, dt),
                speedControl(targetR, measR, integR, dt));
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
    &tick_timer, &support, RCL_MS_TO_NS(TICK_PERIOD_MS),
    tick_timer_callback, true));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &tick_timer));

  return true;
}

// Tear every entity down so a reconnect starts clean, rather than leaking
// publishers and subscriptions on the agent side each time the link drops.
// Return values are deliberately discarded — if teardown fails there is
// nothing useful left to do about it.
void destroyEntities() {
  (void)rcl_publisher_fini(&ticks_l_pub, &node);
  (void)rcl_publisher_fini(&ticks_r_pub, &node);
  (void)rcl_subscription_fini(&cmd_vel_sub, &node);
  (void)rcl_timer_fini(&tick_timer);
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
  lastTickMs = millis();
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
