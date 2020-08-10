/**
 * NavigationFirmware
 * @author Curt Henrichs
 * @date 6-22-20
 * 
 * Navigation Firmware is responsible for providing the low-level drivetrain
 * control and ultrasonic sensing for the YAM platform. Communication is done
 * through custom ROS messages over a rosserial bridge. Low-level wallbanger
 * AI is implemented for testing purposes.
 * 
 * This firmware was written specifically for an Arduino Leonardo. Other boards
 * may require modification.
 * 
 * This firmware functions as a timer directed event-loop to update the follwing
 * components.
 *  - Communication Watchdog: tracks heartbeat messages and disables drivetrain
 *                            if messages do not arrive fast enough
 *  - Ultrasonic hardware polling: Performs read operation for each sensor
 *  - Robot state publishing: Publishes Robot State messages to ROS
 *  - Ultrasonic state publishing: Publishes Ultrasonic State messages to ROS
 *  - ROS subscriber handler: Runs all pending callbacks from subscribers
 *  - Autonomous mode: if active, runs the current state of the autonomous state
 *                     machine using current ultrasonic values
 */

//==============================================================================
//                               Libraries
//==============================================================================

#include "HardwareConfig.h"
#include "VoltageMonitor.h"
#include "Drivetrain.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "Watchdog.h"
#include "Autonomous.h"

#include <elapsedMillis.h>

#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include <yam_msgs/RobotState.h>
#include <yam_msgs/Ultrasonics.h>
#include <yam_msgs/CartesianDrive.h>
#include <yam_msgs/DifferentialDrive.h>

//==============================================================================
//                         Constants and Macro Declaration
//==============================================================================

#define WATCHDOG_TIMER_TIME      (50)
#define SENSOR_PUBLISH_TIME      (500)
#define AUTON_UPDATE_TIME        (100)

//==============================================================================
//                      Private Function Prototypes
//==============================================================================

static void wallBanger(void);
static void heartbeat_cb(const std_msgs::Bool &msg);
static void autonomous_mode_cb(const std_msgs::Bool &msg);
static void cartesian_drive_cb(const yam_msgs::CartesianDrive &msg);
static void differential_drive_cb(const yam_msgs::DifferentialDrive &msg);

//==============================================================================
//                           Private Attributes
//==============================================================================

static bool _autonomous_mode = false;
static bool _drivetrain_active = false;

static elapsedMillis _watchdog_timeout_timer;
static elapsedMillis _sensor_publish_timer;
static elapsedMillis _auton_update_timer;

static MotorController rightMotor(MOTOR_0_DIR_PIN,MOTOR_0_PWM_PIN);
static MotorController leftMotor(MOTOR_1_DIR_PIN,MOTOR_1_PWM_PIN);

static UltrasonicSensor ultrasonicSensors[3] = {
  UltrasonicSensor(ULTRASONIC_0_PIN),
  UltrasonicSensor(ULTRASONIC_1_PIN),
  UltrasonicSensor(UlTRASONIC_2_PIN)
};

static VoltageMonitor voltageMonitor(VOLTAGE_VIN_PIN,VOLTAGE_VIN_SLOPE,VOLTAGE_VIN_INTERCEPT);

//==============================================================================
//                             ROS Interface
//==============================================================================

static ros::NodeHandle nodeHandler;

static yam_msgs::RobotState robotState_msg;
static ros::Publisher robot_state_pub("rs", &robotState_msg);

static yam_msgs::Ultrasonics ultrasonics_msg;
static ros::Publisher ultrasonics_pub("us", &ultrasonics_msg);

static ros::Subscriber<std_msgs::Bool> heartbeat_sub("hb", heartbeat_cb);
static ros::Subscriber<std_msgs::Bool> autonomous_mode_sub("ai", autonomous_mode_cb);
static ros::Subscriber<yam_msgs::CartesianDrive> cartesian_drive_sub("cd", cartesian_drive_cb); 
static ros::Subscriber<yam_msgs::DifferentialDrive> differential_drive_sub("dd", differential_drive_cb);

//==============================================================================
//                                 MAIN
//==============================================================================

/**
 * Main program intialization
 */
void setup(void) {

  /* Initialize Hardware */
  
  //watchdog_begin();
  leftMotor.begin();
  rightMotor.begin();
  drive_init(&leftMotor,&rightMotor);
  drive_hard_stop();
  auton_begin();

  /* Initialize ROS */

  nodeHandler.initNode();

  nodeHandler.advertise(robot_state_pub);
  nodeHandler.advertise(ultrasonics_pub);
  nodeHandler.subscribe(heartbeat_sub);
  nodeHandler.subscribe(autonomous_mode_sub);
  nodeHandler.subscribe(cartesian_drive_sub);
  nodeHandler.subscribe(differential_drive_sub);

  ultrasonics_msg.field_of_view = 0.5236f;
  ultrasonics_msg.min_range = 0.03f;
  ultrasonics_msg.max_range = 4.0f;
  
  /* Initialize Timers */
  _watchdog_timeout_timer = 0;
  _sensor_publish_timer = 0;
  _auton_update_timer = 0;
}

/**
 * Main program loop
 */
void loop(void) {

  // Update watchdog results
  if (_watchdog_timeout_timer >= WATCHDOG_TIMER_TIME) {
    _watchdog_timeout_timer -=  WATCHDOG_TIMER_TIME;

    // Handle watchdog to motor lockout
    if(watchdog_is_locked() && _drivetrain_active){
      drive_hard_stop();
      _drivetrain_active = false;
    } else if (!watchdog_is_locked() && !_drivetrain_active) {
      _drivetrain_active = true;
    }
  }

  /* Update sensors */
  if (_sensor_publish_timer >= SENSOR_PUBLISH_TIME) {
    _sensor_publish_timer = 0;

    // Ultrasonic Sensors
    if (!_autonomous_mode) {
      ultrasonicSensors[0].update();
      ultrasonicSensors[1].update();
      ultrasonicSensors[2].update();
    }
    
    ultrasonics_msg.left_range = ultrasonicSensors[0].getDistance();
    ultrasonics_msg.center_range = ultrasonicSensors[1].getDistance();
    ultrasonics_msg.right_range = ultrasonicSensors[2].getDistance();
    ultrasonics_pub.publish(&ultrasonics_msg);

    // Robot State
    voltageMonitor.update();

    robotState_msg.in_autonomous_mode = _autonomous_mode;
    robotState_msg.watchdog_tripped = !_drivetrain_active;
    robotState_msg.voltage = voltageMonitor.getVoltage();
    robotState_msg.robot_power_state = voltageMonitor.getState();
    robotState_msg.auton_state = auton_get_state();
    robot_state_pub.publish(&robotState_msg);
  }

  // Autonomous mode
  if (_autonomous_mode && _drivetrain_active) {
    wallBanger();
  }

  nodeHandler.spinOnce();
  delay(10);
}

//==============================================================================
//                      Private Function Implementation
//==============================================================================

static void wallBanger(void) {

  if (_auton_update_timer >= AUTON_UPDATE_TIME) {
    _auton_update_timer = 0;

    // Get Sensor State
    ultrasonicSensors[0].update();
    ultrasonicSensors[1].update();
    ultrasonicSensors[2].update();

    // Run state machine
    auton_update(ultrasonicSensors[0].getDistance(),
                 ultrasonicSensors[1].getDistance(),
                 ultrasonicSensors[2].getDistance());

    // Send motor commands
    int l = auton_left_motor();
    int r = auton_right_motor();
    drive_differential(l, r, false);
  }
}

static void heartbeat_cb(const std_msgs::Bool &msg) {
  if (msg.data) {
    watchdog_feed();
  }
}

static void autonomous_mode_cb(const std_msgs::Bool &msg) {
  _autonomous_mode = msg.data;
  _auton_update_timer = 0;
  auton_begin();
}

static void cartesian_drive_cb(const yam_msgs::CartesianDrive &msg) {
  if (!_autonomous_mode && _drivetrain_active) {
    drive_cartesian(msg.x,msg.y);
  }
}

static void differential_drive_cb(const yam_msgs::DifferentialDrive &msg) {
  if (!_autonomous_mode && _drivetrain_active) {
    drive_differential(msg.left, msg.right, false);
  }
}
