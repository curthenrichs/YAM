/*
 * TODO write this
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

#include <ArduinoJson.h>
#include <elapsedMillis.h>

#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <yam_msgs/RobotState.h>
#include <yam_msgs/CartesianDrive.h>
#include <yam_msgs/DifferentialDrive.h>

//==============================================================================
//                         Constants and Macro Declaration
//==============================================================================

#define WATCHDOG_TIMER_TIME (50)
#define ULTRASONIC_TIMER_TIME (1)
#define ROBOT_STATE_TIME (250)

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

static bool _autonomous_mode = true;
static bool _drivetrain_active = true;
static int _ultrasonic_sensor_state = 0;

static elapsedMillis _watchdog_timeout_timer;
static elapsedMillis _ultrasonic_sensor_timer;
static elapsedMillis _robot_state_timer;

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

static ros::NodeHandler nodeHandler;

static yam_msgs::RobotState robotState_msg;
static ros::Publisher robot_state_pub("hardware/robot_state", &robotState_msg);

static std_msgs::Float32 ultrasonic_msgs[3] = {
  sensor_msgs::Range(),
  sensor_msgs::Range(),
  sensor_msgs::Range()
};
static char* ultrasonic_frames[3] = {
  "ultrasonic_left",
  "ultrasonic_center",
  "ultrasonci_right"
}
static ros::Publisher range_pubs[3] = {
  ros::Publisher(ultrasonic_frames[0],&ultrasonic_msgs[0]),
  ros::Publisher(ultrasonic_frames[1],&ultrasonic_msgs[1]),
  ros::Publisher(ultrasonic_frames[2],&ultrasonic_msgs[2])
}

static ros::Subscriber<std_msgs::Bool> heartbeat_sub("hardware/heartbeat", heartbeat_cb);
static ros::Subscriber<std_msgs::Bool> autonomous_mode_sub("hardware/autonomous_mode", autonomous_mode_cb);
static ros::Subscriber<yam_msgs::DifferentialDrive> differential_drive_sub("hardware/differential_drive", differential_drive_cb);

//==============================================================================
//                                 MAIN
//==============================================================================

/**
 * Main program intialization
 */
void setup(void) {

  watchdog_begin();

  leftMotor.begin();
  rightMotor.begin();
  drive_init(&leftMotor,&rightMotor);
  drive_hard_stop();

  _watchdog_timeout_timer = 0;
  _ultrasonic_sensor_timer = 0;

  nodeHandler.initNode();

  nodeHandler.advertise(robot_state_pub);

  for (int i=0; i<3; i++) {
    ultrasonic_msgs[i].header.seq = 0;
    ultrasonic_msgs[i].header.frame_id = ultrasonic_frames[i];
    ultrasonic_msgs[i].radiation_type = 0;
    ultrasonic_msgs[i].field_of_view = 0.5236f;
    ultrasonic_msgs[i].min_range = 0.03f;
    ultrasonic_msgs[i].max_range = 4.0f;

    nodeHandler.advertise(range_pubs[i]);
  }
}

/**
 * Main program loop
 */
void loop(void) {

  // Update watchdog results
  if (_watchdog_timeout_timer >= WATCHDOG_TIMEOUT_TIMER_TIME) {
    _watchdog_timeout_timer -=  WATCHDOG_TIMEOUT_TIMER_TIME;

    // Handle watchdog to motor lockout
    if(watchdog_is_locked() && _drivetrain_active){
      drive_hard_stop();
      _drivetrain_active = false;
    } else if (!watchdog_is_locked() && !_drivetrain_active) {
      _drivetrain_active = true;
    }
  }

  // scan ultrasonics
  if (_ultrasonic_sensor_timer >= ULTRASONIC_TIMER_TIME) {
    _ultrasonic_sensor_timer -= ULTRASONIC_TIMER_TIME;

    ultrasonicSensors[_ultrasonic_sensor_state].update();

    // Publish Ultrasonic state
    sensor_msgs::Range* range_msg = &ultrasonic_msgs[_ultrasonic_sensor_state];
    range_msg->header->seq += 1;
    range_msg->header->stamp = nodeHandler.now();
    range_msg->range = ultrasonicSensors[_ultrasonic_sensor_state].getDistance();
    range_pubs[_ultrasonic_sensor_state].publish(range_msg);

    _ultrasonic_sensor_state++;
    if (_ultrasonic_sensor_state >= 3) {
      _ultrasonic_sensor_state = 0;
    }
  }

  // publish robot state
  if (_robot_state_timer  >= ROBOT_STATE_TIME) {
    _robot_state_timer -= ROBOT_STATE_TIME;

    voltageMonitor.update();

    robotState_msg.in_autonomous_mode = _autonomous_mode;
    robotState_msg.watchdog_tripped = !_drivetrain_active;
    robotState_msg.voltage = voltageMonitor.getVoltage();
    robotState_msg.robot_power_state = voltageMonitor.getState();

    robot_state_pub.publish(&robotState_msg);
  }

  // Autonomous mode
  if (_autonomous_mode && _drivetrain_active) {
    wallBanger();
  }

  nodeHandler.spinOnce();
  delay(1);
}

//==============================================================================
//                      Private Function Implementation
//==============================================================================

static void wallBanger(void) {

  if (ultrasonicSensors[0].getDistance() < 20 || ultrasonicSensors[1].getDistance() < 20 || ultrasonicSensors[2].getDistance() < 20) {

    // Backup
    if (ultrasonicSensors[0].getDistance() < 10 || ultrasonicSensors[1].getDistance() < 10 || ultrasonicSensors[2].getDistance() < 10) {
      drive_cartesian(0,-50);
      delay(250);
    }

    // Decide direction
    if (ultrasonicSensors[1].getDistance() > 50) {
      if (ultrasonicSensors[0].getDistance() > ultrasonicSensors[2].getDistance()) {
        drive_cartesian(50,50);
      } else {
        drive_cartesian(-50,50);
      }
    } else {
      if (ultrasonicSensors[0].getDistance() > ultrasonicSensors[2].getDistance()) {
        drive_cartesian(50,-10);
      } else {
        drive_cartesian(-50,-10);
      }
    }
  } else {
    drive_cartesian(0,50);
  }
}

static void heartbeat_cb(const std_msgs::Bool &msg) {
  if (msg.data) {
    watchdog_feed();
  }
}

static void autonomous_mode_cb(const std_msgs::Bool &msg) {
  _autonomous_mode = msg.data;
}

static void cartesian_drive_cb(const yam_msgs::CartesianDrive &msg) {
  if (!_autonomous_mode && _drivetrain_active) {
    drive_cartesian(msg.x,msg.y);
  }
}

static void differential_drive_cb(const yam_msgs::DifferentialDrive &msg) {
  if (!_autonomous_mode && _drivetrain_active) {
    drive_differential(msg.left,msg.right);
  }
}
