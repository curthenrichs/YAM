/*
 * TODO write this
 */

// Note need to develop battery interface
// Need to develop JSON serial interface

//==============================================================================
//                               Libraries
//==============================================================================

#include "HardwareConfig.h"
#include "BatteryMonitor.h"
#include "Drivetrain.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "Watchdog.h"

#include <elapsedMillis.h>
#include <ArduinoJson.h>

//==============================================================================
//                         Constants and Macro Declaration
//==============================================================================

#define NUM_ULTRASONIC_SENSORS (3)

#define WATCHDOG_TIMEOUT_TIMER_TIME (50)
#define ULTRASONIC_TIMER_TIME (5)
#define BATTERT_MONITOR_TIME (250)

//==============================================================================
//                           Private Attributes
//==============================================================================

static bool _autonomous_mode = true;
static bool _drivetrain_active = true;
static int _ultrasonic_sensor_state = 0;

static elapsedMillis _watchdog_timeout_timer;
static elapsedMillis _ultrasonic_sensor_timer;
static elapsedMillis _battery_monitor_timer;

static MotorController rightMotor(MOTOR_0_DIR_PIN,MOTOR_0_PWM_PIN);
static MotorController leftMotor(MOTOR_1_DIR_PIN,MOTOR_1_PWM_PIN);

static UltrasonicSensor ultrasonicSensors[3] = {
  UltrasonicSensor(ULTRASONIC_0_PIN),
  UltrasonicSensor(ULTRASONIC_1_PIN),
  UltrasonicSensor(UlTRASONIC_2_PIN)
};

static BatteryMonitor batteryMonitor(BATTERY_VIN_PIN,BATTERY_VIN_SLOPE);

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

  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
  DEBUG_SERIAL.println("Starting Serial Debugger");

  _watchdog_timeout_timer = 0;
  _ultrasonic_sensor_timer = 0;
}

/**
 * Main program loop
 */
void loop(void) {

  // feed watchdog timeout value
  if (_watchdog_timeout_timer >= WATCHDOG_TIMEOUT_TIMER_TIME) {
    _watchdog_timeout_timer -=  WATCHDOG_TIMEOUT_TIMER_TIME;
    watchdog_feed();
  }

  // Handle watchdog to motor lockout
  if(watchdog_is_locked() && _drivetrain_active){
    drive_hard_stop();
    _drivetrain_active = false;
  } else if (!watchdog_is_locked() && !_drivetrain_active) {
    _drivetrain_active = true;
  }

  // Drivetrain control
  if (_drivetrain_active) {
    if (_autonomous_mode) {
      //wallBanger();
      //wallBanger_fancy();
      wallBanger_simple();
      //drive_cartesian(100,0);
    } else {
      // ? TODO
    }
  }

  // scan ultrasonics
  if (_ultrasonic_sensor_timer >= ULTRASONIC_TIMER_TIME) {
    _ultrasonic_sensor_timer -= ULTRASONIC_TIMER_TIME;

    ultrasonicSensors[_ultrasonic_sensor_state].update();

    _ultrasonic_sensor_state++;
    if (_ultrasonic_sensor_state >= NUM_ULTRASONIC_SENSORS) {
      _ultrasonic_sensor_state = 0;
    }
  }

  // scan battery
  if (_battery_monitor_timer >= BATTERT_MONITOR_TIME) {
    _battery_monitor_timer -= BATTERT_MONITOR_TIME;

    batteryMonitor.update();
  }

  delay(1);
}

void wallBanger(void) {
  if (ultrasonicSensors[0].getDistance() < 20 || ultrasonicSensors[1].getDistance() < 20 || ultrasonicSensors[2].getDistance() < 20) {
    // Stop!
    drive_cartesian(0,0);

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

void wallBanger_simple(void) {

  // Pack sensor readings
  int sensorValues[3];
  for (int i=0; i<3; i++) {
    sensorValues[i] = ultrasonicSensors[i].getDistance();
  }

  // Check if possible immediate collision
  bool immediateCollision = false;
  for (int i=0; i<3; i++) {
    immediateCollision = immediateCollision || sensorValues[i] < 5;
  }

  // Act
  if (immediateCollision) {
    drive_cartesian(0,-100);
    delay(100);
    if (sensorValues[0] > sensorValues[2]) {
      drive_cartesian(-100,0);
    } else {
      drive_cartesian(100,0);
    }
    delay(10);
  } else {

    // need to turn
    if (sensorValues[1] < 20) {
      if (sensorValues[0] > sensorValues[2]) {
        drive_cartesian(-100,100);
      } else {
        drive_cartesian(100,100);
      }
    } else {
      drive_cartesian(0,100);
    }
    delay(10);
  }
}

void wallBanger_fancy(void) {
  const int turnBound = 10;
  
  // Pack sensor values
  int sensorValues[5];
  for (int i=1; i<4; i++) {
    sensorValues[i] = ultrasonicSensors[i-1].getDistance();
    if (sensorValues[i] > 80) {
      sensorValues[i] = 80;
      
    }
  }
  sensorValues[0] = turnBound;
  sensorValues[4] = turnBound;

  // Check if possible immediate collision
  bool immediateCollision = false;
  for (int i=0; i<5; i++) {
    immediateCollision = immediateCollision || sensorValues[i] < 5;
  }

  // Act
  if (immediateCollision) {
    drive_cartesian(0,-100);
    delay(100);
  } else {

    // Apply Channel bypass
    double channel = sqrt(sq(-0.707 * sensorValues[1] - 0.707 * sensorValues[3]) 
                        + sq(0.707 * sensorValues[1] - 0.707 * sensorValues[3]));
    if (channel < 8) {
      sensorValues[2] = 0;
    }

    // insert intermediate angles
    int c = 0;
    int searchValues[9];
    int values[9];
    for (int i=0; i<4; i++) {
      searchValues[c++] = sensorValues[i];
      searchValues[c++] = (sensorValues[i] + sensorValues[i+1])/2.0;
    }
    searchValues[c] = sensorValues[4];

    // search for best direction
    int maxIndex = 0;
    int maxValue = 0;
    for (int i=0; i<9; i++) {

      // calculate node value
      int value = searchValues[i];
      if (i > 0) {
        value += searchValues[i-1];
      }
      if (i < 8) {
        value += searchValues[i+1];
      }
      values[i] = value;

      // check if better value
      if (value >= maxValue) {
        maxIndex = i;
        maxValue = value;
      }
    }

    // convert from index into angle and make drive vector
    float theta = (180 - maxIndex * 22.5f) * DEG_TO_RAD;
    int x = 100 * cos(theta);
    int y = 100 * sin(theta);

    DEBUG_SERIAL.print("Sensors: ");
    for(int i=0; i<5; i++) {
      DEBUG_SERIAL.print(sensorValues[i]);
      DEBUG_SERIAL.print(", ");
    }
    DEBUG_SERIAL.println("\n");

    DEBUG_SERIAL.print("Values: ");
    for(int i=0; i<9; i++) {
      DEBUG_SERIAL.print(values[i]);
      DEBUG_SERIAL.print(", ");
    }
    DEBUG_SERIAL.println("\n");

    DEBUG_SERIAL.print("maxIndex: ");
    DEBUG_SERIAL.println(maxIndex);
    
    DEBUG_SERIAL.print("X: ");
    DEBUG_SERIAL.print(x);
    DEBUG_SERIAL.print(" Y: ");
    DEBUG_SERIAL.print(y);
    DEBUG_SERIAL.print("\n");

    drive_cartesian(x,y);
  }
}
