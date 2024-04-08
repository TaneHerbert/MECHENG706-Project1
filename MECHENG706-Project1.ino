/**
 * Our Code
 *
 * Authors:
 *  Tane Herbert
 *  Juaninho Penteado
 *  Danny Su
 *  Jackson Taylor
*/

#include <SoftwareSerial.h>
#include <Servo.h>

/**
  * Public Defines
  */

#define NO_BATTERY_V_OK // Uncomment of BATTERY_V_OK if you do not care about battery damage.

#define STARTUP_DELAY 1 // Seconds

#define MAX_SONARDIST_CM 200
#define MIN_SONARDIST_CM 2

/**
 * Public pins
 */

#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin

// Default ultrasonic ranging sensor pins
const int TRIG_PIN = 42;
const int ECHO_PIN = 43;

//Define pins
const int gyroSensorPin = A15;

// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 48;
const byte right_front = 49;

/** 
  * Public Types
  */

//State machine states
enum STATE 
{
  INITIALISING,
  FINDCORNER,
  STRAIGHT,
  RUNNING,
  STOPPED
};

enum IRSENSORTYPE
{
  SHORTRANGE,
  LONGRANGE
};

enum PIDCONTROL
{
  XCONTROL,
  YCONTROL,
  ACONTROL
};

struct nonBlockingTimers
{
  unsigned long lastUpdateTime;
};

struct IRSensor
{
  const int IR_PIN;
  IRSENSORTYPE mSensorType;
  bool isTooFar;
  bool isTooClose;
  bool isInRange;
  float lowerVoltage;
  float upperVoltage;
};

//PID Control System Global Vars
//declare pid struct
struct pidvars 
{
  PIDCONTROL mPIDCONTROL;
  float eprev;
  float eintegral;
  float integralLimit;
  float kp;
  float ki;
  float kd;
  unsigned long prevT;
  unsigned long breakOutTime;
  unsigned long prevBreakOutTime;
  bool withinError;
  float minError;
};

/**
  * Public Variables
  */

// Global Coordinate based variables:
float xCoordinate;
float yCoordinate;
int wallDirection; // (0 = starts in TOP LEFT/BOTTOM RIGHT), (1 = starts in TOP RIGHT/BOTTOM LEFT)

// Time of one loop, 0.07 s (GYRO)
int T = 70;

// Voltage when gyro is initialised
float gyroZeroVoltage = 0;

float gyroSupplyVoltage = 5;   // supply voltage for gyro
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 3; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored

// current angle calculated by angular velocity integral on
float currentAngle = 0;
float prevAngle = 0;
int fullTurns = 0;  // Counter for full turns. Positive for clockwise, negative for counterclockwise

float distances[200] = { 0 };
float angles[200] = { 0 };

float angleCorner[200] = { 0 };
float cornerDistances[200] = { 0 };

int cornerIndex = 0;
int indexForDistances = 0;;

// This variable will track the cumulative change in angle.
float cumulativeAngleChange = 0;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

// Motor Speed
int speed_val = 100;
int speed_change;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

nonBlockingTimers mNonBlockingTimerGyro = 
{
  .lastUpdateTime = 0,
};

nonBlockingTimers mNonBlockingTimerPID = 
{
  .lastUpdateTime = 0,
};

static STATE machine_state;

IRSensor IR_FL = 
{ 
  .IR_PIN = A5, 
  .mSensorType = SHORTRANGE, 
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.30, 
  .upperVoltage = 2.20,
};

IRSensor IR_BL = 
{ 
  .IR_PIN = A7, 
  .mSensorType = LONGRANGE,
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.51,
  .upperVoltage = 2.00
};

IRSensor IR_FR = 
{ 
  .IR_PIN = A9, 
  .mSensorType = LONGRANGE,
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.46,
  .upperVoltage = 2.00
};

IRSensor IR_BR = 
{ 
  .IR_PIN = A11, 
  .mSensorType = SHORTRANGE,
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.31,
  .upperVoltage = 2.30
};

//x coord PID variables
pidvars xVar = 
{
  .mPIDCONTROL = XCONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 1000,
  .kp = 3.38,
  .ki = 0.154, // 0.154
  .kd = 1.02,  // 1.02
  .prevT = 0,
  .breakOutTime = 100,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 100,
};

//y coord PID variables
pidvars yVar = 
{
  .mPIDCONTROL = YCONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 1000,
  .kp = 1.96,
  .ki = 0.205, // 0.205
  .kd = 1.04,  // 1.04
  .prevT = 0,
  .breakOutTime = 100,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 100,
};

//angular PID variables
pidvars aVar = 
{
  .mPIDCONTROL = ACONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 1000, // ????
  .kp = 2.72,
  .ki = 0.343, // 0.343
  .kd = 1.21, // 1.21
  .prevT = 0, 
  .breakOutTime = 500,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 100,
};

//Initialise matrix for inverse kinematics:
float invKMatrix[4][3] =  
{ 
  {1,  1, -165}, 
  {1, -1,  165}, 
  {1, -1, -165}, 
  {1,  1,  165} 
};

float velArray[3];
float angVelArray[4];

/**
 * Private Decleration 
 */

// Motor Movements
void disable_motors();
void enable_motors();
void stop();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left();
void strafe_right();
void speed_change_smooth();

// Sonar
float HC_SR04_range();

// Delay
void delaySeconds(int TimedDelaySeconds);
bool nonBlockingDelay(unsigned long *lastMillis, unsigned long delayMicros);

// Builtin LED FLashing
void slow_flash_LED_builtin();
void fast_flash_double_LED_builtin();

// Gyro
void GyroSetup();
void getCurrentAngle();

// Coordinate System
void updateCoordinates();
void setWallDirection();

// State machine
STATE initialising();
STATE running();
STATE stopped();
STATE findCorner();
STATE straight();

// Battery Voltage
boolean is_battery_voltage_OK();

// IR Distance
float getIRDistance(IRSensor* IRSensor);
void printBool(IRSensor mIRSensor);

// Control Sysytem
void inverseKinematics (float Vx, float Vy, float Az);
float pidControl(pidvars* pidName, float error);

/**
 * Set up
 */

void setup(void) 
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  BluetoothSerial.begin(115200);
  BluetoothSerial.println("MECHENG706_CODE");
  BluetoothSerial.println("Setup....");

  machine_state = INITIALISING;

  // settling time but no really needed
  delaySeconds(STARTUP_DELAY);
}

void loop(void)  //main loop
{
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case FINDCORNER:
      machine_state = findCorner();
      break;
    case STRAIGHT:
      machine_state = straight();
      break;
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };

  /**
   * Methods that must run every loop 
   */
  getCurrentAngle(); // This function must run every 70ms so is placed outside the FSM

  #ifndef NO_BATTERY_V_OK
  if (!is_battery_voltage_OK())
  {
    BluetoothSerial.println("BATTERY FAILURE");
    machine_state = STOPPED;
  }
  #endif
}

//----------------------Motor moments------------------------

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

// ----------------------Sonar------------------------
float HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      BluetoothSerial.println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      BluetoothSerial.println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) 
  {
    BluetoothSerial.println("HC-SR04: Out of range");
    return -1;
  }
    
  return cm;
}

// ----------------------Delay------------------------

void delaySeconds(int TimedDelaySeconds)
{
  for (int i = 0; i < TimedDelaySeconds; i++)
  {
    delay(1000);
  }
}

bool nonBlockingDelay(unsigned long *lastMillis, unsigned long delayMicros) {
  unsigned long currentMicros = millis();

  // Check if the current time minus the last recorded time is greater than the delay
  if (currentMicros - *lastMillis >= delayMicros) {
      // Update the last recorded time
      *lastMillis = currentMicros;
      return true;  // Return true if the delay has elapsed
  }
  return false;  // Return false if the delay has not elapsed
}

// ----------------------Builtin LED FLashing------------------------

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

// ----------------------State machine definitions------------------------

  // Inititlisations
STATE initialising() {
  BluetoothSerial.println("INITIALISING....");

  enable_motors();
  GyroSetup();  //Set up starting voltage for gyro
  setWallDirection();

  // TODO: Needs to be changed to just before starting Straight
  xVar.prevT = micros();
  yVar.prevT = micros();
  aVar.prevT = micros();

  currentAngle = 0;

  return STRAIGHT;
}

STATE findCorner() {

  updateCoordinates();

/*
  BluetoothSerial.print("Front Left is too close: ");
  delay(20);
  BluetoothSerial.println(IR_FL.isTooClose);
  delay(20);
  BluetoothSerial.print("Front Left is too far: ");
  delay(20);
  BluetoothSerial.println(IR_FL.isTooFar);
  delay(20);
  BluetoothSerial.print("Front Left is in range: ");
  delay(20);
  BluetoothSerial.println(IR_FL.isInRange);
  delay(20);

  //BluetoothSerial.println("Y-Coordinate (measured from IR):");

  //BluetoothSerial.println(yCoordinate);

  BluetoothSerial.println("***************************************** ");
  delay(20);

  BluetoothSerial.print("Front Right is too close: ");
  delay(20);
  BluetoothSerial.println(IR_FR.isTooClose);
  delay(20);
  BluetoothSerial.print("Front Right is too far: ");
  delay(20);
  BluetoothSerial.println(IR_FR.isTooFar);
  delay(20);
  BluetoothSerial.print("Front Right is in range: ");
  delay(20);
  BluetoothSerial.println(IR_FR.isInRange);delay(20);

  BluetoothSerial.println("***************************************** ");delay(20);

  BluetoothSerial.print("Back left is too close: ");delay(20);
  BluetoothSerial.println(IR_BL.isTooClose);delay(20);
  BluetoothSerial.print("Back left is too far: ");delay(20);
  BluetoothSerial.println(IR_BL.isTooFar);delay(20);
  BluetoothSerial.print("Back left is in range: ");delay(20);
  BluetoothSerial.println(IR_BL.isInRange);delay(20);

  BluetoothSerial.println("***************************************** ");delay(20);

  BluetoothSerial.print("Back Right is too close: ");delay(20);
  BluetoothSerial.println(IR_BR.isTooClose);delay(20);
  BluetoothSerial.print("Back Right is too far: ");delay(20);
  BluetoothSerial.println(IR_BR.isTooFar);delay(20);
  BluetoothSerial.print("Back Right is in range: ");delay(20);
  BluetoothSerial.println(IR_BR.isInRange);delay(20);

  BluetoothSerial.println("***************************************** ");delay(20);
  */

  BluetoothSerial.println("Y-Coordinate (measured from IR):");
  delay(20);
  BluetoothSerial.println(yCoordinate);
  delay(20);
  BluetoothSerial.println("X-Coordinate (measured from SONAR):");
  delay(20);
  BluetoothSerial.println(xCoordinate);
  delay(20);
  BluetoothSerial.println();

  delay(4000);

  //float distance;

  // distance = getIRDistance(&IR_FL);
  // BluetoothSerial.print("Front Left: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(&IR_FR);
  // BluetoothSerial.print("Front Right: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(&IR_BL);
  // BluetoothSerial.print("Back Left: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(&IR_BR);
  // BluetoothSerial.print("Back Right: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // delay(10000);delay(20);
  // BluetoothSerial.println();delay(20);

  return FINDCORNER;

  /** HOMING STUFF BELOW (PLEASE LEAVE) **/

  // cw();

  // // Continue to turn until its done 1 turn;
  // if (abs(fullTurns) < 1)
  // {
  //   if (abs(abs(currentAngle) - abs(prevAngle)) >= 3) // Take a distance measurement for every 3 degrees turned
  //   {
  //     distances[indexForDistances] = HC_SR04_range();
  //     angles[indexForDistances] = currentAngle;

  //     prevAngle = currentAngle;
  //     indexForDistances++;
  //   }
  //   return FINDCORNER;
  // }

  // stop();

  // // Analyze the collected distances to find corners
  // for (int i = 1; i < indexForDistances - 1; i++) // Start from 1 and end at 78 to avoid out of bound indexes
  // {
  //   if (distances[i] > MAX_SONARDIST_CM || distances[i] < MIN_SONARDIST_CM)
  //     continue; // Skip invalid readings;

  //   // Check for a significant change in distance indicating a corner
  //   // Introduce a threshold (e.g., deltaThreshold) to define what constitutes a significant change
  //   float deltaThreshold = 2; // Adjust based on your robot's environment and sensor
  //   bool isCorner = ((distances[i] - distances[i + 1]) > deltaThreshold) && 
  //                   ((distances[i] - distances[i - 1]) > deltaThreshold);
    
  //   if (isCorner)
  //   {
  //     cornerDistances[cornerIndex] = distances[i - 1];
  //     angleCorner[cornerIndex] = angles[i - 1];
  //     cornerIndex++;
  //     cornerDistances[cornerIndex] = distances[i];
  //     angleCorner[cornerIndex] = angles[i];
  //     cornerIndex++;
  //     cornerDistances[cornerIndex] = distances[i + 1];
  //     angleCorner[cornerIndex] = angles[i + 1];
  //     cornerIndex++;

  //     if (cornerIndex >= 24) 
  //       break;
  //   }
  // }

  // // Output the detected corners
  // BluetoothSerial.println("Corners/Angles");
  // BluetoothSerial.println("");

  // for (int i = 0; i < cornerIndex; i++)
  // {
  //   BluetoothSerial.print("Before Corner Distance: ");
  //   BluetoothSerial.println(cornerDistances[i]);
  //   BluetoothSerial.print("Before Corner Angle: ");
  //   BluetoothSerial.println(angleCorner[i]);
  //   i++;
  //   BluetoothSerial.print("Middle Distance: ");
  //   BluetoothSerial.println(cornerDistances[i]);
  //   BluetoothSerial.print("Middle Angle: ");
  //   BluetoothSerial.println(angleCorner[i]);
  //   i++;
  //   BluetoothSerial.print("After Distance: ");
  //   BluetoothSerial.println(cornerDistances[i]);
  //   BluetoothSerial.print("After Angle: ");
  //   BluetoothSerial.println(angleCorner[i]);
  //   BluetoothSerial.println("");
  // }

  // BluetoothSerial.print("Amount of Corners Indicated: ");
  // BluetoothSerial.println((cornerIndex + 1) /3);
  // BluetoothSerial.print("Distances/Angles Measured:");
  // BluetoothSerial.println(indexForDistances);
  // BluetoothSerial.println("End");

  // return RUNNING;
}

STATE straight()
{
  if (!nonBlockingDelay(&mNonBlockingTimerPID.lastUpdateTime, 50)) // Run straight every 50 ms
  {
    return STRAIGHT;
  }

  float xDesiredPoisition = 1500; // LONG DISTANCE
  float yDesiredPosition = 210;  // SHORT DISTANCE

  updateCoordinates();

  float xError = xDesiredPoisition - xCoordinate;
  float yError = yDesiredPosition - yCoordinate;

  float angleError;
  if (currentAngle <= 180) {
    angleError = currentAngle; // Moving counterclockwise to reach 0
  } else {
    angleError = currentAngle - 360; // Moving clockwise to reach 0
  }

  // BluetoothSerial.print("X-Error: ");
  // delay(20);
  // BluetoothSerial.println(xError);
  // delay(20);
  // BluetoothSerial.print("Y-Error: ");
  // delay(20);
  // BluetoothSerial.println(yError);
  // delay(20);
  // BluetoothSerial.print("Angle-error: ");
  // delay(20);
  // BluetoothSerial.println(angleError);
  // delay(20);
  // BluetoothSerial.println();

  float xVelocity = pidControl(&xVar, xError);
  float yVelocity = pidControl(&yVar, yError);
  float aVelocity = pidControl(&aVar, angleError);

  if (xVar.withinError == true && yVar.withinError == true && aVar.withinError == true)
  {
    int check = 0;

    if (nonBlockingDelay(&xVar.breakOutTime, xVar.breakOutTime))
    {
      check++;
    }

    if (nonBlockingDelay(&yVar.breakOutTime, yVar.breakOutTime))
    {
      check++;
    }

    if (nonBlockingDelay(&aVar.breakOutTime, aVar.breakOutTime))
    {
      check++;
    }

    if (check == 3)
    {
      return STOPPED;
    }
  }

  // BluetoothSerial.print("X-Velocity: ");
  // delay(20);
  // BluetoothSerial.println(xVelocity);
  // delay(20);
  // BluetoothSerial.print("Y-Velocity: ");
  // delay(20);
  // BluetoothSerial.println(yVelocity);
  // delay(20);
  // BluetoothSerial.print("Z-Velocity: ");
  // delay(20);
  // BluetoothSerial.println(aVelocity);
  // delay(20);

  inverseKinematics(xVelocity, yVelocity, aVelocity);

  // BluetoothSerial.print("Motor 1: ");
  // delay(20);
  // BluetoothSerial.println(angVelArray[0]);
  // delay(20);
  // BluetoothSerial.print("Motor 2: ");
  // delay(20);
  // BluetoothSerial.println(-1 * angVelArray[1]);
  // delay(20);
  // BluetoothSerial.print("Motor 3: ");
  // delay(20);
  // BluetoothSerial.println(angVelArray[2]);
  // delay(20);
  // BluetoothSerial.print("Motor 4: ");
  // delay(20);
  // BluetoothSerial.println(-1 * angVelArray[3]);
  // delay(20);

  if (wallDirection == 0)
  {
    left_font_motor.writeMicroseconds(1500 + angVelArray[0]);
    right_font_motor.writeMicroseconds(1500 - angVelArray[1]);
    left_rear_motor.writeMicroseconds(1500 + angVelArray[2]);
    right_rear_motor.writeMicroseconds(1500 - angVelArray[3]);
  }
  else // I think opposite
  {
    left_font_motor.writeMicroseconds(1500 - angVelArray[0]);
    right_font_motor.writeMicroseconds(1500 + angVelArray[1]);
    left_rear_motor.writeMicroseconds(1500 - angVelArray[2]);
    right_rear_motor.writeMicroseconds(1500 + angVelArray[3]);
  }

  angVelArray[0] = 0.0;
  angVelArray[1] = 0.0;
  angVelArray[2] = 0.0;
  angVelArray[3] = 0.0;

  return STRAIGHT;
}

STATE running() {
  fast_flash_double_LED_builtin();

  return RUNNING;
}

STATE stopped() {
  stop();
  disable_motors();
  slow_flash_LED_builtin();

  BluetoothSerial.println("STOPPED HA"); 

  while(1)
  {

  }

  return STOPPED;
}

// ----------------------Battery Checker------------------------

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    BluetoothSerial.print("Lipo level:");
    BluetoothSerial.print(Lipo_level_cal);
    BluetoothSerial.print("%");
    BluetoothSerial.println("");
    
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      BluetoothSerial.println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      BluetoothSerial.println("!Lipo is Overchanged!!!");
    else {
      BluetoothSerial.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      BluetoothSerial.print("Please Re-charge Lipo:");
      BluetoothSerial.print(Lipo_level_cal);
      BluetoothSerial.println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

// ----------------------Gyro------------------------

void GyroSetup() 
{
  // this section is initialize the sensor, find the value of voltage when gyro is zero
  float sum = 0;
  int sensorValue = 0;  // read out value of sensor

  pinMode(gyroSensorPin, INPUT);
  // BluetoothSerial.println("please keep the sensor still for calibration");
  // BluetoothSerial.println("get the gyro zero voltage");
  for (int i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(gyroSensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
}

// TODO: ALWAYS THINKS T = 100 when in fact could be slightly above or below. Need to get actual difference from non blocking delay
void getCurrentAngle() 
{
  if (!nonBlockingDelay(&mNonBlockingTimerGyro.lastUpdateTime, T))
  {
    return;
  }

  // convert the 0-1023 signal to 0-5v
  float gyroRate = (analogRead(gyroSensorPin) * gyroSupplyVoltage) / 1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;  // from Data Sheet, gyroSensitivity is 0.007 V/dps
  // if the angular velocity is less than the threshold, ignore it
  if (abs(angularVelocity) >= rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;

    // Accumulate the angle change
    cumulativeAngleChange += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }

  // Check for full turns
  if (cumulativeAngleChange >= 360) {
    fullTurns++;
    cumulativeAngleChange -= 360; // Reset the cumulative change after counting a turn
  } else if (cumulativeAngleChange <= -360) {
    fullTurns--;
    cumulativeAngleChange += 360; // Reset the cumulative change after counting a turn
  }
}

// ----------------------Coordinate System------------------------

// Update the X, Y, Z coordinates relative to starting point being (~15, ~15, 0°). Starting point once in corner.
void updateCoordinates()
{
  /* General rules of thumb and key information:
    * If AND ONLY if the robot is correctly alligned to be in parallel with the wall, 
    * sonar reading will control X reading and IR sensors will control Y readings
    * If the robot is not correctly alligned with the wall, we will get inaccurate coordinates based on this code
    * The board is 2500mm long and 1200mm 
    * If the robot is 400mm away from the wall in the Y direction, rely on both long ranges to give reading
    * Hypothetically if alligned correctly, the 2 long range IR sensors should add to 1200mm once in the middle
    */

  float backLeftDistance = getIRDistance(&IR_BL);
  float backRightDistance = getIRDistance(&IR_BR);
  float frontRightDistance = getIRDistance(&IR_FR);
  float frontLeftDistance = getIRDistance(&IR_FL);

  // FOR LEFT SIDE TOO CLOSE LESS THAN 40mm
  if ((IR_FL.isTooClose == true) && (IR_BL.isTooClose == true) && (IR_FR.isTooFar == true) && (IR_BR.isTooFar == true)){
    if (wallDirection == 0){ // left side closer to start
      yCoordinate = 72.0;
    }
    else { // right side closer to finish
      yCoordinate = (1200.0 - 72.0);
  }
  }
  //FOR LEFT SIDE  VERY CLOSE, LESS THAN 100mm
  else if ((IR_FL.isInRange) && (IR_BL.isTooClose) && (IR_FR.isTooFar) && (IR_BR.isTooFar)){
    if (wallDirection == 0){ // left side closer to start
      yCoordinate = (frontLeftDistance + 72.0);
    }
    else { // right side closer to finish
      yCoordinate = (1200.0 - 72.0 - frontLeftDistance);
  }
  }

  //FOR LEFT SIDE MEDIUM CLOSE, MORE THAN 100mm, LESS THAN 290mm
  else if ((IR_FL.isInRange) && (IR_BL.isInRange) && (IR_FR.isTooFar) && (IR_BR.isTooFar)){
    //CAN ADD FUNCTIONALITY TO AVERAGE BOTH SENSORS HERE IF NEED BE.
    if (wallDirection == 0){ // left side closer to start
      yCoordinate = (frontLeftDistance + 72.0);
    }
    else { // right side closer to finish
      yCoordinate = (1200.0 - 72.0 - frontLeftDistance);
  }
  }

  //FOR RIGHT SIDE MEDIUM CLOSE, MORE THAN 100mm, LESS THAN 290mm
  else if ((IR_FR.isInRange) && (IR_BR.isInRange) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    //CAN ADD FUNCTIONALITY TO AVERAGE BOTH SENSORS HERE IF NEED BE.
    if (wallDirection == 1){ // right side closer to start
      yCoordinate = (backRightDistance + 72.0);
    }
    else { // left side closer to finish
      yCoordinate = (1200.0 - 72.0 - backRightDistance);
  }
  }

  //FOR RIGHT SIDE VERY CLOSE, LESS THAN 100mm
  else if ((IR_FR.isTooClose) && (IR_BR.isInRange) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    //CAN ADD FUNCTIONALITY TO AVERAGE BOTH SENSORS HERE IF NEED BE.
    if (wallDirection == 1){ // right side closer to start
      yCoordinate = (backRightDistance + 72.0);
    }
    else { // left side closer to finish
      yCoordinate = (1200.0 - 72.0 - backRightDistance);
  }
  }

  //FOR RIGHT SIDE TOO CLOSE, LESS THAN 40mm
  else if ((IR_FR.isTooClose) && (IR_BR.isTooClose) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    //CAN ADD FUNCTIONALITY TO AVERAGE BOTH SENSORS HERE IF NEED BE.
    if (wallDirection == 1){ // right side closer to start
      yCoordinate = 72.0;
    }
    else { // left side closer to finish
      yCoordinate = 1200.0 - 72.0;
  }
  } 

    //FOR IN MIDDLE, MORE THAN 290mm on both sides
  else if ((IR_FL.isTooFar) && (IR_FR.isInRange || IR_BL.isInRange) && (IR_BR.isTooFar)){
    if (frontRightDistance <= 540){
      //use frontright sensor only
      if (wallDirection == 1){ // right side closer to start
        yCoordinate = (frontRightDistance + 72.0);
      }
      else { // left side closer to finish
        yCoordinate = (1200.0 - frontRightDistance - 72.0);
      }
    }

    //If its closer to the left side
    else if (backLeftDistance <= 540){
      //use backleft sensor only
      if (wallDirection == 0){ // left side closer to start
        yCoordinate = (backLeftDistance + 72.0);
      }
      else { // right side closer to finish
        yCoordinate = (1200.0 - backLeftDistance - 72.0);
      }
    }    

    else{
      //dont update the coordinates.
    }
  }

  //FOR WHEN NO CASES WORKING
  else{
    //print to let them know
    BluetoothSerial.println("Error, no cases fit");
    // BluetoothSerial.println("BACK LEFT");
    // printBool(IR_BL);
    // BluetoothSerial.println("BACK RIGHT");
    // printBool(IR_BR);
    // BluetoothSerial.println("FRONT LEFT");
    // printBool(IR_FL);
    // BluetoothSerial.println("FRONT RIGHT");
    // printBool(IR_FR);
    //dont update y coordinate.
  }

  xCoordinate = 2000 - ((10 * HC_SR04_range()) + 122);
}
// ----------------------Control System------------------------

// Call this function once at the beginning after robot is alligned to define what side starts closer to the wall
void setWallDirection()
{
  // If left side starts closer to the wall, we are either in TOP LEFT or BOTTOM RIGHT
  // If right side starts closer to the wall, we are either in TOP RIGHT of BOTTOM LEFT
  (void)getIRDistance(&IR_BR);
  (void)getIRDistance(&IR_FL);
  
  // If left side closer to wall, set wallDirection to 0
  if ((IR_BL.isTooFar == false) || (IR_FL.isTooFar == false)){
    wallDirection = 0;
    // BluetoothSerial.println("Starting point: Top Left or Bottom Right");
  }
  // Else, right side is closer to wall therefore set wallDirection to 1
  else{
    wallDirection = 1;
    // BluetoothSerial.println("Starting point: Top Right or Bottom Left");
  }
}

//Function for inverse kinematics. Input velocities, output angular velocity of each wheel.
void inverseKinematics (float Vx, float Vy, float Az){
  float radius = 27;

  //Input values into the velocity matrix
  velArray[0] = Vx;
  velArray[1] = Vy;
  velArray[2] = Az;

  // Multiplying matrix a and b and storing in array mult.
  for(int i = 0; i < 4; ++i){
    for(int k = 0; k < 3; ++k){
      angVelArray[i] += invKMatrix[i][k] * velArray[k];
    }
    angVelArray[i] / radius;

    // Saturation for motors
    if (angVelArray[i] > 200) {
      angVelArray[i] = 200;
    } else if (angVelArray[i] < -200) {
      angVelArray[i] = -200;
    }
  }
} 

// TODO: Anti acceleration techniques and also exit condition. if the error hasnt changed over certain time then exit

//Pid Logic, input error, and pidvars struct name. Returns velocity.
float pidControl(pidvars* pidName, float error){
  // time difference
  long currT = micros();
  // long et = currT - pidName->prevT;
  float deltaT = ((float)(currT - pidName->prevT))/1.0e6;
  pidName->prevT = currT;

  //derivitive:
  float dedt = (error - pidName->eprev)/(deltaT); 

  // Integral with anti-windup
  float potentialIntegral = pidName->eintegral + error * deltaT;
  // Check if the potential integral term is within bounds
  if (potentialIntegral > pidName->integralLimit) 
  {
    pidName->eintegral = pidName->integralLimit;
  } else if (potentialIntegral < -pidName->integralLimit) 
  {
    pidName->eintegral = -pidName->integralLimit;
  } else 
  {
    pidName->eintegral = potentialIntegral;
  }

  //control signal:
  float velocity = pidName->kp * error + pidName->ki * pidName->eintegral + pidName->kd * dedt;

  // store previous error
  pidName->eprev = error;

  if (error < pidName->minError)
  {
    pidName->withinError = true;
  }
  else
  {
    pidName->prevBreakOutTime = millis();
  }

  return velocity;
}

// ----------------------Infrared Sensor------------------------

float getIRDistance(IRSensor* mIRSensor)
{
  int iteration = 0;
  float totalVoltage = 0;

  for (size_t i = 0; i < 10; i++)
  {
    totalVoltage += analogRead(mIRSensor->IR_PIN);
    iteration++;
  }

  float voltage = ((float)totalVoltage/(float)iteration) * (5.0 / 1023.0);
  float distance = 0;

  if (voltage <= mIRSensor->lowerVoltage)
  {
    mIRSensor->isTooFar = true;
    mIRSensor->isTooClose = false;
    mIRSensor->isInRange = false;
  }

  else if (voltage >= mIRSensor->upperVoltage)
  {
    mIRSensor->isTooClose = true;
    mIRSensor->isTooFar = false;
    mIRSensor->isInRange = false;
  }
  else {
    mIRSensor->isInRange = true;
    mIRSensor->isTooClose = false;
    mIRSensor->isTooFar = false;
  }

  if (mIRSensor->IR_PIN == A5) // Front Left Infrared Short Range (Pretty good)
  {
    distance =  81.487 * pow(voltage, 4) - 485.13 * pow(voltage, 3) + 1065.5 * pow(voltage, 2) - 1090 * voltage + 534.3;
  }
  else if (mIRSensor->IR_PIN == A7) // Back Left Infrared long range (Pretty good)
  {
    distance = -22.701 * pow(voltage, 5) + 310.95 * pow(voltage, 4) - 1461.7 * pow(voltage, 3) + 3187.9 * pow(voltage, 2) - 3440.3 * voltage + 1693.1;
  }
  else if (mIRSensor->IR_PIN == A9) // Front Right Infrared Long Range (Alright)
  {
    distance = -190.52 * pow(voltage, 5) + 1458.5 * pow(voltage, 4) - 4383.5 * pow(voltage, 3) + 6563 * pow(voltage, 2) - 5116.6 * voltage + 1932.8;
  }
  else if (mIRSensor->IR_PIN == A11) // Back Right Infrared Short Range (Alright)
  {
    distance = 61.795 * pow(voltage, 4) - 383.16 * pow(voltage, 3) + 881.92 * pow(voltage, 2) - 954.01 * voltage + 499.54;
  }
  else
  {
    return 0.0;
  }

  return distance; // Remember its distance should return
}

void printBool(IRSensor mIRSensor)
{
  BluetoothSerial.print("Is too close: ");
  delay(20);
  BluetoothSerial.println(mIRSensor.isTooClose);
  delay(20);
  BluetoothSerial.print("Is too far: ");
  delay(20);
  BluetoothSerial.println(mIRSensor.isTooFar);
  delay(20);
  BluetoothSerial.print("Is in Range: ");
  delay(20);
  BluetoothSerial.println(mIRSensor.isInRange);
  delay(20);
  BluetoothSerial.println();
}
