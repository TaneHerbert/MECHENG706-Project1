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
const byte left_front  = 46;
const byte left_rear   = 47;
const byte right_rear  = 48;
const byte right_front = 49;

/** 
  * Public Types
  */

//State machine states
enum STATE 
{
  INITIALISING,
  ORIENTATEROBOT,
  ALIGNATWALL,
  DRIVETOCORNER,
  DRIVEPATHWAY,
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
  ACONTROL,
  ALIGNCONTROL
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
  float integralLimit; // For anti integral wind-up
  float kp;
  float ki;
  float kd;
  unsigned long prevT;
  unsigned long breakOutTime;      // Min break out time 
  unsigned long prevBreakOutTime;  // Prev break out time
  bool withinError; // Error is below minimum error
  float minError;   // Minimum error that allows break out condition
};

/**
  * Public Variables
  */

// Global Coordinate based variables:
float xCoordinate;
float yCoordinate;
int robotDirection; // (0 = sonar facing wall to start), (1 = sonar facing away from wall to start)

// Time of one loop, 0.07 s (GYRO)
int T = 100;

// Voltage when gyro is initialised
float gyroZeroVoltage = 0;

float gyroSupplyVoltage = 5;   // supply voltage for gyro
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 3.0; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored

// current angle calculated by angular velocity integral on
float currentAngle = 0;
float prevAngle = 0;
int fullTurns = 0;  // Counter for full turns. Positive for clockwise, negative for counterclockwise

float currentDist = 0;
float prevDist = 0;
float prevprevDist = 0;

//Boolean for homing function to check the orientation of robot
bool validOrientation = false;

// This variable will track the cumulative change in angle.
float cumulativeAngleChange = 0;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

// Motor Speed
int speed_val = 100;

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

nonBlockingTimers mNonBlockingPrint = 
{
  .lastUpdateTime = 0,
};

nonBlockingTimers mNonBlockingSonar = 
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

IRSensor IR_BL_UNLIMITED = 
{ 
  .IR_PIN = A7, 
  .mSensorType = LONGRANGE,
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.37,
  .upperVoltage = 2.00 
};

IRSensor IR_FR_UNLIMITED = 
{ 
  .IR_PIN = A9, 
  .mSensorType = LONGRANGE,
  .isTooFar = true,
  .isTooClose = true,
  .isInRange = true,
  .lowerVoltage = 0.35,
  .upperVoltage = 2.00
};

//x coord PID variables
pidvars xVar = 
{
  .mPIDCONTROL = XCONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 200,
  .kp = 1.5,
  .ki = 0.154, // 0.154
  .kd = 0.02,  // 1.02
  .prevT = 0,
  .breakOutTime = 50,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 60,
};

//y coord PID variables
pidvars yVar = 
{
  .mPIDCONTROL = YCONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 200,
  .kp = 2.5,
  .ki = 0.205, // 0.205
  .kd = 0.1,  // 1.04
  .prevT = 0,
  .breakOutTime = 50,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 20, // was 30
};

//angular PID variables
pidvars aVar = 
{
  .mPIDCONTROL = ACONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 100,
  .kp = 0.4,
  .ki = 0.0, // 0.343
  .kd = 0.01, // 1.21
  .prevT = 0, 
  .breakOutTime = 20,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 1.0,
};

//Alignment PID variables
pidvars alignVar = 
{
  .mPIDCONTROL = ALIGNCONTROL,
  .eprev = 0,
  .eintegral = 0,
  .integralLimit = 200, // ????
  .kp = 2.2,
  .ki = 2.0, // 0.343
  .kd = 0.0, // 1.21
  .prevT = 0, 
  .breakOutTime = 300,
  .prevBreakOutTime = 0,
  .withinError = false,
  .minError = 20,
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

int pathStep = 0;
int segmentStep = 0;

float xCoordinateDes[20] = {100, 1850, 1850, 130, 130, 1850, 1850, 130, 130, 1850, 1850, 130, 130, 1850, 1850, 130, 130, 1850, 1850, 130};
float yCoordinateDes[20] = {150, 150, 250, 250, 350, 350, 450, 450, 550, 550, 650, 650, 750, 750, 850, 850, 950, 950, 1050, 1050};

float segmentArray[20] = {1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5}; // tells us how many segments we should break each path step into

float xDesired = xCoordinateDes[0];
float yDesired = yCoordinateDes[0];

float xPoint[100]; // X points to travel along the line, adjust the size as needed
float yPoint[100]; // Y points to travel along the line, adjust the size as needed

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

// Sonar
float HC_SR04_range();

// Delay
void delaySeconds(int TimedDelaySeconds);
bool nonBlockingDelay(unsigned long *lastMillis, unsigned long delayMicros);
bool breakOutTimerPID(unsigned long *lastMillis, unsigned long delayMicros);

// Builtin LED FLashing
void slow_flash_LED_builtin();
void fast_flash_double_LED_builtin();

// Gyro
void GyroSetup();
void getCurrentAngle();

// Coordinate System
void updateCoordinates();

// State machine
STATE initialising();
STATE stopped();
STATE orientateRobot();
STATE alignAtWall();
STATE driveToCorner();
STATE drivepathway();

// Battery Voltage
boolean is_battery_voltage_OK();

// IR Distance
float getIRDistance(IRSensor* IRSensor);
void printBool(IRSensor mIRSensor);

// Control System
void inverseKinematics (float Vx, float Vy, float Az);
float pidControl(pidvars* pidName, float error);

bool driveToPosition(float xDesiredPoisition, float yDesiredPosition);
void drivePoints(float xCoordinate, float yCoordinate, float xDesired, float yDesired, int n);

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
    case ORIENTATEROBOT:
      machine_state = orientateRobot();
      break;
    case ALIGNATWALL:
      machine_state = alignAtWall();
      break;
    case DRIVETOCORNER:
      machine_state = driveToCorner();
      break;
    case DRIVEPATHWAY:
      machine_state = drivepathway();
      break;
    case STOPPED:
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

// ----------------------Sonar------------------------
float HC_SR04_range()
{
  if (!nonBlockingDelay(&mNonBlockingSonar.lastUpdateTime, 50))
  {
    return;
  }

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

bool breakOutTimerPID(unsigned long *lastMillis, unsigned long delayMillis)
{
  unsigned long currentMillis = millis();

  // Check if the current time minus the last recorded time is greater than the delay
  if (currentMillis - *lastMillis >= delayMillis) 
  {
    return true;  // Return true if the delay has elapsed
  }
  return false;  // Return false if the delay has not elapsed
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

  currentAngle = 0;

  return ORIENTATEROBOT;
}

STATE orientateRobot() {
  
  /*NEW HOMING CODE*/
  speed_val = 200;
  cw();
  //Turn until it reaches a mininum where both long range sensors are in range
  if (!validOrientation){
    if (abs(abs(currentAngle) - abs(prevAngle)) >= 3) // Check orientation every 9 degrees turned
    {
      currentDist = HC_SR04_range(); //measure current sonar distance
      getIRDistance(&IR_BL_UNLIMITED); //measure distances for long range sensors on either side
      getIRDistance(&IR_FR_UNLIMITED);

      //Check if you are at a minimum point
      if ((prevDist < prevprevDist) && (prevDist < currentDist)){
        //Check if you are at a horizontal orientation, and not vertical (By checking if both long range sensors arent too far)
        if ((!IR_BL_UNLIMITED.isTooFar) && (!IR_FR_UNLIMITED.isTooFar) || (prevDist > 950)){
          validOrientation = true;
          return ORIENTATEROBOT;
        } 
      }
      prevprevDist = prevDist;
      prevDist = currentDist;
      prevAngle = currentAngle;
    }
    return ORIENTATEROBOT;
  }

  stop();
  return ALIGNATWALL;

}

STATE alignAtWall(){

  //Drive backwards
  speed_val = 300;
  reverse();

  //Measure distance
  currentDist = HC_SR04_range() * 10;

  if (currentDist < 1750){
    //if has not reached wall, keep driving forward
    return ALIGNATWALL;
  }
  else{
    //Wait 2 seconds and then stop
    delay(500);
    stop();
    return DRIVETOCORNER;
  }

}

STATE driveToCorner()
{
  if (alignVar.prevT == 0)
  {
    alignVar.prevT = millis();
  }

  if (!nonBlockingDelay(&mNonBlockingTimerPID.lastUpdateTime, 50)) // Run straight every 50 ms
  {
    return DRIVETOCORNER;
  }

  float yDesiredPosition = 70;  // SHORT DISTANCE

  updateCoordinates();

  // if (nonBlockingDelay(&mNonBlockingPrint.lastUpdateTime, 1000)) // Run straight every 50 ms
  // {
  //   // BluetoothSerial.println(yCoordinate);
  //   BluetoothSerial.println("BACK LEFT");
  //   printBool(IR_BL);
  //   BluetoothSerial.println("BACK RIGHT");
  //   printBool(IR_BR);
  //   BluetoothSerial.println("FRONT RIGHT");
  //   printBool(IR_FR);
  //   BluetoothSerial.println("FRONT LEFT");
  //   printBool(IR_FL);
  // }

  float yError = yDesiredPosition - yCoordinate;

  float yVelocity = pidControl(&alignVar, yError);
  inverseKinematics(-380, yVelocity, 0);

  left_font_motor.writeMicroseconds(1500 + angVelArray[0]);
  right_font_motor.writeMicroseconds(1500 - angVelArray[1]);
  left_rear_motor.writeMicroseconds(1500 + angVelArray[2]);
  right_rear_motor.writeMicroseconds(1500 - angVelArray[3]);

  angVelArray[0] = 0.0;
  angVelArray[1] = 0.0;
  angVelArray[2] = 0.0;
  angVelArray[3] = 0.0;

  if (alignVar.withinError == true)
  {
    int check = 0;

    if (breakOutTimerPID(&alignVar.breakOutTime, alignVar.breakOutTime))
    {
      check++;
    }

    if (check == 1)
    {
      alignVar.eprev = 0;
      alignVar.eintegral = 0;

      stop();
      // speed_val = 300;
      // reverse();    
      delay(1000);
      // stop();
      currentAngle = 0;
      return DRIVEPATHWAY;
    }
  }

  return DRIVETOCORNER;
}

STATE drivepathway()
{
  if (driveToPosition(xDesired, yDesired)){ // Will output true if the robot has reached the current desired position
    segmentStep++;
    xDesired = xPoint[segmentStep];
    yDesired = yPoint[segmentStep];

    if (segmentStep >= segmentArray[pathStep]){ // The robot has reached the end of the step and needs to move to the next point path point
      segmentStep = 0; // reset segment step
      pathStep++;

      drivePoints((xCoordinateDes[pathStep-1]),(yCoordinateDes[pathStep-1]),(xCoordinateDes[pathStep]),(yCoordinateDes[pathStep]),(segmentArray[pathStep]));

      xDesired = xPoint[0];
      yDesired = yPoint[0];
    }
    BluetoothSerial.print("X-Position ");
    BluetoothSerial.println(xDesired);
    BluetoothSerial.print("Y-Position ");
    BluetoothSerial.println(yDesired);
    BluetoothSerial.println();

    if (pathStep >= 20)
    {
      return STOPPED;
    }
  }

  return DRIVEPATHWAY;
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

  // if (nonBlockingDelay(&mNonBlockingPrint.lastUpdateTime, 2000)) // Run straight every 50 ms
  // {
  //   BluetoothSerial.println(currentAngle);  
  // }

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

  bool dontUpdate = false;

  float backLeftDistance = getIRDistance(&IR_BL);
  float backRightDistance = getIRDistance(&IR_BR);
  float frontRightDistance = getIRDistance(&IR_FR);
  float frontLeftDistance = getIRDistance(&IR_FL);

  // FOR LEFT SIDE TOO CLOSE LESS THAN 40mm
  if ((IR_FL.isTooClose == true) && (IR_BL.isTooClose == true) && (IR_FR.isTooFar == true) && (IR_BR.isTooFar == true)){
    yCoordinate = 72.0;
  }
  //FOR LEFT SIDE  VERY CLOSE, LESS THAN 100mm
  else if ((IR_FL.isInRange) && (IR_BL.isTooClose) && (IR_FR.isTooFar) && (IR_BR.isTooFar)){
    yCoordinate = (frontLeftDistance + 72.0);
  }

  //FOR LEFT SIDE MEDIUM CLOSE, MORE THAN 100mm, LESS THAN 290mm
  else if ((IR_FL.isInRange) && (IR_BL.isInRange) && (IR_FR.isTooFar) && (IR_BR.isTooFar)){
    yCoordinate = (frontLeftDistance + 72.0);
  }

  //FOR RIGHT SIDE MEDIUM CLOSE, MORE THAN 100mm, LESS THAN 290mm
  else if ((IR_FR.isInRange) && (IR_BR.isInRange) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    yCoordinate = (1200.0 - 72.0 - backRightDistance);
  }

  //FOR RIGHT SIDE VERY CLOSE, LESS THAN 100mm
  else if ((IR_FR.isTooClose) && (IR_BR.isInRange) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    //CAN ADD FUNCTIONALITY TO AVERAGE BOTH SENSORS HERE IF NEED BE.
    yCoordinate = (1200.0 - 72.0 - backRightDistance);
  }

  //FOR RIGHT SIDE TOO CLOSE, LESS THAN 40mm
  else if ((IR_FR.isTooClose) && (IR_BR.isTooClose) && (IR_FL.isTooFar) && (IR_BL.isTooFar)){
    yCoordinate = 1200.0 - 72.0;
  } 

    //FOR IN MIDDLE, MORE THAN 290mm on both sides
  else if ((IR_FL.isTooFar) && (IR_FR.isInRange || IR_BL.isInRange) && (IR_BR.isTooFar)){
    if (frontRightDistance <= 600){
      //use frontright sensor only
      yCoordinate = (1200.0 - frontRightDistance - 72.0);
    }

    //If its closer to the left side
    else if (backLeftDistance <= 540){
      //use backleft sensor only
      yCoordinate = (backLeftDistance + 72.0);
    }    

    else{
      //dont update the coordinates.
      dontUpdate = true;
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

  xCoordinate = 2000 - ((10 * HC_SR04_range()) + 105);
}
// ----------------------Control System------------------------

//Function for inverse kinematics. Input velocities, output angular velocity of each wheel.
void inverseKinematics (float Vx, float Vy, float Az)
{
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
    if (angVelArray[i] > 500) {
      angVelArray[i] = 500;
    } else if (angVelArray[i] < -500) {
      angVelArray[i] = -500;
    }
  }
} 

// TODO: If the error hasnt changed over certain time then exit

//Pid Logic, input error, and pidvars struct name. Returns velocity.
float pidControl(pidvars* pidName, float error){
  // time difference
  long currT = micros();

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

  if (pidName->mPIDCONTROL == XCONTROL)
  {
    if(nonBlockingDelay(&mNonBlockingPrint.lastUpdateTime, 200))
    {
      BluetoothSerial.println(dedt);
    }
  }

  if ((abs(error) < abs(pidName->minError)))
  {
    pidName->withinError = true;
  }
  else
  {
    pidName->withinError = false;
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

bool driveToPosition(float xDesiredPoisition, float yDesiredPosition)
{
  if (xVar.prevT == 0 && xVar.prevT == 0 && aVar.prevT == 0)
  {
    xVar.prevT = millis();
    yVar.prevT = millis();
    aVar.prevT = millis();
  }

  if (!nonBlockingDelay(&mNonBlockingTimerPID.lastUpdateTime, 80)) // Run straight every 50 ms
  {
    return false;
  }

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

  // if (nonBlockingDelay(&mNonBlockingPrint.lastUpdateTime, 2000)) // Run straight every 50 ms
  // {
  //   BluetoothSerial.print("X-coordinate ");
  //   delay(20);
  //   BluetoothSerial.println(xCoordinate);     delay(20);
  //   BluetoothSerial.print("Y-coordinate ");    delay(20);
  //   BluetoothSerial.println(yCoordinate);    delay(20);
  //   BluetoothSerial.print("X-Leave ");    delay(20);
  //   BluetoothSerial.println(xVar.withinError);    delay(20);
  //   BluetoothSerial.print("Y-Leave ");    delay(20);
  //   BluetoothSerial.println(yVar.withinError);    delay(20);
  // }

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

  left_font_motor.writeMicroseconds(1500 + angVelArray[0]);
  right_font_motor.writeMicroseconds(1500 - angVelArray[1]);
  left_rear_motor.writeMicroseconds(1500 + angVelArray[2]);
  right_rear_motor.writeMicroseconds(1500 - angVelArray[3]);

  angVelArray[0] = 0.0;
  angVelArray[1] = 0.0;
  angVelArray[2] = 0.0;
  angVelArray[3] = 0.0;

  if (xVar.withinError == true && yVar.withinError == true && aVar.withinError == true)
  {
    int check = 0;

    if (breakOutTimerPID(&xVar.breakOutTime, xVar.breakOutTime))
    {
      check++;
    }
    
    if (breakOutTimerPID(&yVar.breakOutTime, yVar.breakOutTime))
    {
      check++;
    }
    
    if (breakOutTimerPID(&aVar.breakOutTime, aVar.breakOutTime))
    {
      check++;
    }

    if (check == 3)
    {
      aVar.eprev = 0;
      xVar.eprev = 0;
      yVar.eprev = 0;

      aVar.eintegral = 0;
      xVar.eintegral = 0;
      yVar.eintegral = 0;

      aVar.prevT = 0;
      xVar.prevT = 0;
      yVar.prevT = 0;

      if (pathStep == 4 || pathStep == 8 || pathStep == 12 || pathStep == 16)
      {
        reverse();
        delay(400);
        stop();
        delay(400);
        currentAngle = 0;
      }

      if (pathStep == 2 || pathStep == 6 || pathStep == 10 || pathStep == 14)
      {
        forward();
        delay(400);
        stop();
        delay(400);
        currentAngle = 0;
      }
      return true;
    }
  }

  return false;
}

void drivePoints(float xCoordinate, float yCoordinate, float xDesired, float yDesired, int n) 
{
  // Calculate the step increments for each axis
  float xStep = (xDesired - xCoordinate) / n;
  float yStep = (yDesired - yCoordinate) / n;

  // Populate the arrays with the points
  // Start from 1 to skip the current position and end at n to include the desired position
  for (int z = 1; z <= n; z++) {
    xPoint[z - 1] = xCoordinate + xStep * z;
    yPoint[z - 1] = yCoordinate + yStep * z;
  } 
}
