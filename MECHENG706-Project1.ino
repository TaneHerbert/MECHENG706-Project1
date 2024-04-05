/**
 * Our Code
 *
 * Authors:
 *  Tane Herbert
 *  Juaninho Penteado
 *  Danny Su
 *  Jackson Taylor
*/

//test change

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

#define frontLeft 97 // SHORT RANGE IR (Number relates to specefic sensor)
#define backLeft 69  // LONG RANGE IR (Number relates to specefic sensor)
#define frontRight 8 // LONG RANGE IR (Number relates to specefic sensor)
#define backRight 66 // SHORT RANGE IR (Number relates to specefic sensor)

const int IR_PIN_FL = A5; // Front Left Infrared
const int IR_PIN_BL = A7; // Back Left Infrared
const int IR_PIN_FR = A4; // Front Right Infrared
const int IR_PIN_BR = A6; // Back Right Infrared

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
enum STATE {
  INITIALISING,
  FINDCORNER,
  RUNNING,
  STOPPED
};

enum IRSENSORTYPE
{
  SHORTRANGE,
  LONGRANGE
};

struct nonBlockingTimers
{
  unsigned long lastUpdateTimeGyro;
};

struct IRSensor
{
  const int IR_PIN;
  IRSENSORTYPE mSensorType;
};

/**
  * Public Variables
  */

// Values according to the IR sensors
float frontLeftDistance;
float frontRightDistance;
float backLeftDistance;
float backRightDistance;

// Coordinate based variables:
float xCoordinate;
float yCoordinate;
int wallDirection; // (0 = starts in TOP LEFT/BOTTOM RIGHT), (1 = starts in TOP RIGHT/BOTTOM LEFT)

float fLV;
float fRV;
float bLV;
float bRV;

// Time of one loop, 0.1 s
int T = 70;

// Voltage when gyro is initialised
float gyroZeroVoltage = 0;

float gyroSupplyVoltage = 5;   // supply voltage for gyro
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored

// current angle calculated by angular velocity integral on
float currentAngle = 0;
float prevAngle = 0;
int fullTurns = 0;  // Counter for full turns. Positive for clockwise, negative for counterclockwise

float distances[200] = { 0 };
float angles[200] = { 0 };

float angleCorner[200] = {0};
float cornerDistances[200] = {0};

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

nonBlockingTimers mNonBlockingTimers = 
{
  .lastUpdateTimeGyro = 0,
};

static STATE machine_state;

static IRSensor IR_FL = { .IR_PIN = A5, .mSensorType = SHORTRANGE };
static IRSensor IR_BL = { .IR_PIN = A7, .mSensorType = LONGRANGE  };
static IRSensor IR_FR = { .IR_PIN = A9, .mSensorType = LONGRANGE  };
static IRSensor IR_BR = { .IR_PIN = A11, .mSensorType = SHORTRANGE }; // Was A6

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

// Infrared Sensor
void updateIRDistance(int irSensor);

// Coordinate System
void updateCoordinates();

// Control System
void Move();
void setWallDirection();

// State machine
STATE initialising();
STATE running();
STATE stopped();
STATE findCorner();

// Battery Voltage
boolean is_battery_voltage_OK();

// IR Distance
float getIRDistance(IRSensor IRSensor);

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
      currentAngle = 0;
      break;
    case FINDCORNER:
      machine_state = findCorner();
      break;
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };

  // /**
  //  * Methods that must run every loop 
  //  */
  // getCurrentAngle(); // This function must run every 100ms so is placed outside the FSM

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
    BluetoothSerial.print("HC-SR04: Out of range");
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

STATE initialising() {
  //initialising
  BluetoothSerial.println("INITIALISING....");

  // Inititlisations

  enable_motors();
  GyroSetup();  //Set up starting voltage for gyro
  setWallDirection();

  BluetoothSerial.println("RUNNING STATE...");

  return FINDCORNER;
}

STATE findCorner() {

  // float distance = HC_SR04_range();
  // BluetoothSerial.println(distance);
  // BluetoothSerial.println();

  // delay(2000);

  // float distance;

  // distance = getIRDistance(IR_FR);
  // BluetoothSerial.print("Front Right: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(IR_FL);
  // BluetoothSerial.print("Front Left: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  updateCoordinates();

  BluetoothSerial.println("Y-Coordinate (measured from IR):");

  BluetoothSerial.println(yCoordinate);

  delay(2000);


  return FINDCORNER;

  // updateCoordinates();

  // BluetoothSerial.println("Y-Coordinate (measured from IR):");

  // BluetoothSerial.println(yCoordinate);

  // delay(2000);

  // float distance;

  // distance = getIRDistance(IR_FL);
  // BluetoothSerial.print("Front Left: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(IR_FR);
  // BluetoothSerial.print("Front Right: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(IR_BL);
  // BluetoothSerial.print("Back Left: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // distance = getIRDistance(IR_BR);
  // BluetoothSerial.print("Back Right: ");
  // BluetoothSerial.println(distance);

  // delay(2000);

  // BluetoothSerial.println();


  return FINDCORNER;
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

STATE running() {
  // static unsigned long previous_millis;

  // fast_flash_double_LED_builtin();
  BluetoothSerial.println(currentAngle);
  // delay(500);

  // //Arduino style 500ms timed execution statement
  // if (millis() - previous_millis > 500) {
  //   previous_millis = millis();

  //   BluetoothSerial.println("RUNNING---------");
  //   speed_change_smooth();

    // #ifndef NO_BATTERY_V_OK
    //   if (!is_battery_voltage_OK()) return STOPPED;
    // #endif
  // }

  return RUNNING;
}

STATE stopped() {
  stop();
  disable_motors();
  slow_flash_LED_builtin();

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
  BluetoothSerial.println("please keep the sensor still for calibration");
  BluetoothSerial.println("get the gyro zero voltage");
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
  if (!nonBlockingDelay(&mNonBlockingTimers.lastUpdateTimeGyro, T))
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

// ----------------------Infrared Sensor------------------------

void updateIRDistance(int irSensor)
{
  //update voltage/distance reading for front left IR sensor  
  if (irSensor == frontLeft){
    fLV = analogRead(IR_PIN_FL) * (5.0 / 1023.0);
    // If voltage within accurate range, set to calculated distance
    if (fLV < 2.75){
      frontLeftDistance = 11.26 * pow(fLV, 4) -104.53 * pow(fLV, 3) + 358.65 * pow(fLV, 2) -565.76 * fLV + 413.57;
    } 
  // Else set to minimum accurate reading    
    else {
      frontLeftDistance = 40;
    }
  }

  //update voltage/distance reading for back left IR sensor  
  if (irSensor == backLeft){
    bLV = analogRead(IR_PIN_BL) * (5.0 / 1023.0);
    if (bLV < 2.25){
      backLeftDistance = 50.961 * pow(bLV, 4) -355.71 * pow(bLV, 3) + 972.91 * pow(bLV, 2) -1316.4 * bLV + 882.22;
    } 
    else {
      backLeftDistance = 100;
    }
  }

  //update voltage/distance reading for back right IR sensor  
  if (irSensor == backRight){
    bRV = analogRead(IR_PIN_BR) * (5.0 / 1023.0);
    if (bRV < 2.57){
      backRightDistance = 33.515 * pow(bRV, 4) -236.7 * pow(bRV, 3) + 627.52 * pow(bRV, 2) -788.81 * bRV + 478.65;
    } 
    else {
      backRightDistance = 40;
    }
  }

  //update voltage/distance reading for back right IR sensor  
  if (irSensor == frontRight){
    fRV = analogRead(IR_PIN_FR) * (5.0 / 1023.0);
    if (bRV < 2.38){
      frontRightDistance = 130.26 * pow(fRV, 4) -889.33 * pow(fRV, 3) + 2284.3 * pow(fRV, 2) -2732.8 * fRV + 478.65;
    } 
    else {
      frontRightDistance = 100;
    }
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
  
    backLeftDistance = getIRDistance(IR_BL);
    backRightDistance = getIRDistance(IR_BR);
    frontRightDistance = getIRDistance(IR_FR);
    frontLeftDistance = getIRDistance(IR_FL);

    // // FOR LEFT SIDE CLOSE
    // if ((frontLeftDistance < 250) && (backLeftDistance < 250)){
    //  if (wallDirection == 0){ // left side closer to start
    //    yCoordinate = (frontLeftDistance);
    //   }
    //  else { // left side closer to finish
    //    yCoordinate = (1200 - frontLeftDistance);
    //  }    
    // }

    // FOR RIGHT SIDE CLOSE
    if ((frontRightDistance < 250) && (backRightDistance < 250)){
      if (wallDirection == 1){ // right side closer to start
        yCoordinate = backRightDistance;
      }
      else { // right side closer to finish
        yCoordinate = (1200 - backRightDistance);
      }
    } 

    // // FOR IN THE CENTRE 
    // if ((frontRightDistance > 250) && (backLeftDistance > 250)){
    //   if (wallDirection == 0){ // adjust coordinates to be relative to TOP LEFT/BOTTOM RIGHT at start
    //     yCoordinate = ((backLeftDistance) + (1200 - frontRightDistance)) / 2;
    //   }
    //   else { // adjust coordinates to be relative to TOP RIGHT/BOTTOM LEFT at start
    //     yCoordinate = ((1200 - backLeftDistance) + (frontRightDistance)) / 2;
    //   }
    // }  
  }

// ----------------------Control System------------------------

  // Implements PID control using X,Y,Z coordinate system
  void Move()
  {
    /* TWO STATES: Driving close to the wall, Driving away from the wall.
    
    For Driving close to the wall (<200mm): we will rely on the IR sensors on the side close to wall.
    For Driving away from the wall (>200mm): we will rely on the long range IR sensors either side
    */

  }

  // Call this function once at the beginning after robot is alligned to define what side starts closer to the wall
  void setWallDirection()
  {
    // If left side starts closer to the wall, we are either in TOP LEFT or BOTTOM RIGHT
    // If right side starts closer to the wall, we are either in TOP RIGHT of BOTTOM LEFT
    backLeftDistance = getIRDistance(IR_BL);
    backRightDistance = getIRDistance(IR_BR);
    frontRightDistance = getIRDistance(IR_FR);
    frontLeftDistance = getIRDistance(IR_FL);
    
    // If left side closer to wall, set wallDirection to 0
    if (frontLeftDistance < backRightDistance){
      wallDirection = 0;
      BluetoothSerial.print("Starting point: Top Left or Bottom Right");
      BluetoothSerial.print("TEST TEST");
    }
    // Else, right side is closer to wall therefore set wallDirection to 1
    else{
      wallDirection = 1;
      BluetoothSerial.print("Starting point: Top Right or Bottom Left");
      BluetoothSerial.print("TEST TEST");
    }
  }

  float getIRDistance(IRSensor IRSensor)
  {
    int iteration = 0;
    float totalVoltage = 0;

    for (size_t i = 0; i < 10; i++)
    {
      totalVoltage += analogRead(IRSensor.IR_PIN);
      iteration++;
    }

    float voltage = ((float)totalVoltage/(float)iteration) * (5.0 / 1023.0);
    float distance = 0;

    if (IRSensor.IR_PIN == A5) // Front Left Infrared Short Range (Pretty good)
    {
      distance =  81.487 * pow(voltage, 4) - 485.13 * pow(voltage, 3) + 1065.5 * pow(voltage, 2) - 1090 * voltage + 534.3;
    }
    else if (IRSensor.IR_PIN == A7) // Back Left Infrared long range (Pretty good)
    {
      distance = -22.701 * pow(voltage, 5) + 310.95 * pow(voltage, 4) - 1461.7 * pow(voltage, 3) + 3187.9 * pow(voltage, 2) - 3440.3 * voltage + 1693.1;
    }
    else if (IRSensor.IR_PIN == A9) // Front Right Infrared Long Range (Alright)
    {
      distance = -190.52 * pow(voltage, 5) + 1458.5 * pow(voltage, 4) - 4383.5 * pow(voltage, 3) + 6563 * pow(voltage, 2) - 5116.6 * voltage + 1932.8;
    }
    else if (IRSensor.IR_PIN == A11) // Back Right Infrared Short Range (Alright)
    {
      distance = 61.795 * pow(voltage, 4) - 383.16 * pow(voltage, 3) + 881.92 * pow(voltage, 2) - 954.01 * voltage + 499.54;
    }
    else
    {
      return 0.0;
    }

    return distance;
  }
