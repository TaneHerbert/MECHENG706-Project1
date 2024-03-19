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

// #define NO_BATTERY_V_OK // Uncomment of BATTERY_V_OK if you do not care about battery damage.

#define STARTUP_DELAY 1 // Seconds

#define MAX_SONARDIST_CM 400
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
  RUNNING,
  STOPPED
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

//Time of one loop
int T = 100;

//Voltage when gyro is initialised
float gyroZeroVoltage = 0;

float gyroSupplyVoltage = 5;   // supply voltage for gyro
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored
float gyroRate = 0;            // read out value of sensor in voltage

// current angle calculated by angular velocity integral on
//Let 180 degrees be straight forward (Just ease of coding for now)
float currentAngle = 180;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

// Motor Speed
int speed_val = 150;
int speed_change;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

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

// Builtin LED FLashing
void slow_flash_LED_builtin();
void fast_flash_double_LED_builtin();

// Gyro
void GyroSetup();
void getCurrentAngle();


// Homing
void findCorner();

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

// Battery Voltage
boolean is_battery_voltage_OK();

/**
 * Set up
 */

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  BluetoothSerial.begin(115200);

  BluetoothSerial.println("MECHENG706_Base_Code_07.03.2024");
  BluetoothSerial.println("Setup....");

  // Inits
  GyroSetup();  //Set up starting voltage for gyro

  // settling time but no really needed
  delaySeconds(STARTUP_DELAY);
}

void loop(void)  //main loop
{
  delaySeconds(5);

  // static STATE machine_state = INITIALISING;
  // //Finite-state machine Code
  // switch (machine_state) {
  //   case INITIALISING:
  //     machine_state = initialising();
  //     break;
  //   case RUNNING:  //Lipo Battery Volage OK
  //     machine_state = running();
  //     break;
  //   case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
  //     machine_state = stopped();
  //     break;
  // };

  findCorner();

  while(1)
  {

  }
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

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
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  BluetoothSerial.println("Enabling Motors...");
  enable_motors();
  BluetoothSerial.println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis;

  fast_flash_double_LED_builtin();

  //Arduino style 500ms timed execution statement
  if (millis() - previous_millis > 500) {
    previous_millis = millis();

    BluetoothSerial.println("RUNNING---------");
    speed_change_smooth();

    #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
    #endif
  }

  return RUNNING;
}

STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;

  disable_motors();
  slow_flash_LED_builtin();

  //print massage every 500ms
  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    BluetoothSerial.println("STOPPED---------");

    #ifndef NO_BATTERY_V_OK
      //500ms timed if statement to check lipo and output speed settings
      if (is_battery_voltage_OK()) {
        BluetoothSerial.print("Lipo OK waiting of voltage Counter 10 < ");
        BluetoothSerial.println(counter_lipo_voltage_ok);

        counter_lipo_voltage_ok++;

        //Making sure lipo voltage is stable
        if (counter_lipo_voltage_ok > 10) {
          counter_lipo_voltage_ok = 0;
          enable_motors();
          BluetoothSerial.println("Lipo OK returning to RUN STATE");
          return RUNNING;
        }
      } 
      else {
        counter_lipo_voltage_ok = 0;
      }
    #endif
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

// ----------------------Homing------------------------

void findCorner()
{  
  cw();
  enable_motors();

  float distances[80];
  float cornerDistances[8];

  int cornerIndex = 0;

  for (int i = 0; i < 80; i++) // TODO: Change to gyro so we know we done a 360 degree turn
  {
    // Measure Distance
    distances[i] = HC_SR04_range();
    delay(20);
  }
 
  stop();
  disable_motors();

  // Analyze the collected distances to find corners
  for (int i = 1; i < 79; i++) // Start from 1 and end at 78 to avoid out of bound indexes
  {
    if (distances[i] > MAX_SONARDIST_CM || distances[i] < MIN_SONARDIST_CM)
      continue; // Skip invalid readings;

    // Check for a significant change in distance indicating a corner
    // Introduce a threshold (e.g., deltaThreshold) to define what constitutes a significant change
    float deltaThreshold = 10.0; // Adjust based on your robot's environment and sensor
    bool isCorner = (abs(distances[i] - distances[i + 1]) > deltaThreshold) && 
                    (abs(distances[i] - distances[i - 1]) > deltaThreshold);
    
    if (isCorner)
    {
      cornerDistances[cornerIndex] = distances[i];
      cornerIndex++;
      // if (cornerIndex >= 8) break;
    }
  }

  // Output the detected corners
  BluetoothSerial.println("Corners");
  for (int i = 0; i < cornerIndex; i++)
  {
    BluetoothSerial.println(cornerDistances[i]);
  }
  BluetoothSerial.print("Amount of Corners Indicated: ");
  BluetoothSerial.println(cornerIndex);
  BluetoothSerial.println("End");
}

// ----------------------Gyro------------------------

void GyroSetup() 
{
  // this section is initialize the sensor, find the value of voltage when gyro is zero
  int i;
  float sum = 0;
  int sensorValue = 0;  // read out value of sensor

  pinMode(gyroSensorPin, INPUT);
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(gyroSensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
}

void getCurrentAngle() 
{
  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(gyroSensorPin) * gyroSupplyVoltage) / 1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;  // from Data Sheet, gyroSensitivity is 0.007 V/dps
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }
  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
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
  
    getCurrentAngle(); // update current angle (Z coordinate)
    updateIRDistance(frontLeft);
    updateIRDistance(backLeft);
    updateIRDistance(frontRight);
    updateIRDistance(backRight);

    // FOR LEFT SIDE CLOSE
    if ((frontLeftDistance < 200) && (backLeftDistance < 200)){
     if (wallDirection == 0){ // left side closer to start
       yCoordinate = (frontLeftDistance + backLeftDistance) / 2;
      }
     else { // left side closer to finish
       yCoordinate = ((1200 - frontLeftDistance) + (1200 - backLeftDistance)) / 2;
     }    
    }

    // FOR RIGHT SIDE CLOSE
    if ((frontRightDistance < 200) && (backRightDistance < 200)){
      if (wallDirection == 1){ // right side closer to start
        yCoordinate = (frontRightDistance + backRightDistance) / 2;
      }
      else { // right side closer to finish
        yCoordinate = ((1200 - frontRightDistance) + (120 - backRightDistance)) / 2;
      }
    } 

    // FOR IN THE CENTRE 
    if ((frontRightDistance > 200) && (backLeftDistance > 200)){
      if (wallDirection == 0){ // adjust coordinates to be relative to TOP LEFT/BOTTOM RIGHT at start
        yCoordinate = ((backLeftDistance) + (1200 - frontRightDistance)) / 2;
      }
      else { // adjust coordinates to be relative to TOP RIGHT/BOTTOM LEFT at start
        yCoordinate = ((1200 - backLeftDistance) + (frontRightDistance)) / 2;
      }
    }  

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

    updateIRDistance(frontLeft);
    updateIRDistance(backLeft);
    updateIRDistance(frontRight);
    updateIRDistance(backRight);
    
    // If left side closer to wall, set wallDirection to 0
    if ((frontLeftDistance + backLeftDistance) < (frontRightDistance + backRightDistance)){
      wallDirection = 0;
    }
    // Else, right side is closer to wall therefore set wallDirection to 1
    else{
      wallDirection = 1;
    }
  }