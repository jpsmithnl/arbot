/*
 * This sketch provides connectivity from ROS to an ARbot.  The ARbot consists of an iRobot Create,
 * with a connected arduino_irobot_bridge, which is connected to an Arduino.  The Arduino should
 * be connected to the computer running ROS through a USB cable.  
 *
 * The following functionality is implemented:
 * - Control of the robot's velocity through ROS topic "cmd_vel" which accepts a Twist message.
 * - Control of the servo connected to Arduino pin 9 through ROS topic "servo" (send int from 0-180).
 */

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Roomba.h>
#include <Servo.h> 
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point32.h>

// The ROS node
ros::NodeHandle nh;

// Devices
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
SoftwareSerial serial(6, 8);
//Roomba roomba(&Serial);
Roomba roomba(&Serial3);
Servo servo;
MPU6050 accelgyro;

// Most recent accelerometer (ax, ay, az) and gyro (gx, gy, gz) readings.
//int16_t ax, ay, az;
//int16_t gx, gy, gz;


// Robot parameters.
float HALF_BASELINE = 264 / 2; // mm

// Bias calculation time in milliseconds
unsigned long BIAS_TIME = 10000;

// Variables representing current states of various sensors and devices.
bool leds_on = false;

// Current and last pose.
float x = 0, y = 0, theta = 0;
float lastX = 0, lastY = 0, lastTheta = 0;

// For reading sensor data from the robot.
uint8_t roombaBuffer[52];

// For estimating bias of the gyro.
bool biasMeasuringMode = true;
float gyroBias = 0;
unsigned long biasCount = 0;
unsigned long startTime; // In microseconds

// To keep track of time (in microseconds)
unsigned long time, lastTime;

void toggle_leds() {
  leds_on = !leds_on;
  if (leds_on)
    roomba.leds(ROOMBA_MASK_LED_PLAY, 255, 255);
  else
    roomba.leds(ROOMBA_MASK_LED_PLAY, 0, 0);
}

// cmd_vel callback.
void cmd_vel_cb( const geometry_msgs::Point32& cmd_msg){
  // We extract only the forward (v) and rotation (w) speeds.
  float v = cmd_msg.x; // Forward speed in mm / sec
  float w = cmd_msg.z; // Angular speed in radians / sec
  
  // Apply differential-drive kinematic model.  Left and right are swapped and negated 
  // here because we define forwards as the direction of the gripper, not the bumper.
  // Also, we do not divide by the wheel radius because the 'driveDirect' command below
  // actually expects wheel roll speeds, not rates of rotation.
  int16_t leftVelocity = -(v + HALF_BASELINE * w); // / WHEEL_RADIUS;
  int16_t rightVelocity = -(v - HALF_BASELINE * w); // / WHEEL_RADIUS;

  lcd.clear();
  lcd.print(leftVelocity);
  lcd.print(", ");
  lcd.print(rightVelocity);

  roomba.driveDirect(leftVelocity, rightVelocity);
  
  toggle_leds();
}

// Servo callback.
void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

  toggle_leds();
}

// Subscribers
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<std_msgs::UInt16> servo_sub("servo", servo_cb);

// Publishers
//ros::Publisher<a

void setup(){
  lcd.begin(16, 2);
  lcd.print("Estimating bias");
  lcd.setCursor(0, 1);
  lcd.print("Do not move bot!");
  
  roomba.start();
  roomba.fullMode();
  
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_sub);
  
  servo.attach(9); //attach it to pin 9

  // Initialize IMU
  Wire.begin(); // needed for I2C
  accelgyro.initialize();  
  startTime = micros();    
}

void updateBias(int16_t gyroValue) {
  gyroBias += gyroValue;
  biasCount++;
  
  if (time - startTime > BIAS_TIME) {
    gyroBias /= biasCount;
    biasMeasuringMode = false;
    displayPose();
  }
}

void updateTheta(int16_t gyroValue) {
  // Update theta if the deviation from the bias value is above a noise threshold.
  if (abs(gyroValue - gyroBias) > 0.5 * gyroBias) {
    // Integrate the angular speed.  Divide by 4096 to get degrees.
    theta += (gyroValue - gyroBias)/4096;
  }
}

void updateXY() {
  /*
  bool ret = roomba.getSensors(Roomba::SensorDistance, roombaBuffer, 2);
  int distance = roombaBuffer[1] + 256 * roombaBuffer[0];
  
  // We use a partially updated theta value to better reflect movement over the
  // whole of the sampling period.
  float midTheta = lastTheta
  x += 
  */
}

void displayPose() {
  lcd.clear();
  lcd.print(x);
  lcd.setCursor(8, 0);
  lcd.print(y);
  lcd.setCursor(0, 1);
  lcd.print(theta);
}

void loop(){
  lastTime = time;
  time = micros();
  
  int16_t gyroValue = accelgyro.getRotationZ();

  if (biasMeasuringMode)
    updateBias(gyroValue);
  else {
    lastX = x;
    lastY = y;
    lastTheta = theta;
    
    updateTheta(gyroValue);
    updateXY();
    
    if (x != lastX || y != lastY || theta != lastTheta)
      displayPose();
  }
  
  nh.spinOnce();
}
