//****************************************//
//* Autonomous Parking solution code     *//
//* UoN 2022 - Noor Almoayed             *//
//****************************************//

#include <Wire.h> // required for I2C communication between sensors 
#include <MPU6050_tockn.h> // include MPU library 
#include <Servo.h>    //include the servo library
#include <Encoder.h> // include encoder library
#include <math.h> // include mathematical functions library 

#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define LED_PIN 17   // ESP32 pin GIOP17 connected to LED's pin
#define trigPin 26  // ESP32 pin GIOP26 connected to Ultrasonic Sensor's TRIG pin
#define echoPin 27  // ESP32 pin GIOP27 connected to Ultrasonic Sensor's ECHO pin
#define LEDHC_PIN 17 // ESP32 pin GIOP17 connected to LED's pin
#define Distance_THRESHOLD 50  //CENTIMETERS
#define enA 5  // enableA command line  - should be PWM pin
#define enB 6  // enableB command line  - should be PWM pin


MPU6050 mpu6050(Wire);
//declaring variables 
int angle = 0;
int distance = 999;
long duration; // wider than int
float distancecm; // decimal values 
int leftMotor_speed, rightMotor_speed, servoAngle;
int x = 0;


void setup()
{
  Serial.begin(9600); // starts serial communication at baud rate of 9600
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  mpu6050.begin(): // I2C initializes mpu sensor 
  mpu6050.calcGyroOffsets(true); // I2C initializes mpu sensor 
  pinMode(LED_PIN)
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}


void loop()
{
  //moving the car forward for 1 second: 
  leftMotor_speed = 255;
  rightMotor_speed = 249;
  servoAngle = 85; //ideally would be 90 
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); // function sends values to motors and servos
  delay(1000); // pauses execution for 1 second 
  
  //stoping the car: 
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //rotating the car 180 degrees
  leftMotor_speed = 150;
  rightMotor_speed = 70;
  servoAngle = 100;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle > -180) {
    angle = accelerometer(); // reads angle 
  }

  //stoping car again
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //reverse until 10cm away from wall
  leftMotor_speed = -100; // negative sign so that it can turn in other direction
  rightMotor_speed = -70;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultrasonic();
    delay(1000);
  }
  while (distance >= 10);
  distance = 999;

  //stoping car again 
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //rotating 90 degrees
  leftMotor_speed = 50;
  rightMotor_speed = 150;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle < -90) {
    angle = accelerometer();
  }

  //stopping again 
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //reverse untill 10cm from wall
  leftMotor_speed = -100;
  rightMotor_speed = -70;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultrasonic();
    delay(1000);
  }
  while (distance >= 10);
  distance = 999;
  
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 85;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000000);
}

 
void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)

{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to devices being controlled by arduino and esp32
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}


float accelerometer() //reading and saving MPU angle 
{
  mpu6050.update() //calling update function 
  return (mpu6050.getAngleZ()); // update function returns valye for angle measured along Z- Axis 
}


float ultrasonic() //reading and saving distance measured 
{
  float distance;
  digitalWrite(trigPin, LOW); // sends trigger signal 
  delayMicroseconds(1000); // waiting time
  digitalWrite(trigPin, HIGH); // change high again and measures duration of pulse 
  delayMicroseconds(1000); // measures in microseconds 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034 / 2); // calculates distance using speed of sound in air 
}

 
 
