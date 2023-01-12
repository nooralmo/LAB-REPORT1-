#include <Servo.h>    //include the servo library
// include encoder library
#include <Encoder.h>
#define servoPin 4
Servo myservo;        // create servo object to control a servo
 
#define enA 5   //EnableA command line - should be a PWM pin
#define enB 6   //EnableB command line - should be a PWM pin
 
#define INa A0  //Channel A direction
#define INb A1  //Channel A direction
#define INc A2  //Channel B direction
#define INd A3  //Channel B direction
 
byte speedSetting = 0;  //initial speed = 0
 
//enable pins with interrupt capability
Encoder myEnc(2, 3);
 
long oldPosition  = -999;
int count = 0; //encoder count initialised to 0
float distance;
 
void setup() {
  //initialise serial communication
  Serial.begin(9600);
 
 
  myservo.write(85);
  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)
 
  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);  
 

  motors(255, 249);
}
 

void loop() {
  goBackwards(); //moving backwards meant the car went straighter
  long newPosition = myEnc.read(); //myEnc.read reads the new position
 
  if (newPosition != oldPosition) //updating position
  {    
    oldPosition = newPosition;
 
    count += 1; //encoder count increases
 
    Serial.print("Distance(m): ");
    Serial.println(distance);
  }
 
  if(count == 1500)
  {
    stopMotors();
    delay(50000);
  }
}
 
void motors(int leftSpeed, int rightSpeed) {
 
  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}
 
void goBackwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}
 
void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}
 
