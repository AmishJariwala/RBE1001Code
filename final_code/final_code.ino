//Combining Line sensor, ultrasonic, and RCTLL TELEop
#include <Arduino.h>
#include <RBE1001Lib.h>
#include "Motor.h"
#include "Rangefinder.h"
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>
#include <Esp32WifiManager.h>
#include "wifi/WifiManager.h"
#include "WebPage.h"
#include <Timer.h>
Motor leftMotor;
Motor rightMotor;
const int buttonPin = 0;
const int ledPin = 13;
int count = 0;
const int LinesensorPinLeft = A4;
const int LinesensorPinRight = A3;
//FOR LINE 
const float KpL= -.15;
const int High = 500;
const int low = 200;
const int SetPointRight = 2637;
const int SetPointLeft = 2486;
// FOR ULTRASONIC
Rangefinder ultrasonic;
float distance; //gets distance of UltraSonic sensor to Object
// FOR ULTRASONIC 
const float Kpu = 6;
// https://wpiroboticsengineering.github.io/RBE1001Lib/classRangefinder.html
// FOR RCTLL 
Rangefinder rangefinder1;
Servo lifter;
ESP32AnalogRead leftLineSensor;
ESP32AnalogRead rightLineSensor;
ESP32AnalogRead servoPositionFeedback;
WebPage buttonPage;
WifiManager manager;
Timer dashboardUpdateTimer;  // times when the dashboard should update
bool upDown=false;



//enum state {lineFollow,
//  armMove,
//  turning,
//  teleop,
//};
//

void setup() {    
//RCTLL Setup
  manager.setup();
  while (manager.getState() != Connected) {
    manager.loop();
    delay(1);
  }
Serial.begin(115200);  // This will initialize the Serial as 115200 for prints
Motor::allocateTimer(0); // used by the DC Motors
ESP32PWM::allocateTimer(1);// Used by servos
rightMotor.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
leftMotor.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
rangefinder1      .attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
lifter          .attach(SERVO_PIN);
leftLineSensor      .attach(LEFT_LINE_SENSE);
rightLineSensor     .attach(RIGHT_LINE_SENSE);
servoPositionFeedback   .attach(SERVO_FEEDBACK_SENSOR);
lifter.write(0);
buttonPage.initalize();
dashboardUpdateTimer.reset(); // reset the dashbaord refresh timer
pinMode(buttonPin, INPUT_PULLUP);
pinMode(ledPin, OUTPUT);
pinMode(LinesensorPinLeft, INPUT);
pinMode(LinesensorPinRight, INPUT);
  leftMotor.setGains(0.35, 0.06, 0);
 rightMotor.setGains(0.35, 0.06, 0);
}
void turnMotor (int motorSpeed,int distance,int delay_ms) {
    rightMotor.StartMoveFor(distance,motorSpeed);
    leftMotor.MoveFor((-1 * distance),motorSpeed);
    delay(delay_ms);
}

void statesAreFun (int state = )

void runStateMachine() {

  float left = (buttonPage.getJoystickX()+buttonPage.getJoystickY())*360;
  float right = (buttonPage.getJoystickX()-buttonPage.getJoystickY())*360;

  leftMotor.SetSpeed(left);
  rightMotor.SetSpeed(right);
  lifter.write(buttonPage.getSliderValue(0)*180);
}

uint32_t packet_old=0;
void updateDashboard() {
  if (dashboardUpdateTimer.getMS() > 100) {
    buttonPage.setValue("Left linetracker", leftLineSensor.readMiliVolts());
    buttonPage.setValue("Right linetracker",
        rightLineSensor.readMiliVolts());
    buttonPage.setValue("Ultrasonic",
        rangefinder1.getDistanceCM());
    buttonPage.setValue("Left Motor degrees",
        leftMotor.getCurrentDegrees());
    buttonPage.setValue("Right Motor degrees",
        rightMotor.getCurrentDegrees());
//    buttonPage.setValue("Servo degrees",
//        servoPositionFeedback);

//  Serial.println("Joystick angle="+String(buttonPage.getJoystickAngle())+
//    " magnitude="+String(buttonPage.getJoystickMagnitude())+
//    " x="+String(buttonPage.getJoystickX())+                  " y="+String(buttonPage.getJoystickY()) +
//      " slider="+String(buttonPage.getSliderValue(0)));

    dashboardUpdateTimer.reset();
  }
}

void Ultrasonic!(){
 delay(30);
 distance = ultrasonic.getDistanceCM();
  float sum = 2;
 float error = 1;
 float avgdistance = 3;
 error = distance - 40;

for (int i = 0; i<=7; i++){
  delay (50);
  distance = ultrasonic.getDistanceCM();
  sum = distance + sum;
  avgdistance = sum/7;
}

Serial.println(avgdistance);
//Serial.println(distance);
//Serial.println(sum);
//Serial.println(Error);
if (avgdistance   >= 100 ) { //if the robot is more then a meter away from the wall, gofullspeed 

  leftMotor.SetSpeed(-360);
  rightMotor.SetSpeed(-360);
}
 else {
      leftMotor.SetSpeed(-(error)*Kpu);
      rightMotor.SetSpeed(-(error)*Kpu);
  }

}
void Linesensor!(){
  int LinesensorRight = analogRead(LinesensorPinRight);
  int LinesensorLeft = analogRead(LinesensorPinLeft);
  int errorRight = SetPointRight - LinesensorRight;
  int errorLeft = SetPointLeft - LinesensorLeft;
  
  //float controlLeft;
  //float controlRight;
  float controlRight = errorRight*KpL;
  float controlLeft = errorLeft*KpL;
  
  //Serial.print("right: ");
  //Serial.print(controlRight);
  //Serial.print("\t");
  //Serial.print("left: ");
  //Serial.println(controlLeft);
  
  rightMotor.SetSpeed(controlLeft);  //rightmotor
  leftMotor.SetSpeed(controlRight);  //leftmotor
    
  if (controlRight > -300 && controlLeft > -300){
    rightMotor.StartMoveFor(-160, 275);
    leftMotor.MoveFor(-160,275);
    turnMotor (180, 180, 0);
    rightMotor.SetSpeed(0);  //rightmotor
    leftMotor.SetSpeed(0);  //leftmotor
    }
}
void loop() {
//RCTLL
    float left = (buttonPage.getJoystickX()+buttonPage.getJoystickY())*360;
    float right = (buttonPage.getJoystickX()-buttonPage.getJoystickY())*360;
    leftMotor.SetSpeed(left);
    rightMotor.SetSpeed(right);
    lifter.write(buttonPage.getSliderValue(0)*180);
Ultrasonic!();
Linesensor!();
}




