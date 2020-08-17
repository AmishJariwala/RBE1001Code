//RBE FINAL PROJECT CODE GROUP 9
//obstacle/ line code - https://docs.google.com/document/d/10BRL98603Dk3H1-oiZiwWvU-U_vGfEIT0DE4adzjYzI/edit?usp=sharing
//project overview - https://wpi.instructure.com/courses/20563/files/2903950/download?wrap=1
//slides for design - https://wpi.instructure.com/courses/20563/files/2904017/download?wrap=1


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
int state =2;

void setup() {    
Serial.println ("starting serial print");
//RCTLL Setup
//  manager.setup();
// while (manager.getState() != Connected) {
//   manager.loop();
//   delay(1);
//  }
Serial.begin(115200);  // This will initialize the Serial as 115200 for prints
Motor::allocateTimer(0); // used by the DC Motors
ESP32PWM::allocateTimer(1);// Used by servos
rightMotor.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
leftMotor.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
rangefinder1      .attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  Serial.println ("b4lifter");
lifter          .attach(SERVO_PIN);
Serial.println ("afterlifter");
leftLineSensor      .attach(LEFT_LINE_SENSE);
rightLineSensor     .attach(RIGHT_LINE_SENSE);
servoPositionFeedback   .attach(SERVO_FEEDBACK_SENSOR);
////buttonPage.initalize();
////dashboardUpdateTimer.reset(); // reset the dashboard refresh timer
pinMode(buttonPin, INPUT_PULLUP);
pinMode(ledPin, OUTPUT);
pinMode(LinesensorPinLeft, INPUT);
pinMode(LinesensorPinRight, INPUT);
leftMotor.setGains(0.35, 0.06, 0);
 rightMotor.setGains(0.35, 0.06, 0);
int motorDelay = 10;
Serial.println("done with setup");
}









//Functions
void turn (int degreeMotorTurn) {
int motorInput = degreeMotorTurn*2;
rightMotor.StartMoveFor(motorInput,288);
leftMotor.MoveFor((-1 * motorInput),288);
delay (10);
state++;
}
//void idle () {
//if (buttonPin == HIGH) {
//state++;
//}
//}

void lineFollow () {
  Serial.println ("STARTING LINE FOLLOW");
  delay(1000);
 int LinesensorRight = analogRead(LinesensorPinRight);
  int LinesensorLeft = analogRead(LinesensorPinLeft);
  int errorRight = SetPointRight - LinesensorRight;
  int errorLeft = SetPointLeft - LinesensorLeft;
  
  float controlRight = errorRight*KpL;
  float controlLeft = errorLeft*KpL;

  rightMotor.SetSpeed(controlLeft);  //rightmotor
  leftMotor.SetSpeed(controlRight);  //leftmotor
distance = ultrasonic.getDistanceCM(); 
    
  if (controlRight > -300 && controlLeft > -300){ // increment state when an intersection is reached    rightMotor.SetSpeed(0);  //rightmotor
    rightMotor.StartMoveFor(-160, 275);
  leftMotor.MoveFor(-160,275);
    leftMotor.SetSpeed(0);  //leftmotor
    rightMotor.SetSpeed(0);
state++;

    }
else if (distance == 10){
int error = distance - 5;
      leftMotor.SetSpeed(-(error)*Kpu);
      rightMotor.SetSpeed(-(error)*Kpu);
state++;
}
}



void pickUpCan() {
  Serial.print ("picking up can");
 upDown=!upDown;
  int loopTime = 4000;// 4 second loop
  int servoRange =180;
//lower arm
  Serial.print ("lower");
  lifter.write(150);
//move forward 
  Serial.print ("forward");
   rightMotor.StartMoveFor(-360,60);
    leftMotor.MoveFor(-360,60);
// raise arm
  Serial.print ("raising");
  lifter.write(10);

//move backward
  Serial.print ("goingback");
     rightMotor.StartMoveFor(360,60);
    leftMotor.MoveFor(360,60);
state++;
}


void lowerArm(){
  lifter.write(50);
}




//RCTLL stuff
//  float left = (buttonPage.getJoystickX()+buttonPage.getJoystickY())*360;
//  float right = (buttonPage.getJoystickX()-buttonPage.getJoystickY())*360;
//
//  leftMotor.SetSpeed(left);
//  rightMotor.SetSpeed(right);
//  lifter.write(buttonPage.getSliderValue(0)*180);
//uint32_t packet_old=0;


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

void Ultrasonic (){
 distance = ultrasonic.getDistanceCM();
  float sum = 2;
 float error = 1;
 float avgdistance = 3;
 error = distance - 20;

for (int i = 0; i<=7; i++){
  delay (50);
  distance = ultrasonic.getDistanceCM();
  sum = distance + sum;
  avgdistance = sum/7;
}

//Serial.println(avgdistance);
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

 
void statesAreFun ()  {// sequential state machine
switch (state){
 // case 1: idle(); break; // wait for the button to be pressed
  case 2: Serial.print ("2"); pickUpCan(); break;   
  case 3: Serial.print ("3"); lineFollow();break;
 case 4: Serial.print ("4"); turn(-90);break; 
 case 5:Serial.print ("5"); lineFollow();break;
 case 6:Serial.print ("6");pickUpCan(); break; 
 case 7: Serial.print ("7");turn(180);break;
 case 8: Serial.print ("8");lineFollow();break;
  case 9: Serial.print ("9");turn(90);break;
  case  10: Serial.print ("10");lineFollow ();break; // this might need to be a slightly different function
//  case 12: scan(); break;
}
}

/*
){            //if the ultrasonic sensor sees a platform, increment the state

}
}
void teleop!(){
  float left = (buttonPage.getJoystickX()+buttonPage.getJoystickY())*360;
    float right = (buttonPage.getJoystickX()-buttonPage.getJoystickY())*360;
    leftMotor.SetSpeed(left);
    rightMotor.SetSpeed(right);
    lifter.write(buttonPage.getSliderValue(0)*180);
}
int locate(){
}


*/

void loop() {
delay (5000);
statesAreFun () ;

}

