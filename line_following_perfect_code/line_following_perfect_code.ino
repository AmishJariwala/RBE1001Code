#include <Arduino.h>
#include <RBE1001Lib.h>

Motor rightMotor;
Motor leftMotor;
const int buttonPin = 0;
const int ledPin = 13;
int count = 0;
const int LinesensorPinLeft = A4;
const int LinesensorPinRight = A3;
const float Kp = -.15;
const int High = 500;
const int low = 200;
const int SetPointRight = 2637;
const int SetPointLeft = 2486;
void setup()
{
Motor::allocateTimer(0);
rightMotor.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
leftMotor.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
rightMotor.setGains(0.35, 0.06, 0);
leftMotor.setGains(0.35, 0.06, 0);
pinMode(buttonPin, INPUT_PULLUP);
pinMode(ledPin, OUTPUT);
pinMode(LinesensorPinLeft, INPUT);
pinMode(LinesensorPinRight, INPUT);
Serial.begin(115200);// This will initialize the Serial as 115200 for prints
}

void turnMotor (int motorSpeed,int distance,int delay_ms) {
    rightMotor.StartMoveFor(distance,motorSpeed);
    leftMotor.MoveFor((-1 * distance),motorSpeed);
    delay(delay_ms);
}


void loop (){
int LinesensorRight = analogRead(LinesensorPinRight);
int LinesensorLeft = analogRead(LinesensorPinLeft);
int errorRight = SetPointRight - LinesensorRight;
int errorLeft = SetPointLeft - LinesensorLeft;

float controlLeft;
float controlRight;
controlRight = errorRight*Kp;
controlLeft = errorLeft*Kp;

Serial.print("right: ");
Serial.print(LinesensorRight);
Serial.print("\t");
Serial.print("left: ");
Serial.println(LinesensorLeft);
rightMotor.SetSpeed(controlLeft);  //rightmotor
leftMotor.SetSpeed(controlRight);  //leftmotor
  
if (controlRight > -300 && controlLeft > -300){
 rightMotor.StartMoveFor(-160, 275);
 leftMotor.MoveFor(-160,275);
 turnMotor (180, 180, 0);
 rightMotor.SetSpeed(0);  //rightmotor
 leftMotor.SetSpeed(0);  //leftmotor
 delay (1000);
  while (1){
 
 }
  } 
}

