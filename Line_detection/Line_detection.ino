#include <Arduino.h>
#include <RBE1001Lib.h>

Motor motor1;
Motor motor2;
Rangefinder ultrasonic;
const int buttonPin = 0;
const int ledPin = 13;
int count = 0;
const int LinesensorPinLeft = A4;
const int LinesensorPinRight = A3;

void setup()
{
  Motor::allocateTimer(0);
  motor1.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
  motor2.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
  motor1.setGains(0.35, 0.06, 0);
 motor2.setGains(0.35, 0.06, 0);
  pinMode(buttonPin, INPUT_PULLUP);
 pinMode(ledPin, OUTPUT);
  Serial.begin(115200);// This will initialize the Serial as 115200 for prints
  
  //Pins typically default to INPUT, but the code reaads easier if you are explicit:
  pinMode(LinesensorPinLeft, INPUT);
  pinMode(LinesensorPinRight, INPUT);r
}
 
void loop (){
  int adcLinesensorRight = analogRead(LinesensorPinRight);
  Serial.println(adcLinesensorRight);
  int adcLinesensorLeft = analogRead(LinesensorPinLeft);
  Serial.println(adcLinesensorLeft);
  delay(500);
  float line = (adcLinesensorLeft + adcLinesensorRight)/2;
  
if (line >= 50) { //50 centimeters if statement, if the robot is more than 50 centimeters away from an object it moves
  motor1.SetSpeed(30);
  motor2.SetSpeed(30);
}
 else { //if the robot is closer than 50cm to an object the motors stop.
  motor1.SetEffort(0);
  motor2.SetEffort(0);
  
 }
}
