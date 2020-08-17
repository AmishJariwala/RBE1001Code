#include <Arduino.h>
#include <RBE1001Lib.h>

Motor motor1;
Motor motor2;
Rangefinder ultrasonic;
const int buttonPin = 0;
const int ledPin = 13;
int count = 0;
float distance = ultrasonic.getDistanceCM();//gets distance of UltraSonic sensor to Objec
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
  pinMode(LinesensorPinLeft, INPUT);
  pinMode(LinesensorPinRight, INPUT);
  Serial.begin(115200);// This will initialize the Serial as 115200 for prints
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO); //sets up Ultrasonic sensor
}
void turnMotor (int motorSpeed,int distance,int delay_ms) {
    motor1.StartMoveFor(distance,motorSpeed);
    motor2.MoveFor((-1 * distance),motorSpeed);
    delay(delay_ms);
}

void loop (){
int LinesensorRight = analogRead(LinesensorPinRight);
Serial.println(LinesensorRight);
int LinesensorLeft = analogRead(LinesensorPinLeft);
Serial.println(LinesensorLeft);
delay(500);
float sum = 2;
float error = 1;
float rightAvgDistance = 3;
float leftAvgDistance = 4;
for (int i = 0; i<=5; i++){
LinesensorRight = analogRead(LinesensorPinRight);
  sum = LinesensorRight + sum;
  rightAvgDistance = sum/5;
  error = rightAvgDistance - error;
}

for (int i = 0; i<=5; i++){
  
  LinesensorRight = analogRead(LinesensorPinRight);
  sum = LinesensorRight + sum;
  leftAvgDistance = sum/5;
  error = leftAvgDistance - 40;
}
Serial.println(rightAvgDistance);
Serial.println(leftAvgDistance);
//Serial.println(distance);
//Serial.println(sum);
//Serial.println(Error);

if(leftAvgDistance<=0){
    motor1.SetSpeed(150*((abs(error)/40.0)));
    motor2.SetSpeed(200*((abs(error)/40.0)));
  }
  else if(leftAvgDistance>0){
  motor1.SetSpeed(200*(1-(abs(error)/40.0)));
  motor2.SetSpeed(200*(1-(abs(error)/40.0)));
  }
else if(rightAvgDistance>=0){
  motor1.SetSpeed(200*(1-(abs(error)/40.0)));
  motor2.SetSpeed(150*(1-(abs(error)/40.0)));
  }
else if(rightAvgDistance>0){
  motor1.SetSpeed(200*(1-(abs(error)/40.0)));
  motor2.SetSpeed(200*(1-(abs(error)/40.0)));
 }
else if(((rightAvgDistance+leftAvgDistance)/2)>0){
 turnMotor(185,185,10); // turn left
 motor1.SetSpeed(0);
 motor2.SetSpeed(0);
 }
else { //if the robot is closer than 50cm to an object the motors stop.
 motor1.SetSpeed(-100);
 motor2.SetSpeed(-100);
 }

  
}

