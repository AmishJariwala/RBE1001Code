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
 
void loop (){
   int adcLinesensorRight = analogRead(LinesensorPinRight);
  Serial.println(adcLinesensorRight);
  int adcLinesensorLeft = analogRead(LinesensorPinLeft);
  Serial.println(adcLinesensorLeft);
 delay(500);
  float sum = 2;
 float Error = 1;
 float avgdistance = 3;
for (int i = 0; i<=5; i++){
  
  distance = ultrasonic.getDistanceCM();
  sum = distance + sum;
  avgdistance = sum/5;
  Error = avgdistance - 40;
}
Serial.println(avgdistance);
//Serial.println(distance);
//Serial.println(sum);
//Serial.println(Error);
if (distance  >= 65 ) { //50 centimeters if statement, if the robot is more than 50 centimeters away from an object it moves
  //if Ultra is mounted in front, there is 15cm between Ultra sensor and front of robot FOR AMISH
  motor1.SetSpeed(-100);
  motor2.SetSpeed(-100);
}
else if(distance<=38){
    motor1.SetSpeed(200*((abs(Error)/40.0)));
    motor2.SetSpeed(200*((abs(Error)/40.0)));
  }
  else if(distance>=42){
  motor1.SetSpeed(-200*(1-(abs(Error)/40.0)));
  motor2.SetSpeed(-200*(1-(abs(Error)/40.0)));
  }

else { //if the robot is closer than 50cm to an object the motors stop.
  motor1.SetEffort(0);
  motor2.SetEffort(0);
  
 }

  
}

