
#include <Arduino.h>
#include <Motor.h>
#include <RBE1001Lib.h>
Motor motor1;
Motor motor2;
const int buttonPin = 0;
const int forward = 0;
const int left = 1;
const int right = 2;
const int back = 3;
void setup()
{
  Serial.begin(115200);
  Motor::allocateTimer(0);
  motor1.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
  motor2.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
  pinMode(buttonPin, INPUT_PULLUP);
}
int x = 0;
void wheelmove(int x, int dir) {
  if (dir==left) {
    motor1.StartMoveFor(x, 60);
    motor2.MoveTo(x * -1, 60);
  }
  else if(dir== right){
    motor1.StartMoveFor(-1, 60);
    motor2.MoveTo(x, 60);
  }
  else if(dir==back){
    motor1.StartMoveFor(-1, 60);
    motor2.MoveTo(x * -1, 60);
  }
  else motor2.StartMoveFor(x, 60);{
    motor2.MoveTo(x, 60);
  }
}
void loop()
{
  while (digitalRead(buttonPin)) {
    for (int z = 0 ; z == 4; z++) {
      wheelmove(360,forward);
      wheelmove(166,left);
    }
    for (int z = 0 ; z == 5; z++) {
      wheelmove(360,forward);
      wheelmove(597,left);
    }{
    wheelmove(968, forward);
    wheelmove(166,left);
    wheelmove(645, forward);
    wheelmove(166, right);
    wheelmove(323, forward);
    wheelmove(166, right);
    wheelmove(645, forward);
    }
  }
}


