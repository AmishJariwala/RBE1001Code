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
  Motor rightMotor;
  Motor leftMotor;
  const int buttonPin = 0;
  const int ledPin = 13;
  int count = 0;
  const int LinesensorPinLeft = A4;
  const int LinesensorPinRight = A3;
  //FOR LINE 
  const float KpL= -.28;
  const int High = 500;
  const int low = 200;
  const int SetPointRight = 2637;
  const int SetPointLeft = 2486;
  //// FOR ULTRASONIC
  float distance; //gets distance of UltraSonic sensor to Object
  // FOR ULTRASONIC 
  const float Kpu = 6;
  // FOR RCTLL 
  Rangefinder ultrasonic;
  Servo lifter;
  ESP32AnalogRead leftLineSensor;
  ESP32AnalogRead rightLineSensor;
  ESP32AnalogRead servoPositionFeedback;
  WebPage buttonPage;
  WifiManager manager;
  Timer dashboardUpdateTimer;  // times when the dashboard should update
  bool upDown=false;
  int state =2;
 // int state =21; // to start at teleop

//setup
  void setup() {    
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
      ultrasonic      .attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
        Serial.println ("b4lifter");
      lifter          .attach(SERVO_PIN);
      Serial.println ("afterlifter");
      leftLineSensor      .attach(LEFT_LINE_SENSE);
      rightLineSensor     .attach(RIGHT_LINE_SENSE);
      servoPositionFeedback   .attach(SERVO_FEEDBACK_SENSOR);
      pinMode(buttonPin, INPUT_PULLUP);
      pinMode(ledPin, OUTPUT);
      pinMode(LinesensorPinLeft, INPUT);
      pinMode(LinesensorPinRight, INPUT);
      leftMotor.setGains(0.45, 0.06, 0);
      rightMotor.setGains(0.45, 0.06, 0);
      int motorDelay = 10;
      buttonPage.initalize();
      dashboardUpdateTimer.reset(); // reset the dashboard refresh timer
      Serial.println("done with setup");
  }

  //FUNCTIONS
  //turning functions - one slow one normal (about 60% speed and 85%)
    void turn (int degreeMotorTurn) {
      int motorInput = degreeMotorTurn*2;
      rightMotor.StartMoveFor(motorInput,300);
      leftMotor.MoveFor((-1 * motorInput),300);
      delay (10);
      state++;
    }
    void turnSLOW (int degreeMotorTurn) {
      int motorInput = degreeMotorTurn*2;
      rightMotor.StartMoveFor(motorInput,200);
      leftMotor.MoveFor((-1 * motorInput),200);
      delay (10);
      state++;
    }
    
  //line follows
    void lineFollow () {
     int LinesensorRight = analogRead(LinesensorPinRight);
      int LinesensorLeft = analogRead(LinesensorPinLeft);
      int errorRight = SetPointRight - LinesensorRight;
      int errorLeft = SetPointLeft - LinesensorLeft;
      
    float controlLeft;
    float controlRight;
    controlRight = errorRight*KpL;
    controlLeft = errorLeft*KpL;
    
      rightMotor.SetSpeed(controlRight);  //rightmotor
      leftMotor.SetSpeed(controlLeft);  //leftmotor
        
      if (controlRight > -300 && controlLeft > -300){ // increment state when an intersection is reached    rightMotor.SetSpeed(0);  //rightmotor
        Serial.println ("INTERSECTION");
        rightMotor.StartMoveFor(-145, 275);
        leftMotor.MoveFor(-145,275);
        leftMotor.SetSpeed(0);  //leftmotor
        rightMotor.SetSpeed(0);
     state++;
        }
    }
    
  //line follows without moving forward at the end
    void lineFollowDrop () {
      //Serial.println ("STARTING LINE FOLLOW");
     int LinesensorRight = analogRead(LinesensorPinRight);
      int LinesensorLeft = analogRead(LinesensorPinLeft);
      int errorRight = SetPointRight - LinesensorRight;
      int errorLeft = SetPointLeft - LinesensorLeft;
      
    float controlLeft;
    float controlRight;
    controlRight = errorRight*KpL;
    controlLeft = errorLeft*KpL;
    
    rightMotor.SetSpeed(controlRight);  //rightmotor
    leftMotor.SetSpeed(controlLeft);  //leftmotor
    
      if (controlRight > -300 && controlLeft > -300){ // increment state when an intersection is reached    rightMotor.SetSpeed(0);  //rightmotor
        Serial.println ("INTERSECTION");
        leftMotor.SetSpeed(0);  //leftmotor
        rightMotor.SetSpeed(0);
     state++;
        }
    
    }
    
    
  //pick up can function
    void pickUpCan() {
      Serial.print ("picking up can");
     upDown=!upDown;
      int loopTime = 4000;// 4 second loop
      int servoRange =180;
    //lower arm
      Serial.print ("lower");
      lifter.write(0);
    //move forward 
      Serial.print ("forward");
       rightMotor.StartMoveFor(180,180);
        leftMotor.MoveFor(180,180);
    // raise arm
      Serial.print ("raising");
      lifter.write(180);
    //move backward a tiny bit
      Serial.print ("goingback");
        rightMotor.StartMoveFor(-90,180);
       leftMotor.MoveFor(-90,80);
    state++;
    }
    
    
  //lowering arm functions
    void lowerArm(){
      lifter.write(0);
    delay (500);
    state++;
    }
    
      void lowerArm2(){
      lifter.write(50);
    delay (500);
    state++;
    }
    
    
  //function for the RCTLL TELEOP
    void runStateMachine() {
    
      float left = (buttonPage.getJoystickX()+buttonPage.getJoystickY())*360;
      float right = (buttonPage.getJoystickX()-buttonPage.getJoystickY())*360;
    
      leftMotor.SetSpeed(left);
      rightMotor.SetSpeed(right);
      lifter.write(buttonPage.getSliderValue(0)*180);
    }
    
    
  //function for the RCTLL TELEOP
    uint32_t packet_old=0;
    void updateDashboard() {
        if (dashboardUpdateTimer.getMS() > 100) {
          buttonPage.setValue("Left linetracker", leftLineSensor.readMiliVolts());
          buttonPage.setValue("Right linetracker",
              rightLineSensor.readMiliVolts());
          buttonPage.setValue("Left Motor degrees",
              leftMotor.getCurrentDegrees());
          buttonPage.setValue("Right Motor degrees",
              rightMotor.getCurrentDegrees());
              
          dashboardUpdateTimer.reset();
        }
    }
    
  //ULTRASONIC (NOT USED)
    //void Ultrasonic (){
    // distance = ultrasonic.getDistanceCM();
    //  float sum = 2;
    // float error = 1;
    // float avgdistance = 3;
    // error = distance - 20;
    //
    //for (int i = 0; i<=7; i++){
    //  delay (50);
    //  distance = ultrasonic.getDistanceCM();
    //  sum = distance + sum;
    //  avgdistance = sum/7;
    //}
    //
    ////Serial.println(avgdistance);
    ////Serial.println(distance);
    ////Serial.println(sum);
    ////Serial.println(Error);
    //if (avgdistance   >= 100 ) { //if the robot is more then a meter away from the wall, gofullspeed 
    //
    //  leftMotor.SetSpeed(-360);
    //  rightMotor.SetSpeed(-360);
    //}
    // else {
    //      leftMotor.SetSpeed(-(error)*Kpu);
    //      rightMotor.SetSpeed(-(error)*Kpu);
    //  }
    //
    //}
  
  //scanning (NOT USED)
      void drive (int motorInput) {
        rightMotor.StartMoveFor(motorInput,288);
        leftMotor.MoveFor((motorInput),288);
        delay (10);
      }
      
    
      void searchPattern () {
          int length = 236.25; // about 25 cm in degrees 
          int adjust = 15;
          int distanceFromCan = ultrasonic.getDistanceCM();
          int n=1;
          int countPH = 47;
          turn (-90);
          distanceFromCan = ultrasonic.getDistanceCM();
          
          drive(length); 
          distanceFromCan = ultrasonic.getDistanceCM();
          
          turn (90);
          distanceFromCan = ultrasonic.getDistanceCM();
          drive(length+adjust); 
          
          while(distanceFromCan > 20)  {
          
          turn (90);
          distanceFromCan = ultrasonic.getDistanceCM();
          drive(countPH); 
          distanceFromCan = ultrasonic.getDistanceCM();
           turn (90);   
              distanceFromCan = ultrasonic.getDistanceCM();
          
          drive(length+adjust); 
              distanceFromCan = ultrasonic.getDistanceCM();
          
           turn (-90);   
              distanceFromCan = ultrasonic.getDistanceCM();
          
          drive(countPH); 
              distanceFromCan = ultrasonic.getDistanceCM();
             turn  (-90);
              distanceFromCan = ultrasonic.getDistanceCM();
          
          drive(length+adjust); 
              distanceFromCan = ultrasonic.getDistanceCM();
          
          }
          turn(180);
          turn(180);
          state = 21;
      }
    
  //runs teleop
    void RCTLLloop(){
      manager.loop();
        runStateMachine();  // do a pass through the state machine
        if(manager.getState() == Connected){// only update if WiFi is up
          updateDashboard();  // update the dashboard values
      }
    }

//states function
  void statesAreFun ()  {// sequential state machine
    switch (state){
       // case 1: idle(); break; // wait for the button to be pressed
       case 2: Serial.print (state); pickUpCan(); break;      //first can on line
       case 3: Serial.print (state); lineFollow();break;      //follow to intersection
       case 4: Serial.print (state); turnSLOW(-90);break;     //turn left
       case 5:Serial.print (state); lineFollowDrop();break;   //follow to drop off
       case 6: Serial.print (state);turnSLOW(200);break;      //turn around
       case 7:Serial.print (state);lowerArm(); break;         //drops first can
       case 8: Serial.print (state);lineFollow();break;       //follows to intersection from dropoff
       case 9: Serial.print (state);turn(90);break;           // turn right
       case 10: Serial.print (state);lineFollowDrop();break;  // go to 2nd can
       case 11: Serial.print (state);lineFollowDrop();break; // go to 2nd can
       case 12: Serial.print (state);turn(190);break;         //turn around
       case 13: Serial.print (state); pickUpCan(); break;      //2nd can on line
       case 14: Serial.print (state);lineFollowDrop();break;  //tiny bit between 2nd can and line follow
       case 15:Serial.print (state);lineFollow();delay (100); break; //to intersection
       case 16: Serial.print (state);lineFollowDrop();break;  //to dropoff
       case 17: Serial.print (state);turnSLOW(195);break;     //turn around
       case 18: Serial.print (state);lowerArm2();break;       //drop off second can(requires second function bc higher platform(lower arm less))
       case 19: Serial.print (state);lineFollow();  break;    //from dropoff to intersection
       case 20: Serial.print (state);lineFollowDrop();break;  //to start position
      //case 21: Serial.print (state);searchPattern();break;
      //case 22: Serial.print (state);pickUpCan();break;
      case 21: Serial.print (state); RCTLLloop(); break;      //into teleop!!!
    }
  }


void loop() {
statesAreFun () ;

}

