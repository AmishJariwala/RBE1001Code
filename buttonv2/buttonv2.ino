//Amish Jariwala's code for Intro to Microcontrollers 7/16/20
//What this code does is to: do nothing when the button on the controller is not pressed/reset the onTime variable, and when pressed the program initiates a loop. This for loop uses the onTime variable, which stands for how long the led light will stay on. This loop is set to always run(as long as the button is pressed) and adds 1 second(to onTime) per run through the for loop.

const int buttonPin = 0;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int onTime = 1000; //sets the variable of onTime to 1000 at the begining of the code

void setup() 
{
  pinMode(ledPin, OUTPUT);        // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT);      // initialize the pushbutton pin as an input:
}



void loop() //loop constantly running to detect if the button is pressed
{
  // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  if (buttonState == LOW) // if it is, the buttonState is LOW, running a loop turing the light on for 1 sec and off for 1, then on 2, off 1...etc
  {     
    digitalWrite(ledPin, HIGH);   // turn the LED on 
    delay(onTime);                  // wait for a linear growth of seconds starting at one
    digitalWrite(ledPin, LOW);    // turn the LED off 
    delay(1000);                  // wait for a second
    onTime = onTime + 1000; //adds one second at end of loop to onTime        
  }
  else //Turns LED off and resets the onTime variable so that the next time the button is pressed, the timing resets.
  {
    digitalWrite(ledPin, LOW);  // turn LED off:
    onTime = 1000; //resets the onTime
  }
}


/*******************************************************************************************************************************************
 * Original code for the blinking light.
void loop() { 
  for (int onTime = 1000; onTime > 0; onTime = onTime + 1000) //creates onTime variable, sets to 1000, adds 1000 every time onTime>0 (always)
  {
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(onTime);                  // wait for a second
    digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                  // wait for a second
  
  }
}
*********************************************************************************************************************************************/ 


