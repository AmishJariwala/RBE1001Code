const int ledPin = 13;

// the setup routine runs once at the start of the program:
void setup() 
{                
  pinMode(ledPin, OUTPUT);         // initialize the digital pin as an output.
}

// the loop routine runs over and over:


    
void loop() { 
  for (int m = 1000; m > 0; m = m + 1000)
  {
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(m);                  // wait for a second
    digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                  // wait for a second
  
  }
}
