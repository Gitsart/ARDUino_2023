/*  Example for MobaTools
    Moving a stepper back and forth
*/
#include <MobaTools.h>

// Adjust pins, steps and time as needed
const byte stepPin = 22;
const byte dirPin  = 24;
const int stepsPerRev = 200;   // Steps per Revolution ( example with 1/4 microsteps )
const long  targetPos = 1600;         // stepper moves between 0 and targetpos
long nextPos=0;


MoToStepper myStepper ( stepsPerRev, STEPDIR ); // instance of motor
MoToTimer stepperPause;                    // Pause between stepper moves
bool stepperRunning;

void setup() {
  myStepper.attach( stepPin, dirPin );
  myStepper.setSpeed( 600 );  // 60 Rev/Min ( if stepsPerRev is set correctly )
  myStepper.setRampLen( 200 );
  stepperRunning = true; //default when pgm starts
  Serial.begin(9600);

  
}

void loop() {
 
  while(!myStepper.moving()==true){
    Serial.println("stopped");
    delay(5000);
    nextPos+=400;
    myStepper.moveTo(nextPos );
  }
  
  
  while(myStepper.moving()==true){
    Serial.println("motorstoppping this will print for 2 sec");
  }

  // The sketch is not blocked while the stepper is moving nor while it is stopped.
  // Other nonblocking  tasks can be added here
}
