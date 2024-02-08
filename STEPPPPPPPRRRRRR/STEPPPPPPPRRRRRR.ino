const int stepPin = 22;
const int dirPin = 24;
const int enPin = 26;
const int homeSwitchPin = 9; // Connect a home position switch to this pin
const int homingInterval = 360; // Define the homing interval in steps

int stepCount = 0;

bool homingCompleted = false;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); // Motor active
  pinMode(homeSwitchPin, INPUT_PULLUP); // Home position switch with pull-up resistor

  Serial.begin(9600);
  Serial.print("START");

  homing(); // Run homing routine to set the home position
}

void loop() {
  if (!homingCompleted) {
    homing(); // Run homing routine if not completed
  } else {
    // Your main code logic goes here
    // For example, rotate the motor clockwise for 360 steps
    rotateClockwise(360);
    delay(1000);

    // Rotate the motor counterclockwise for 360 steps
    rotateCounterClockwise(360);
    delay(1000);
  }
}

void rotateClockwise(int numSteps) {
  digitalWrite(dirPin, HIGH); // Enables the motor to move in a particular direction
  for (int x = 0; x < numSteps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10000);
    stepCount++;
    Serial.print("Clockwise:");
    Serial.println(stepCount);
  }
}

void rotateCounterClockwise(int numSteps) {
  digitalWrite(dirPin, LOW); // Changes the direction of rotation
  for (int x = 0; x < numSteps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10000);
    stepCount--;
    Serial.print("CounterClockwise:");
    Serial.println(stepCount);
  }
}

void homing() {
  // Move the motor within the homing interval until the home switch is pressed
  while (digitalRead(homeSwitchPin) == HIGH ) {
    digitalWrite(dirPin, HIGH); // Enables the motor to move in a particular direction
    for (int x = 0; x < homingInterval; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10000);
    stepCount++;}
    digitalWrite(dirPin, LOW); // Changes the direction of rotation
    for (int x = 0; x < homingInterval; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10000);
    stepCount--;}

    Serial.print("Homing:");
    Serial.println(stepCount);
  }

  // Set the homingCompleted flag to true once homing is finished
  homingCompleted = true;

  // Move the motor a few steps in the clockwise direction to settle at the home position
  rotateClockwise(10);
}
