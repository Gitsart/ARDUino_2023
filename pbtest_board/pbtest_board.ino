

void setup() {
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  Serial.begin(9600);

}

void loop() {
  while (digitalRead(2 ==LOW)){
    Serial.println("BACKPB");
  }

}
