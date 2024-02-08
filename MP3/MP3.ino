void setup() {
  Serial.begin(9600);
  Serial2.begin(9600); // DFPlayer Mini default baud rate
}

void loop() {
  // Send play command
  Serial2.write(0xAA);
  Serial2.write(0x37);
  Serial2.write(0x00);
  Serial2.write(0x01);
  Serial2.write(0x00);
  Serial2.write(0x01);
  Serial2.write(0x00);
  Serial2.write(0x05);
  Serial2.write(0xEF);

  delay(5000); // Adjust delay as needed

  // Send stop command
  Serial2.write(0xAA);
  Serial2.write(0x37);
  Serial2.write(0x00);
  Serial2.write(0x01);
  Serial2.write(0x00);
  Serial2.write(0x01);
  Serial2.write(0x00);
  Serial2.write(0x06);
  Serial2.write(0xEF);

  while (1); // Stop here, you can modify as needed
}
