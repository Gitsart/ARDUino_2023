int Start_pb = 2;
int Emergency = 3;
int u_turn = A1;


void setup() {
  pinMode(Emergency, INPUT_PULLUP);
  pinMode(Start_pb, INPUT_PULLUP);
  pinMode(u_turn, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.print("START");

}

void loop() {
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(A1,HIGH);
  if(digitalRead(Start_pb == LOW)) {Serial.println("FORWARD");}
  else if(digitalRead(u_turn ==LOW)) { Serial.println("UTURN");}
  else if(digitalRead(Emergency ==LOW)) { Serial.println("EMERGENCY");}
  else {Serial.println("ERROR");}
  }
