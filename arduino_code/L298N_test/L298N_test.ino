void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
}

void loop() {
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  delay(2000);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  delay(1000);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  delay(2000);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  delay(1000);
}
