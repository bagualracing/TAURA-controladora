int a = 10;
int b = 9;
int c = 8;
void setup() {
  // put your setup code here, to run once:
   pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
   digitalWrite(a,LOW);
   digitalWrite(b,LOW);
   digitalWrite(c,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(a,HIGH);
  delay(1000);
  digitalWrite(c,LOW);
  delay(1000);
  digitalWrite(b,HIGH);
  delay(1000);
  digitalWrite(a,LOW);
  delay(1000);
  digitalWrite(c,HIGH);
  delay(1000);
  digitalWrite(b,LOW);
  delay(1000);
}
