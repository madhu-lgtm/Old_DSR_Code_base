int pwmpin = 9;
int dt = 500;
int pwmval;
void setup() {
  // put your setup code here, to run once:
  pinMode(pwmpin,INPUT);
  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  pwmval = pulseIn(pwmpin,HIGH);
  Serial.println(pwmval);
  delay(dt);

}
