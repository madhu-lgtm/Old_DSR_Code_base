int read_pin = 9;
int pwm ;
void setup() {
  // put your setup code here, to run once:
  pinMode(read_pin, INPUT);
  Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:
  pwm = pulseIn(read_pin,HIGH);
  Serial.println(pwm);
}
