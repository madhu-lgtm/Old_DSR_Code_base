unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
unsigned long dwell = 2000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  curr_millis = millis();
  if ((curr_millis - prev_millis) >= dwell)
  {
    Serial.println("Disable Driver");
    prev_millis = curr_millis;
  }


}
