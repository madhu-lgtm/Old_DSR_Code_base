/*
External PullDown Resistor

              (Dx) o/p = 0     Connect to Arduino Gnd 
                |                |  if connected to 
   ______/ _____|____R10K________|  external power supply
  |                              |
  |                              |
  |                              |
  |_________ + 5V -______________|
   Arduino Power/ External Power

*/



int buttonPin = 11;
int buttonVal=0;
int buzzPin = 12;

void setup()
{
  pinMode(buttonPin,INPUT);// remove comments if using extenal resistots (external pullup or external pulldown)
  pinMode(buzzPin,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(buzzPin,digitalRead(buttonPin));
  delay(100);
}