
// Tarpaulin Dropping

// Pins And Connections 
int in1_pin = 3;
int in2_pin = 4;
int sig_read_pin = 5; //d5 on Nano
int sig_val ;
int sig_min = 1051;
int sig_trim = 1501;
int sig_max = 1951;
int sig_buffer = 100;
int dt = 200;


void setup() {
  // put your setup code here, to run once:
  pinMode(in1_pin,OUTPUT);
  pinMode(in2_pin,OUTPUT);
  pinMode(sig_read_pin,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sig_val = pulseIn(sig_read_pin,HIGH);
  Serial.println(sig_val);
  if(sig_val >= (sig_min-sig_buffer) && sig_val <= (sig_trim - sig_buffer))
  {
    digitalWrite(in1_pin,LOW);
    digitalWrite(in2_pin,HIGH);
  }
  else if(sig_val >= (sig_trim - sig_buffer) && sig_val <= (sig_trim + sig_buffer))
  {
    digitalWrite(in1_pin,HIGH);
    digitalWrite(in2_pin,HIGH);
  }
  else if(sig_val > (sig_trim + sig_buffer) && sig_val < (sig_max + sig_buffer) )
  {
    digitalWrite(in1_pin,HIGH);
    digitalWrite(in2_pin,LOW);

  }
  else
  {
    digitalWrite(in1_pin,HIGH);
    digitalWrite(in2_pin,HIGH);
  }
  delay(dt);

}
