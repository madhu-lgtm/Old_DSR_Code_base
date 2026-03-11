#define ENCODER_PPR 10 

const int EN_A_1 = 2;  //  external interrupt
const int EN_A_2 = 3;
const int EN_A_3 = 18;
const int EN_A_4 = 19;
const int EN_A_5 = 20;

int interval = 1000; //  interval in ms

volatile long encoderValue_1 = 0;
volatile long encoderValue_2 = 0;
volatile long encoderValue_3 = 0;
volatile long encoderValue_4 = 0;
volatile long encoderValue_5 = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

float rpm_1 = 0;
float rpm_2 = 0;
float rpm_3 = 0;
float rpm_4 = 0;
float rpm_5 = 0;

float gr = 0.0103;
//2, 3, 18, 19, 20, and 21(MEGA Interrupts)


void setup() 
{
  Serial.begin(57600);
  
  attachInterrupt(digitalPinToInterrupt(EN_A_1), updateEncoder_1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_A_2), updateEncoder_2, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_A_3), updateEncoder_3, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_A_4), updateEncoder_4, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_A_5), updateEncoder_5, RISING);

   encoderValue_1 = 0;
   encoderValue_2 = 0;
   encoderValue_3 = 0;
   encoderValue_4 = 0;
   encoderValue_5 = 0;

   previousMillis = millis();
}

void loop()
{
  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    
    // Revolutions per minute (RPM) = (total encoder pulse in 1s / encoder ppr) x 60s
    rpm_1 = ((float)encoderValue_1 * 60.0 / ENCODER_PPR)*gr;
    rpm_2 = ((float)encoderValue_2 * 60.0 / ENCODER_PPR)*gr;
    rpm_3 = ((float)encoderValue_3 * 60.0 / ENCODER_PPR)*gr;
    rpm_4 = ((float)encoderValue_4 * 60.0 / ENCODER_PPR)*gr;
    rpm_5 = ((float)encoderValue_5 * 60.0 / ENCODER_PPR)*gr;


      Serial.print("M1 = ");
      Serial.print(rpm_1);
      Serial.print(" , M2 = ");
      Serial.print(rpm_2);
      Serial.print(" , M3 = ");
      Serial.print(rpm_3);
      Serial.print(" , M4 = ");
      Serial.print(rpm_4);
      Serial.print(" , M5 = ");
      Serial.println(rpm_5);
      
  
    encoderValue_1 = 0;
    encoderValue_2 = 0;
    encoderValue_3 = 0;
    encoderValue_4 = 0;
    encoderValue_5 = 0;

  }
}

void updateEncoder_1()
{
  encoderValue_1++;
}

void updateEncoder_2()
{
  encoderValue_2++;
}

void updateEncoder_3()
{
  encoderValue_3++;
}

void updateEncoder_4()
{
  encoderValue_4++;
}

void updateEncoder_5()
{
  encoderValue_5++;
}


