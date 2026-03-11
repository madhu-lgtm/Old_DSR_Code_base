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

long previousMillis = 0;
long currentMillis = 0;

int rpm_1 = 0;
int rpm_2 = 0;
int rpm_3 = 0;
int rpm_4 = 0;
int rpm_5 = 0;

float gr = 0.0103;
//2, 3, 18, 19, 20, and 21(MEGA Interrupts)


void setup() 
{
  Serial.begin(57600);
  
  attachInterrupt(digitalPinToInterrupt(EN_A), updateEncoder, RISING);
  
   encoderValue = 0;
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
    rpm_1 = (float)((encoderValue_1 * 60 / ENCODER_PPR)*gr);
    rpm_2 = (float)((encoderValue_2 * 60 / ENCODER_PPR)*gr);
    rpm_3 = (float)((encoderValue_3 * 60 / ENCODER_PPR)*gr);
    rpm_4 = (float)((encoderValue_4 * 60 / ENCODER_PPR)*gr);
    rpm_5 = (float)((encoderValue_5 * 60 / ENCODER_PPR)*gr);


      Serial.print("M1 = ");
      Serial.print(RPM_1);
      Serial.print(" , M2 = ");
      Serial.print(RPM_2);
      Serial.print(" , M3 = ");
      Serial.print(RPM_3);
      Serial.print(" , M4 = ");
      Serial.print(RPM_4);
      Serial.print(" , M5 = ");
      Serial.println(RPM_5);
      
  
    encoderValue_1 = 0;
    encoderValue_2 = 0;
    encoderValue_3 = 0;
    encoderValue_4 = 0;
    encoderValue_5 = 0;

  }
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from encoder A phase
  encoderValue_1++;
  encoderValue_2++;
  encoderValue_3++;
  encoderValue_4++;
  encoderValue_5++;
}
