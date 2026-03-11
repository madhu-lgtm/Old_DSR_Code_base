
// Single Module Test :- 28-01-2025
// Implemented RPM Filter for single module :- 05-02-2025
// extended filter 
// blockage function






// Updated Code For Error in 2min:30 Sec

/* 100 RPM ACTIVE : JGB 37 - 520 - 12V - 100R 
// Included 12v SONAL Buzzer with Relay
// Extrapolated program for 5 modules
// Creating Discreat Functions
// included servo lib
// included setup functions for all slave ids
// Added Some Delay after writting parameters
// removed feedback and toggle state (no rpm feedback and no blockage detection)
// Edited RPM Write Function
// Added Operational delay
// updated RPM Write function
// commented hobbywing esc control
// changed lpr and max rpm
// removed servo lib
// removed hobbywing function,check blockage and feedback function
// Included provision for all models
// Removed Provision for 4 models

-------------MOTOR MODEL-------------------
Base Motor :- JGB 37 - 520 - 12V - 60R  (60RPM, 100RPM)
Purchase Link :- 

Base Motor :- SPG 30E - 60K (75 RPM )
Purchase Link :- 

----------------RMCS2303--------------------
Purchase Link :- 

-----------------MEGA---------------------
Purchase Link :- 



-----------------------SLAVE ID SELECTION----------------------------
  (4     2    1) - Binary Select

   J1   J2   J3  (from led on board)
   0    0    0  - to ignore hardware selection
   0    0    1  - n1_id 
   0    1    0  - n2_id
   0    1    1  - n3_id
   1    0    0  - n4_id
   1    0    1  - n5_id
   1    1    0  - n6_id
   1    1    1  - n7_id

---------------------MOTOR CONTROLE COMMANDS------------------------- 
  MOTOR CONTROL 
  1)  rmcs.Enable_Digital_Mode(n1_id,1) - Wil enable motor in digital speed control mode. 0-fwd,1-reverse direction. 
  2)  rmcs.Speed(n1_id,speed) - Will Set the Speed(RPM)
  3)  rmcs.Brake_Motor(n1_id,0) - Brake motor. 0-fwd,1-reverse direction. 
  4)  rmcs.Speed_Feedback(n1_id) - RPM Feed Back (incorrect for magnetic encoder)
  5)  rmcs.Disable_Digital_Mode(n1_id,0) - Disable motor in digital speed control mode. 0-fwd,1-reverse direction. 

  PID COMMANDS 
  1)  rmcs.WRITE_PARAMETER(n1_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed) - To write parameters to motor driver
  2)  rmcs.READ_PARAMETER(n1_id) - To read parameters from motor driver

  Digital Speed Control Mode

0101 (257) Enable motor in CW
0100 (256) Disable motor in CW
0104 (260) Brake in CW
0109 (265) Enable motor in CCW
0108 (264) Disable motor in CCW
010C (268) Brake in CCW


-------------------------CONNECTIONS----------------------------
Motor Driver to Encoder Motor  (n1_id) -> ref SLAVE ID SELECTION
   GND     ->   GND 
   +5VDC   ->   VCC
   ENC_B   ->   C2
   ENC_A   ->   C1
   HALL_U  ->   No Connection 
   W       ->   No Connection
   MOTOR - ->   M2 
   MOTOR + ->   M1
   VDD     ->   12v(Power Source) 
   GND     ->   GND(Power Source)

Motor Driver To Mega
   1-GND   ->   GND 
   2-RXD   ->   TX1 (D18)
   3-TXD   ->   RX1 (D19)

   ENC_A   ->   D2  (For Pulse Feedback Alarm)

Interrupt pins in mega - 2,3,21,20,A4,A5

Buzzer To Mega 
emty ->  GND 
 +   ->  D11
*/

#include <FastLED.h>
#define LED_PIN     2
#define NUM_LEDS    20
#include<RMCS2303drive.h>
RMCS2303 rmcs;                    //object for class RMCS2303
CRGB leds[NUM_LEDS];

byte n1_id=1;     //**********  NEED TO CHECK *****************
byte n2_id=2;

int INP_CONTROL_MODE=257;
int PP_gain=32;//32
int PI_gain=16;//16
int VF_gain=32;//32

//int LPR=390;//334 //6.5*60 = 390 //7*60 = 420 //6.8*60 = 408  //**********  NEED TO CHECK *****************
//int LPR =  1008;// 12*168 = 2016 // 6*168 = 1008 **********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (60 RPM))
int LPR = 11;//585;//234;//585; //6*90 = 540 //5.5*90 = 495//6.5*90 = 585 **********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (100 RPM))

int acceleration=5000;//5000
int speed=1000; //8000

int min_pwm = 1051;  //**********  NEED TO CHECK *****************
int max_pwm = 1951;  //**********  NEED TO CHECK *****************
int min_rpm = 0;     //**********  NEED TO CHECK *****************
//int max_rpm = 75;  //**********  NEED TO CHECK *****************(SPG 30E - 60K (75 RPM ))
//int max_rpm = 60;  //**********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (60 RPM))
//int max_rpm = 100; //**********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (100 RPM))
int max_rpm = 6500; //**********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 66RPM (66 RPM)) 
const int inputPwmSignalPin = 9;
const int rpmCutOff = 1250;


int dt2 = 100; //operational delay 
volatile int ToggleState = 0;

const int relayPin = 10;
int rpm_diff = 25;
volatile int blocked = 0;

double FBSLL;
double FBSUL;
double FBSLL2;
double FBSUL2;

double GR = 0.0111; // 66RPM Motor
double CurrentRPM;
double CurrentRPM2;

int BlockageCounter = 0;
int BlockageCounter2 = 0;
int BlockMaxLimit = 5;
int dt = 100; //loop time in ms


void setup()
{
   pinMode(relayPin,OUTPUT);
   digitalWrite(relayPin,LOW);
   FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

   rmcs.Serial_selection(0);       //0-Hardware serial,1-Software serial
  // delay(dt2);
   rmcs.Serial0(9600);     
  // delay(dt2);       // Set Serial0 Baud
   rmcs.begin(&Serial2,9600);    //mega2560:Serial1,Serial2,Serial3 and set baudrate.
  // delay(dt2);
   
   rmcs.WRITE_PARAMETER(n1_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed); //Write parameters to drive
   rmcs.WRITE_PARAMETER(n2_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  // delay(dt2);
  //  rmcs.READ_PARAMETER(n1_id);
  //  delay(dt2);
  rmcs.Disable_Digital_Mode(n1_id,0);
  rmcs.Disable_Digital_Mode(n2_id,0);
    
}

void loop()
{
   int read_input_pwm = pulseIn(inputPwmSignalPin,HIGH); // for flight test
   //int read_input_pwm = 2000; //for test bench
   speed = map(read_input_pwm,min_pwm,max_pwm,min_rpm,max_rpm);
   speed = constrain(speed, min_rpm, max_rpm); // Sets Minimum and maximum limits
   Serial.print("Calculated RPM : ");
   Serial.println(speed);
   
 
 if(speed >= rpmCutOff)
 {
   BlockageCounter = 0;
   BlockageCounter2 = 0;
  //-------------------------- Seeder1 -----------------------
  for(int i = 1;i<=BlockMaxLimit;i++)
  {
      rmcs.Enable_Digital_Mode(n1_id, 0);
      rmcs.Speed(n1_id,speed);   
      int n1_feed_back = rmcs.Speed_Feedback(n1_id);

      if(n1_feed_back < speed)
      {
        FBSLL = n1_feed_back;
      }
      else if (n1_feed_back >= speed)
      {
        FBSUL = n1_feed_back;
      }
      if (FBSUL && FBSLL) 
      {
        CurrentRPM = ((FBSLL+FBSUL)/2.0)*GR;
         Serial.print("1cond1 = ");
         Serial.println(CurrentRPM);
      }
      else if (FBSLL == 0 && n1_feed_back == 0)
      {
        CurrentRPM = 0 ;
        Serial.print("1cond2 = ");
         Serial.println(CurrentRPM);
      }
      else if (FBSLL)
      {
        CurrentRPM = FBSLL*GR;
        Serial.print("1cond3 = ");
         Serial.println(CurrentRPM);
      }
      else if (FBSUL)
      { 
        CurrentRPM = FBSUL*GR;
        Serial.print("1cond4 = ");
         Serial.println(CurrentRPM);
      }
    //--------------------------------------------------------------------
    // ---------------------- Seeder 2 -----------------------------------
    rmcs.Enable_Digital_Mode(n2_id, 0);
      rmcs.Speed(n2_id,speed);   
      int n2_feed_back = rmcs.Speed_Feedback(n2_id);

      if(n2_feed_back < speed)
      {
        FBSLL2 = n2_feed_back;
      }
      else if (n2_feed_back >= speed)
      {
        FBSUL2 = n2_feed_back;
      }
      if (FBSUL2 && FBSLL2) 
      {
        CurrentRPM2 = ((FBSLL2+FBSUL2)/2.0)*GR;
         Serial.print("2cond1 = ");
         Serial.println(CurrentRPM2);
      }
      else if (FBSLL2 == 0 && n2_feed_back == 0)
      {
        CurrentRPM2 = 0 ;
        Serial.print("2cond2 = ");
         Serial.println(CurrentRPM2);
      }
      else if (FBSLL2)
      {
        CurrentRPM2 = FBSLL2*GR;
        Serial.print("2cond3 = ");
         Serial.println(CurrentRPM2);
      }
      else if (FBSUL2)
      { 
        CurrentRPM2 = FBSUL2*GR;
        Serial.print("2cond4 = ");
         Serial.println(CurrentRPM2);
      }
    //--------------------------------------------------------------

       delay(dt);
    //-------------Seeder 1--------------------------------------
      if (CurrentRPM == 0)
      {
        BlockageCounter++;
      }
      if(BlockageCounter >= BlockMaxLimit)
      {
        Serial.println("1Seed Blocked");
        digitalWrite(relayPin,HIGH);
      }  
    //---------------------------------------------------------------
    //-------------------Seeder 2------------------------------------
     if (CurrentRPM2 == 0)
      {
        BlockageCounter2++;
      }
      if(BlockageCounter2 >= BlockMaxLimit)
      {
        Serial.println("2Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //----------------------------------------------------------------
  }
 }
 else 
 {
    rmcs.Disable_Digital_Mode(n1_id,0);
    rmcs.Disable_Digital_Mode(n2_id,0);

 }

}

