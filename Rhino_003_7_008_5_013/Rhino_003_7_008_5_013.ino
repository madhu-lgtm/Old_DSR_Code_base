// Single Module Test :- 28-01-2025
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

int INP_CONTROL_MODE=257;
int PP_gain=32;//32
int PI_gain=16;//16
int VF_gain=32;//32

//int LPR=390;//334 //6.5*60 = 390 //7*60 = 420 //6.8*60 = 408  //**********  NEED TO CHECK *****************
//int LPR =  1008;// 12*168 = 2016 // 6*168 = 1008 **********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (60 RPM))
int LPR = 585;//234;//585; //6*90 = 540 //5.5*90 = 495//6.5*90 = 585 **********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (100 RPM))

int acceleration=500;//5000
int speed=10; //8000

int min_pwm = 1051;  //**********  NEED TO CHECK *****************
int max_pwm = 1951;  //**********  NEED TO CHECK *****************
int min_rpm = 0;     //**********  NEED TO CHECK *****************
//int max_rpm = 75;  //**********  NEED TO CHECK *****************(SPG 30E - 60K (75 RPM ))
//int max_rpm = 60;  //**********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (60 RPM))
int max_rpm = 100;//160;//100;   //**********  NEED TO CHECK *****************(JGB 37 - 520 - 12V - 60R (100 RPM))
const int inputPwmSignalPin = 9;
const int rpmCutOff = 25;

int dt = 250; //loop time in ms
int dt2 = 100; //operational delay 
volatile int ToggleState = 0;

const int relayPin = 10;
int rpm_diff = 25;
volatile int blocked = 0;

void setup()
{

   pinMode(relayPin,OUTPUT);
   digitalWrite(relayPin,LOW);
   FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

   rmcs.Serial_selection(0);       //0-Hardware serial,1-Software serial
  // delay(dt2);
   rmcs.Serial0(9600);     
  // delay(dt2);       // Set Serial0 Baud
   rmcs.begin(&Serial3,9600);    //mega2560:Serial1,Serial2,Serial3 and set baudrate.
  // delay(dt2);
   
   rmcs.WRITE_PARAMETER(n1_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed); //Write parameters to drive
  // delay(dt2);
  //  rmcs.READ_PARAMETER(n1_id);
  //  delay(dt2);
  rmcs.Disable_Digital_Mode(n1_id,0);
    
}

void loop()
{
  int read_input_pwm = pulseIn(inputPwmSignalPin,HIGH); // for flight test
  speed = map(read_input_pwm,min_pwm,max_pwm,min_rpm,max_rpm);
  speed = constrain(speed, min_rpm, max_rpm); // Sets Minimum and maximum limits
  // Serial.print("Calculated RPM : ");
  // Serial.println(speed);

     if(speed >= rpmCutOff)
     {
      if(blocked == 0)
      {
      if(ToggleState == 0)
      {
        ToggleState = 1;
        rmcs.Enable_Digital_Mode(n1_id, 0);
      }
      
      rmcs.Speed(n1_id,speed);   
      Serial.print("set speed = ");
      Serial.println(speed);

        
      int n1_feed_back = rmcs.Speed_Feedback(n1_id);
      Serial.println(n1_feed_back);
     
      if(n1_feed_back == 0 )
      {
        rmcs.Disable_Digital_Mode(n1_id,0);
        blocked = 1;
        //Serial.println("Clear Blockage");
        digitalWrite(relayPin,HIGH);
      } 

     }
    //  else  { 
    //     digitalWrite(relayPin,LOW);
    //  }
     }
     else
     {
       ToggleState = 0;
       rmcs.Disable_Digital_Mode(n1_id,0);
     }     
    //delay(dt);
}

