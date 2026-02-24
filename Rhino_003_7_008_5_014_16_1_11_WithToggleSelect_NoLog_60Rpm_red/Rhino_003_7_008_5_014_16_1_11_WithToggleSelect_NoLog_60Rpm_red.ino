// Updated Code for 3 Modules (10-09-2025)
// Updated Code for 2 Modules and cleaned up the code (10-09-2025)
// updated code for 8 modules (04-08-2025)
// updated code for red indication on motor drivers ; Changed max and and min rpms (17-06-2025) RE-CALIBRATION OF SEED IS REQUIRED
// uncomented rpm print statements (13-06-2025)
// updated for 5 Modules (27-05-2025)
// updated for 3 modules
// Updated code to turn off buzzer when switched off the toggle switch on tx
// Changed Gear Ratio for 60 RPM Motor
// Changed inputPwmSignalPin_2 = fron 5 to 4
// Commented rmcs.ESTOP(n5_id)
// updated INP CONTROL Mode from 257 to 260 (18-05-2025)
// included spi and sd lib
// only nozzle 5 active 
// Dividing the functions 
// Removed pulseIn Function for PWM_2
// Removed Toggle1 Physical Switch Logic
// Incorporated Toggle 1 physical switch
// Updated to 5 Modules with light indication
// Updated to 2 Modules 
// Single Module Test :- 28-01-2025
// Implemented RPM Filter for single module :- 05-02-2025
// extended filter 
// blockage function

/*
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
  6)  rmcs.ESTOP(n1_id) - This function will stop motor and cut off power from drive to motor

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
#define NUM_LEDS    12 //8_Seeders * 4_leds = 32 --> 2*4 = 8 --> 3*4 = 12
#include<RMCS2303drive.h>
RMCS2303 rmcs;  //object for class RMCS2303
CRGB leds[NUM_LEDS];


byte n1_id=1;     //**********  NEED TO CHECK *****************
byte n2_id=2;
byte n3_id=3;
byte n4_id=4;
byte n5_id=5; 
byte n6_id=6;
byte n7_id=7;
byte n8_id=8;

int INP_CONTROL_MODE=260;// 257-> 260
int PP_gain=32;//32
int PI_gain=16;//16
int VF_gain=32;//32

int LPR = 11;
int acceleration=5000;//5000
int speed=1000; //8000

int min_pwm = 1051;  //**********  NEED TO CHECK *****************
int max_pwm = 1951;  //**********  NEED TO CHECK *****************
int min_rpm = 500;     //0->500
int max_rpm = 5000; //(JGB 37 - 520 - 12V - 66RPM (66 RPM)) 6500-> 5000 

const int inputPwmSignalPin = 9;
const int inputPwmSignalPin_2 = 4;//5 or 4 
const int rpmCutOff = 1250;

int dt2 = 100; //operational delay 
volatile int ToggleState = 0;

const int relayPin = 10;
int rpm_diff = 25;
volatile int blocked = 0;

int dwell = 1000; // 500 to 1000 //500 working good on bench for one seerder
int gain_rpm = 1000;// Need to tune (500 to 1000) //1000 is good

double FBSLL;
double FBSUL;
double FBSLL2;
double FBSUL2;
double FBSLL3;
double FBSUL3;
double FBSLL4;
double FBSUL4;
double FBSLL5;
double FBSUL5;
double FBSLL6;
double FBSUL6;
double FBSLL7;
double FBSUL7;
double FBSLL8;
double FBSUL8;


double GR = 0.0103; // 66RPM = 0.0111 ;  60RPM = 0.0103 
double CurrentRPM;
double CurrentRPM2;
double CurrentRPM3;
double CurrentRPM4;
double CurrentRPM5;
double CurrentRPM6;
double CurrentRPM7;
double CurrentRPM8;

int BlockageCounter = 0;
int BlockageCounter2 = 0;
int BlockageCounter3 = 0;
int BlockageCounter4 = 0;
int BlockageCounter5 = 0;
int BlockageCounter6 = 0;
int BlockageCounter7 = 0;
int BlockageCounter8 = 0;


int BlockMaxLimit = 2;
int dt = 200; //loop time in ms

int config1;
int config2;
int config3;
int config4;
int config5;
int config6;
int config7;
int config8;
int config9;
int config10;
int config11;
int config12;
int config13;
int config14;
int config15;
int config16;
int config17;

int config1_pwm = 1101;
int config2_pwm = 1151; //All 5 active
int config3_pwm = 1201;
int config4_pwm = 1251;
int config5_pwm = 1301;
int config6_pwm = 1351;
int config7_pwm = 1401;
int config8_pwm = 1451;
int config9_pwm = 1501;
int config10_pwm = 1551;
int config11_pwm = 1601;
int config12_pwm = 1651;
int config13_pwm = 1701;
int config14_pwm = 1751;
int config15_pwm = 1801;
int config16_pwm = 1851;
int config17_pwm = 1901;

int config1_pwm_min = 1076;
int config2_pwm_min = 1126;
int config3_pwm_min = 1176;
int config4_pwm_min = 1226;
int config5_pwm_min = 1276;
int config6_pwm_min = 1323;
int config7_pwm_min = 1376;
int config8_pwm_min = 1426;
int config9_pwm_min = 1476;
int config10_pwm_min = 1526;
int config11_pwm_min = 1576;
int config12_pwm_min = 1626;
int config13_pwm_min = 1676;
int config14_pwm_min = 1726;
int config15_pwm_min = 1776;
int config16_pwm_min = 1826;
int config17_pwm_min = 1876;

int config1_pwm_max = 1126;
int config2_pwm_max = 1176;
int config3_pwm_max = 1226;
int config4_pwm_max = 1276;
int config5_pwm_max = 1326;
int config6_pwm_max = 1376;
int config7_pwm_max = 1426;
int config8_pwm_max = 1476;
int config9_pwm_max = 1526;
int config10_pwm_max = 1576;
int config11_pwm_max = 1626;
int config12_pwm_max = 1676;
int config13_pwm_max = 1726;
int config14_pwm_max = 1776;
int config15_pwm_max = 1826;
int config16_pwm_max = 1876;
int config17_pwm_max = 1926;

int read_input_pwm;
int read_input_pwm_2;

void setup()
{
   pinMode(relayPin,OUTPUT);
   digitalWrite(relayPin,LOW);
   FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

   rmcs.Serial_selection(0);       //0-Hardware serial,1-Software serial
   rmcs.Serial0(9600); // Set Serial0 Baud  9600   
   rmcs.begin(&Serial3,9600);    //mega2560:Serial1,Serial2,Serial3 and set baudrate.
  
   rmcs.WRITE_PARAMETER(n1_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed); //Write parameters to drive
   rmcs.WRITE_PARAMETER(n2_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
   rmcs.WRITE_PARAMETER(n3_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  //  rmcs.WRITE_PARAMETER(n4_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  //  rmcs.WRITE_PARAMETER(n5_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  //  rmcs.WRITE_PARAMETER(n6_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  //  rmcs.WRITE_PARAMETER(n7_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  //  rmcs.WRITE_PARAMETER(n8_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);

   rmcs.Disable_Digital_Mode(n1_id,0);
   rmcs.Disable_Digital_Mode(n2_id,0);
   rmcs.Disable_Digital_Mode(n3_id,0);
  //  rmcs.Disable_Digital_Mode(n4_id,0);
  //  rmcs.Disable_Digital_Mode(n5_id,0);
  //  rmcs.Disable_Digital_Mode(n6_id,0);
  //  rmcs.Disable_Digital_Mode(n7_id,0);
  //  rmcs.Disable_Digital_Mode(n8_id,0);
}

void speed_select()
{
   read_input_pwm = pulseIn(inputPwmSignalPin,HIGH); // for flight test
   
   //read_input_pwm = 1500; //for test bench
   speed = map(read_input_pwm,min_pwm,max_pwm,min_rpm,max_rpm);
   speed = constrain(speed, min_rpm, max_rpm); // Sets Minimum and maximum limits
  //  Serial.print("RPM_SET : ");
  //  Serial.println(speed);    
}

void configuration_select()
{
  //read_input_pwm_2 = pulseIn(inputPwmSignalPin_2,HIGH);
   read_input_pwm_2 = 1151;
   //Serial.println(read_input_pwm_2);

   if(read_input_pwm_2 <= config1_pwm_max)
   {
    config1 = 1;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }
   else if (read_input_pwm_2 >= config2_pwm_min && read_input_pwm_2 <= config2_pwm_max)
   {
    config1 = 0;
    config2 = 1; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }
   else if (read_input_pwm_2 >= config3_pwm_min && read_input_pwm_2 <= config3_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 1;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config4_pwm_min && read_input_pwm_2 <= config4_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 1;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config5_pwm_min && read_input_pwm_2 <= config5_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 1;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }
   else if (read_input_pwm_2 >= config6_pwm_min && read_input_pwm_2 <= config6_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 1;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config7_pwm_min && read_input_pwm_2 <= config7_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 1;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config8_pwm_min && read_input_pwm_2 <= config8_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 1;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config9_pwm_min && read_input_pwm_2 <= config9_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 1;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config10_pwm_min && read_input_pwm_2 <= config10_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 1;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config11_pwm_min && read_input_pwm_2 <= config11_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 1;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config12_pwm_min && read_input_pwm_2 <= config12_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 1;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config13_pwm_min && read_input_pwm_2 <= config13_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 1;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config14_pwm_min && read_input_pwm_2 <= config14_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 1;
    config15 = 0;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config15_pwm_min && read_input_pwm_2 <= config15_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 1;
    config16 = 0;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config16_pwm_min && read_input_pwm_2 <= config16_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 1;
    config17 = 0;
   }

   else if (read_input_pwm_2 >= config17_pwm_min && read_input_pwm_2 <= config17_pwm_max)
   {
    config1 = 0;
    config2 = 0; 
    config3 = 0;
    config4 = 0;
    config5 = 0;
    config6 = 0;
    config7 = 0;
    config8 = 0;
    config9 = 0;
    config10 = 0;
    config11 = 0;
    config12 = 0;
    config13 = 0;
    config14 = 0;
    config15 = 0;
    config16 = 0;
    config17 = 1;
   }
   //------------------------------------------------------
}

void check_seeder1()
{
  //-------------------------- Seeder1 -----------------------
  // if (config2 == 1 || config3 == 1 || config4 == 1 || config5 == 1 || config6 == 1 || config11 == 1 || config13 == 1|| config15 == 1 || config17 == 1)
  // {
        rmcs.Enable_Digital_Mode(n1_id, 0);
        leds[0] = CRGB(0,255,0); // Green 
        leds[1] = CRGB(0,255,0);
        leds[2] = CRGB(0,255,0);
        leds[3] = CRGB(0,255,0);

      rmcs.Speed(n1_id,speed);   
      int n1_feed_back = rmcs.Speed_Feedback(n1_id);

      //----------------- red indication ---------------
        if (n1_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n1_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n1_id,0);
        }
     //----------------------------------------

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
        //  Serial.print("M1_A = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL == 0 && n1_feed_back == 0)
      {
        CurrentRPM = 0 ;
        //  Serial.print("M1_Z = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL)
      {
        CurrentRPM = FBSLL*GR;
        //  Serial.print("M1_L = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSUL)
      { 
        CurrentRPM = FBSUL*GR;
        //  Serial.print("M1_U = ");
        //  Serial.println(CurrentRPM);
      }

      //-------------Seeder 1--------------------------------------
      if (CurrentRPM == 0)
      {
        BlockageCounter++;
      }
      if(BlockageCounter >= BlockMaxLimit)
      {
        leds[0] = CRGB(255,0,0); //RED
        leds[1] = CRGB(255,0,0);
        leds[2] = CRGB(255,0,0);
        leds[3] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n1_id,0); //need to check
        // Serial.println("1Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //---------------------------------------------------------------
  // }
  // else 
  // {
  //       leds[0] = CRGB(255,0,255); //purple
  //       leds[1] = CRGB(255,0,255);
  //       leds[2] = CRGB(255,0,255);
  //       leds[3] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n1_id,0);      
  //   }
    //--------------------------------------------------------------------
}

void check_seeder2()
{
     // ---------------------- Seeder 2 -----------------------------------
  //   if (config2 == 1 || config3 == 1 || config4 == 1 || config5 == 1 || config7 == 1 || config12 == 1 || config16 == 1)
  // {
        rmcs.Enable_Digital_Mode(n2_id, 0);
        leds[4] = CRGB(0,255,0); // Green 
        leds[5] = CRGB(0,255,0);
        leds[6] = CRGB(0,255,0);
        leds[7] = CRGB(0,255,0);
      rmcs.Speed(n2_id,speed);   
      int n2_feed_back = rmcs.Speed_Feedback(n2_id);

      //----------------- red indication ---------------
 
        if (n2_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n2_id,0);
          delay(dwell*2);
          rmcs.Enable_Digital_Mode(n2_id,0);
          
        }
     //----------------------------------------

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
        //  Serial.print("M2_A = ");
        //  Serial.println(CurrentRPM2);
      }
      else if (FBSLL2 == 0 && n2_feed_back == 0)
      {
        CurrentRPM2 = 0 ;
        //  Serial.print("M2_Z = ");
        //  Serial.println(CurrentRPM2);
      }
      else if (FBSLL2)
      {
        CurrentRPM2 = FBSLL2*GR;
        //  Serial.print("M2_L = ");
        //  Serial.println(CurrentRPM2);
      }
      else if (FBSUL2)
      { 
        CurrentRPM2 = FBSUL2*GR;
        //  Serial.print("M2_U = ");
        //  Serial.println(CurrentRPM2);
      }
      //-------------------Seeder 2------------------------------------
     if (CurrentRPM2 == 0)
      {
        BlockageCounter2++;
      }
      if(BlockageCounter2 >= BlockMaxLimit)
      {
        leds[4] = CRGB(255,0,0); //RED
        leds[5] = CRGB(255,0,0);
        leds[6] = CRGB(255,0,0);
        leds[7] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n2_id,0);

        // Serial.println("2Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //--------------------------------------------------------------
  // }
  //   else 
  // {
  //       leds[4] = CRGB(255,0,255); //purple
  //       leds[5] = CRGB(255,0,255);
  //       leds[6] = CRGB(255,0,255);
  //       leds[7] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n2_id,0);
  // }
//--------------------------------------------------------------

}

void check_seeder3()
{
  // ---------------------- Seeder 3 -----------------------------------
// if (config2 == 1 || config3 == 1 || config4 == 1 || config7 == 1 || config8 == 1 || config11 == 1|| config13 == 1 || config14 == 1)
//   {
      rmcs.Enable_Digital_Mode(n3_id, 0);
        leds[8] = CRGB(0,255,0); // Green 
        leds[9] = CRGB(0,255,0);
        leds[10] = CRGB(0,255,0);
        leds[11] = CRGB(0,255,0);
      rmcs.Speed(n3_id,speed);   
      int n3_feed_back = rmcs.Speed_Feedback(n3_id);
      // Serial.print("n3_feed_back = "); // Testing 
      // Serial.println(n3_feed_back); // Testing 

      //----------------- red indication ---------------
        if (n3_feed_back > (speed+gain_rpm))
        {
        //  Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n3_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n3_id,0);
          
        }
     //----------------------------------------

      if(n3_feed_back < speed)
      {
        FBSLL3 = n3_feed_back;
      }
      else if (n3_feed_back >= speed)
      {
        FBSUL3 = n3_feed_back;
      }
      if (FBSUL3 && FBSLL3) 
      {
        CurrentRPM3 = ((FBSLL3+FBSUL3)/2.0)*GR;
        //  Serial.print("M3_A = ");
        //  Serial.println(CurrentRPM3);
      }
      else if (FBSLL3 == 0 && n3_feed_back == 0)
      {
        CurrentRPM3 = 0 ;
        //  Serial.print("M3_Z = ");
        //  Serial.println(CurrentRPM3);
      }
      else if (FBSLL3)
      {
        CurrentRPM3 = FBSLL3*GR;
        //  Serial.print("M3_L = ");
        //  Serial.println(CurrentRPM3);
      }
      else if (FBSUL3)
      { 
        CurrentRPM3 = FBSUL3*GR;
        //  Serial.print("M3_U = ");
        //  Serial.println(CurrentRPM3);
      }

      //-------------------Seeder 3------------------------------------
     if (CurrentRPM3 == 0)
      {
        BlockageCounter3++;
      }
      if(BlockageCounter3 >= BlockMaxLimit)
      {
        leds[8] = CRGB(255,0,0); //RED
        leds[9] = CRGB(255,0,0);
        leds[10] = CRGB(255,0,0);
        leds[11] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n3_id,0);

        // Serial.println("3Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //----------------------------------------------------------------
  //  }
  //   else 
  // {
  //       leds[8] = CRGB(255,0,255); //purple
  //       leds[9] = CRGB(255,0,255);
  //       leds[10] = CRGB(255,0,255);
  //       leds[11] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n3_id,0);
  // }

    //--------------------------------------------------------------
}

void check_seeder4()
{
  // ---------------------- Seeder 4 -----------------------------------
// if (config2 == 1 || config3 == 1 || config7 == 1 || config8 == 1 || config9 == 1 || config12 == 1 || config15 == 1)
//   {
      rmcs.Enable_Digital_Mode(n4_id, 0);
        leds[12] = CRGB(0,255,0); // Green 
        leds[13] = CRGB(0,255,0);
        leds[14] = CRGB(0,255,0);
        leds[15] = CRGB(0,255,0);
      rmcs.Speed(n4_id,speed);   
      int n4_feed_back = rmcs.Speed_Feedback(n4_id);

      //----------------- red indication ---------------
         if (n4_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n4_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n4_id,0);
          
        }
     //----------------------------------------

      if(n4_feed_back < speed)
      {
        FBSLL4 = n4_feed_back;
      }
      else if (n4_feed_back >= speed)
      {
        FBSUL4 = n4_feed_back;
      }
      if (FBSUL4 && FBSLL4) 
      {
        CurrentRPM4 = ((FBSLL4+FBSUL4)/2.0)*GR;
        //  Serial.print("M4_A = ");
        //  Serial.println(CurrentRPM4);
      }
      else if (FBSLL4 == 0 && n4_feed_back == 0)
      {
        CurrentRPM4 = 0 ;
        // Serial.print("M4_Z = ");
        //  Serial.println(CurrentRPM4);
      }
      else if (FBSLL4)
      {
        CurrentRPM4 = FBSLL4*GR;
        // Serial.print("M4_L = ");
        //  Serial.println(CurrentRPM4);
      }
      else if (FBSUL4)
      { 
        CurrentRPM4 = FBSUL4*GR;
        // Serial.print("M4_U = ");
        //  Serial.println(CurrentRPM4);
      }
      //-------------------Seeder 4------------------------------------
     if (CurrentRPM4 == 0)
      {
        BlockageCounter4++;
      }
      if(BlockageCounter4 >= BlockMaxLimit)
      {
        leds[12] = CRGB(255,0,0); //RED
        leds[13] = CRGB(255,0,0);
        leds[14] = CRGB(255,0,0);
        leds[15] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n4_id,0);

        // Serial.println("4Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //----------------------------------------------------------------

  // }
  // else 
  // {
  //       leds[12] = CRGB(255,0,255); //purple
  //       leds[13] = CRGB(255,0,255);
  //       leds[14] = CRGB(255,0,255);
  //       leds[15] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n4_id,0);
  // }
    //--------------------------------------------------------------
}

void check_seeder5()
{
  // ------------------ Seeder 5 -----------------------------------
// if (config2 == 1 || config7 == 1 || config8 == 1 || config9 == 1 || config10 == 1 || config11 == 1 || config14 == 1|| config16 == 1 || config17 == 1)
//   {
      rmcs.Enable_Digital_Mode(n5_id, 0);
        leds[16] = CRGB(0,255,0); // Green 
        leds[17] = CRGB(0,255,0);
        leds[18] = CRGB(0,255,0);
        leds[19] = CRGB(0,255,0);
      rmcs.Speed(n5_id,speed);   
     int n5_feed_back = rmcs.Speed_Feedback(n5_id);

      //----------------- red indication ---------------
        if (n5_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n5_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n5_id,0);
        
        }
     //----------------------------------------

      if(n5_feed_back < speed)
      {
        FBSLL5 = n5_feed_back;
      }
      else if (n5_feed_back >= speed)
      {
        FBSUL5 = n5_feed_back;
      }
      if (FBSUL5 && FBSLL5) 
      {
         CurrentRPM5 = ((FBSLL5+FBSUL5)/2.0)*GR;
        //  Serial.print("M5_A = ");
        //  Serial.println(CurrentRPM5);
      }
      else if (FBSLL5 == 0 && n5_feed_back == 0)
      {
        CurrentRPM5 = 0 ;
        // Serial.print("M5_Z = ");
        //  Serial.println(CurrentRPM5);
      }
      else if (FBSLL5)
      {
        CurrentRPM5 = FBSLL5*GR;
        // Serial.print("M5_L = ");
        //  Serial.println(CurrentRPM5);
      }
      else if (FBSUL5)
      { 
        CurrentRPM5 = FBSUL5*GR;
        // Serial.print("M5_U = ");
        //  Serial.println(CurrentRPM5);
      }
    //--------------------------------------------------------------
    //-------------------Seeder 5------------------------------------
     if (CurrentRPM5 == 0)
      {
        BlockageCounter5++;
      }
      if(BlockageCounter5 >= BlockMaxLimit)
      {
        leds[16] = CRGB(255,0,0); //RED
        leds[17] = CRGB(255,0,0);
        leds[18] = CRGB(255,0,0);
        leds[19] = CRGB(255,0,0);

       rmcs.Disable_Digital_Mode(n5_id,0);
        // Serial.println("5Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    
//   }
//  else 
//   {
//         leds[16] = CRGB(255,0,255); //purple
//         leds[17] = CRGB(255,0,255);
//         leds[18] = CRGB(255,0,255);
//         leds[19] = CRGB(255,0,255);

//         rmcs.Disable_Digital_Mode(n5_id,0);
//   }
//----------------------------------------------------------------
}

void check_seeder6()
{
  //-------------------------- Seeder6 -----------------------
  // if (config2 == 1 || config3 == 1 || config4 == 1 || config5 == 1 || config6 == 1 || config11 == 1 || config13 == 1|| config15 == 1 || config17 == 1)
  // {
        rmcs.Enable_Digital_Mode(n6_id, 0);  
        leds[20] = CRGB(0,255,0); // Green 
        leds[21] = CRGB(0,255,0);
        leds[22] = CRGB(0,255,0);
        leds[23] = CRGB(0,255,0);

      rmcs.Speed(n6_id,speed);   
      int n6_feed_back = rmcs.Speed_Feedback(n6_id);

      //----------------- red indication ---------------
        if (n6_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n6_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n6_id,0);
        }
     //----------------------------------------

      if(n6_feed_back < speed)
      {
        FBSLL6 = n6_feed_back;
      }
      else if (n6_feed_back >= speed)
      {
        FBSUL6 = n6_feed_back;
      }
      if (FBSUL6 && FBSLL6) 
      {
        CurrentRPM6 = ((FBSLL6+FBSUL6)/2.0)*GR;
        //  Serial.print("M1_A = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL6 == 0 && n6_feed_back == 0)
      {
        CurrentRPM6 = 0 ;
        //  Serial.print("M1_Z = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL6)
      {
        CurrentRPM6 = FBSLL6*GR;
        //  Serial.print("M1_L = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSUL6)
      { 
        CurrentRPM6 = FBSUL6*GR;
        //  Serial.print("M1_U = ");
        //  Serial.println(CurrentRPM);
      }

      //-------------Seeder 6--------------------------------------
      if (CurrentRPM6 == 0)
      {
        BlockageCounter6++;
      }
      if(BlockageCounter6 >= BlockMaxLimit)
      {
        leds[20] = CRGB(255,0,0); //RED
        leds[21] = CRGB(255,0,0);
        leds[22] = CRGB(255,0,0);
        leds[23] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n6_id,0); //need to check
        // Serial.println("1Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //---------------------------------------------------------------
  
  // }
  // else 
  // {
  //       leds[20] = CRGB(255,0,255); //purple
  //       leds[21] = CRGB(255,0,255);
  //       leds[22] = CRGB(255,0,255);
  //       leds[23] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n6_id,0);      
  //   }
    //--------------------------------------------------------------------
}

void check_seeder7()
{
  //-------------------------- Seeder7 -----------------------
  // if (config2 == 1 || config3 == 1 || config4 == 1 || config5 == 1 || config6 == 1 || config11 == 1 || config13 == 1|| config15 == 1 || config17 == 1)
  // {
        
        rmcs.Enable_Digital_Mode(n7_id, 0);
        
        
        leds[24] = CRGB(0,255,0); // Green 
        leds[25] = CRGB(0,255,0);
        leds[26] = CRGB(0,255,0);
        leds[27] = CRGB(0,255,0);

      rmcs.Speed(n7_id,speed);   
      int n7_feed_back = rmcs.Speed_Feedback(n7_id);

      //----------------- red indication ---------------
        if (n7_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n7_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n7_id,0);
        }

     //----------------------------------------

      if(n7_feed_back < speed)
      {
        FBSLL7 = n7_feed_back;
      }
      else if (n7_feed_back >= speed)
      {
        FBSUL7 = n7_feed_back;
      }
      if (FBSUL7 && FBSLL7) 
      {
        CurrentRPM7 = ((FBSLL7+FBSUL7)/2.0)*GR;
        //  Serial.print("M1_A = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL7 == 0 && n7_feed_back == 0)
      {
        CurrentRPM7 = 0 ;
        //  Serial.print("M1_Z = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL7)
      {
        CurrentRPM7 = FBSLL7*GR;
        //  Serial.print("M1_L = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSUL7)
      { 
        CurrentRPM7 = FBSUL7*GR;
        //  Serial.print("M1_U = ");
        //  Serial.println(CurrentRPM);
      }

      //-------------Seeder 7--------------------------------------
      if (CurrentRPM7 == 0)
      {
        BlockageCounter7++;
      }
      if(BlockageCounter7 >= BlockMaxLimit)
      {
        leds[24] = CRGB(255,0,0); //RED
        leds[25] = CRGB(255,0,0);
        leds[26] = CRGB(255,0,0);
        leds[27] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n7_id,0); //need to check
        // Serial.println("1Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //---------------------------------------------------------------

  // }
  // else 
  // {
  //       leds[24] = CRGB(255,0,255); //purple
  //       leds[25] = CRGB(255,0,255);
  //       leds[26] = CRGB(255,0,255);
  //       leds[27] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n7_id,0);      
  //   }
    //--------------------------------------------------------------------
}

void check_seeder8()
{
  //-------------------------- Seeder8 -----------------------
  // if (config2 == 1 || config3 == 1 || config4 == 1 || config5 == 1 || config6 == 1 || config11 == 1 || config13 == 1|| config15 == 1 || config17 == 1)
  // {
        rmcs.Enable_Digital_Mode(n8_id, 0);  
        leds[28] = CRGB(0,255,0); // Green 
        leds[29] = CRGB(0,255,0);
        leds[30] = CRGB(0,255,0);
        leds[31] = CRGB(0,255,0);

      rmcs.Speed(n8_id,speed);   
      int n8_feed_back = rmcs.Speed_Feedback(n8_id);

      //----------------- red indication ---------------

        if (n8_feed_back > (speed+gain_rpm))
        {
         // Serial.println("Re enable two************");
          rmcs.Disable_Digital_Mode(n8_id,0);
          delay(dwell);
          rmcs.Enable_Digital_Mode(n8_id,0);
        }

     //----------------------------------------

      if(n8_feed_back < speed)
      {
        FBSLL8 = n8_feed_back;
      }
      else if (n8_feed_back >= speed)
      {
        FBSUL8 = n8_feed_back;
      }
      if (FBSUL8 && FBSLL8) 
      {
        CurrentRPM8 = ((FBSLL8+FBSUL8)/2.0)*GR;
        //  Serial.print("M1_A = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL8 == 0 && n8_feed_back == 0)
      {
        CurrentRPM8 = 0 ;
        //  Serial.print("M1_Z = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSLL8)
      {
        CurrentRPM8 = FBSLL8*GR;
        //  Serial.print("M1_L = ");
        //  Serial.println(CurrentRPM);
      }
      else if (FBSUL8)
      { 
        CurrentRPM8 = FBSUL8*GR;
        //  Serial.print("M1_U = ");
        //  Serial.println(CurrentRPM);
      }

      //-------------Seeder 8--------------------------------------
      if (CurrentRPM8 == 0)
      {
        BlockageCounter8++;
      }
      if(BlockageCounter8 >= BlockMaxLimit)
      {
        leds[28] = CRGB(255,0,0); //RED
        leds[29] = CRGB(255,0,0);
        leds[30] = CRGB(255,0,0);
        leds[31] = CRGB(255,0,0);

        rmcs.Disable_Digital_Mode(n8_id,0); //need to check
        // Serial.println("1Seed Blocked");
        digitalWrite(relayPin,HIGH);
      } 
    //---------------------------------------------------------------
  
  // }
  // else 
  // {
  //       leds[28] = CRGB(255,0,255); //purple
  //       leds[29] = CRGB(255,0,255);
  //       leds[30] = CRGB(255,0,255);
  //       leds[31] = CRGB(255,0,255);

  //       rmcs.Disable_Digital_Mode(n8_id,0);      
  //   }
    //--------------------------------------------------------------------
}

void check_relay()
{
  //-------------- Relay Low ------------------
    // if ((BlockageCounter < BlockMaxLimit) && (BlockageCounter2 < BlockMaxLimit) && (BlockageCounter3 < BlockMaxLimit) && (BlockageCounter4 < BlockMaxLimit) && (BlockageCounter5 < BlockMaxLimit ) && (BlockageCounter6 < BlockMaxLimit ) && (BlockageCounter7 < BlockMaxLimit ) && (BlockageCounter8 < BlockMaxLimit ))
    // {
    //   digitalWrite(relayPin,LOW);
    // }

  if ((BlockageCounter < BlockMaxLimit) && (BlockageCounter2 < BlockMaxLimit) && (BlockageCounter3 < BlockMaxLimit))
    {
      digitalWrite(relayPin,LOW);
    }
}

void check_rpm_cutoff()
{
   if(speed >= rpmCutOff)
 {
   BlockageCounter = 0;
   BlockageCounter2 = 0;
   BlockageCounter3 = 0;
  //  BlockageCounter4 = 0;
  //  BlockageCounter5 = 0;
  //  BlockageCounter6 = 0;
  //  BlockageCounter7 = 0;
  //  BlockageCounter8 = 0;

   for(int i = 1;i<=BlockMaxLimit;i++)
  { 
   check_seeder1();
   check_seeder2();
   check_seeder3();
  //  check_seeder4();
  //  check_seeder5();
  //  check_seeder6();
  //  check_seeder7();
  //  check_seeder8();
   delay(dt); 
  }
   check_relay();
 }
 else 
 {
    rmcs.Disable_Digital_Mode(n1_id,0);
    rmcs.Disable_Digital_Mode(n2_id,0);
    rmcs.Disable_Digital_Mode(n3_id,0);
    // rmcs.Disable_Digital_Mode(n4_id,0);
    // rmcs.Disable_Digital_Mode(n5_id,0);
    // rmcs.Disable_Digital_Mode(n6_id,0);
    // rmcs.Disable_Digital_Mode(n7_id,0);
    // rmcs.Disable_Digital_Mode(n8_id,0);

    // --------------- All Turned OFF ------------

        leds[0] = CRGB(0,150,255); //blue
        leds[1] = CRGB(0,150,255);
        leds[2] = CRGB(0,150,255);
        leds[3] = CRGB(0,150,255);

        leds[4] = CRGB(0,150,255); //blue
        leds[5] = CRGB(0,150,255);
        leds[6] = CRGB(0,150,255);
        leds[7] = CRGB(0,150,255);

        leds[8] = CRGB(0,150,255); //blue
        leds[9] = CRGB(0,150,255);
        leds[10] = CRGB(0,150,255);
        leds[11] = CRGB(0,150,255);

        // leds[12] = CRGB(0,150,255); //blue
        // leds[13] = CRGB(0,150,255);
        // leds[14] = CRGB(0,150,255);
        // leds[15] = CRGB(0,150,255);

        // leds[16] = CRGB(0,150,255); //blue
        // leds[17] = CRGB(0,150,255);
        // leds[18] = CRGB(0,150,255);
        // leds[19] = CRGB(0,150,255);

        // leds[20] = CRGB(0,150,255); //blue
        // leds[21] = CRGB(0,150,255);
        // leds[22] = CRGB(0,150,255);
        // leds[23] = CRGB(0,150,255);

        // leds[24] = CRGB(0,150,255); //blue
        // leds[25] = CRGB(0,150,255);
        // leds[26] = CRGB(0,150,255);
        // leds[27] = CRGB(0,150,255);

        // leds[28] = CRGB(0,150,255); //blue
        // leds[29] = CRGB(0,150,255);
        // leds[30] = CRGB(0,150,255);
        // leds[31] = CRGB(0,150,255);

    digitalWrite(relayPin,LOW);
    // //-----------------------------------------------------

 }
 FastLED.show();
}

void loop()
{
   //1) Speed Select
   //2) Configuration Select 
   //3) Check RPM Cutoff
  
   speed_select();
   configuration_select();
   check_rpm_cutoff();
}
