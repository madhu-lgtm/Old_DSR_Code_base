
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
#include <RMCS2303drive.h>

#define LED_PIN     2
#define NUM_LEDS    20
#define RELAY_PIN   10
#define INPUT_PWM_1 9
#define INPUT_PWM_2 4

#define MIN_PWM 1051
#define MAX_PWM 1951
#define MIN_RPM 0
#define MAX_RPM 6500
#define RPM_CUTOFF 1250
#define BLOCK_MAX 2
#define DT 200

RMCS2303 rmcs;
CRGB leds[NUM_LEDS];
byte motor_ids[] = {1, 2, 3, 4, 5};
double GR = 0.0111;
int blockage_counters[5] = {0};
int speed = 0;
int configs[17] = {0};

struct ConfigRange { int min, max; };
ConfigRange configRanges[17] = {
    {1076, 1126}, {1126, 1176}, {1176, 1226}, {1226, 1276}, {1276, 1326},
    {1323, 1376}, {1376, 1426}, {1426, 1476}, {1476, 1526}, {1526, 1576},
    {1576, 1626}, {1626, 1676}, {1676, 1726}, {1726, 1776}, {1776, 1826},
    {1826, 1876}, {1876, 1926}
};

void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    rmcs.Serial_selection(0);
    rmcs.Serial0(9600);
    rmcs.begin(&Serial3, 9600);

    for (byte id : motor_ids) {
        rmcs.WRITE_PARAMETER(id, 260, 32, 16, 32, 11, 5000, speed);
        rmcs.Disable_Digital_Mode(id, 0);
    }
}

void speed_select() {
    int pwm = pulseIn(INPUT_PWM_1, HIGH);
    speed = constrain(map(pwm, MIN_PWM, MAX_PWM, MIN_RPM, MAX_RPM), MIN_RPM, MAX_RPM);
}

void configuration_select() {
    int pwm = pulseIn(INPUT_PWM_2, HIGH);
    memset(configs, 0, sizeof(configs));
    for (int i = 0; i < 17; i++) {
        if (pwm >= configRanges[i].min && pwm <= configRanges[i].max) {
            configs[i] = 1;
            break;
        }
    }
}

bool checkConfigs(const int indices[], int size) {
    for (int i = 0; i < size; i++) {
        if (configs[indices[i]]) return true;
    }
    return false;
}

void check_seeder(byte index, int ledStart, const int configIndices[], int configSize) {
    if (checkConfigs(configIndices, configSize)) {
        rmcs.Enable_Digital_Mode(motor_ids[index], 0);
        fill_solid(leds + ledStart, 4, CRGB(0, 255, 0));
        rmcs.Speed(motor_ids[index], speed);
        int feedback = rmcs.Speed_Feedback(motor_ids[index]);
        double rpm = feedback * GR;
        if (rpm == 0) blockage_counters[index]++;
        if (blockage_counters[index] >= BLOCK_MAX) {
            fill_solid(leds + ledStart, 4, CRGB(255, 0, 0));
            rmcs.Disable_Digital_Mode(motor_ids[index], 0);
            digitalWrite(RELAY_PIN, HIGH);
        }
    } else {
        fill_solid(leds + ledStart, 4, CRGB(255, 0, 255));
        rmcs.Disable_Digital_Mode(motor_ids[index], 0);
    }
}

void check_rpm_cutoff() {
    if (speed < RPM_CUTOFF) {
        for (byte id : motor_ids) rmcs.Disable_Digital_Mode(id, 0);
        fill_solid(leds, NUM_LEDS, CRGB(0, 150, 255));
        FastLED.show();
        return;
    }

    memset(blockage_counters, 0, sizeof(blockage_counters));
    for (int i = 0; i < BLOCK_MAX; i++) {
        check_seeder(0, 16, (int[]){1, 2, 3, 4, 5, 10, 12, 14, 16}, 9);
        check_seeder(1, 12, (int[]){1, 2, 3, 4, 6, 11, 15}, 7);
        check_seeder(2, 8, (int[]){1, 2, 3, 6, 7, 10, 12, 13}, 8);
        check_seeder(3, 4, (int[]){1, 2, 6, 7, 8, 11, 14}, 7);
        check_seeder(4, 0, (int[]){1, 6, 7, 8, 9, 10, 13, 15, 16}, 9);
        delay(DT);
    }
    check_relay();
    FastLED.show();
}

void check_relay() {
    for (int i = 0; i < 5; i++) {
        if (blockage_counters[i] >= BLOCK_MAX) return;
    }
    digitalWrite(RELAY_PIN, LOW);
}

void loop() {
    speed_select();
    configuration_select();
    check_rpm_cutoff();
}
