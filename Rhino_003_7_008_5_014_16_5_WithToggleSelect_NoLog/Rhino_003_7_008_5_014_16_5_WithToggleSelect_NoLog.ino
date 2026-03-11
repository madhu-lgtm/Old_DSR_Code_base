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
#define DT 50

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

    static const int indices0[] = {1, 2, 3, 4, 5, 10, 12, 14, 16};
    static const int indices1[] = {1, 2, 3, 4, 6, 11, 15};
    static const int indices2[] = {1, 2, 3, 6, 7, 10, 12, 13};
    static const int indices3[] = {1, 2, 6, 7, 8, 11, 14};
    static const int indices4[] = {1, 6, 7, 8, 9, 10, 13, 15, 16};

    for (int i = 0; i < BLOCK_MAX; i++) {
        check_seeder(0, 16, indices0, 9);
        check_seeder(1, 12, indices1, 7);
        check_seeder(2, 8, indices2, 8);
        check_seeder(3, 4, indices3, 7);
        check_seeder(4, 0, indices4, 9);
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
