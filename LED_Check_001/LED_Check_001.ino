#include <FastLED.h>
#define LED_PIN     2
#define NUM_LEDS    32 
CRGB leds[NUM_LEDS];

void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

}

void loop() {
  // put your main code here, to run repeatedly:
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

        leds[12] = CRGB(0,150,255); //blue
        leds[13] = CRGB(0,150,255);
        leds[14] = CRGB(0,150,255);
        leds[15] = CRGB(0,150,255);

        leds[16] = CRGB(0,150,255); //blue
        leds[17] = CRGB(0,150,255);
        leds[18] = CRGB(0,150,255);
        leds[19] = CRGB(0,150,255);

        leds[20] = CRGB(0,150,255); //blue
        leds[21] = CRGB(0,150,255);
        leds[22] = CRGB(0,150,255);
        leds[23] = CRGB(0,150,255);

        leds[24] = CRGB(0,150,255); //blue
        leds[25] = CRGB(0,150,255);
        leds[26] = CRGB(0,150,255);
        leds[27] = CRGB(0,150,255);

        leds[28] = CRGB(0,150,255); //blue
        leds[29] = CRGB(0,150,255);
        leds[30] = CRGB(0,150,255);
        leds[31] = CRGB(0,150,255);

        FastLED.show();
        delay(1000);

}
