#include <TinyGPSPlus.h>
#include <GPSTimer.h>

/*
  Encodes GPS time into square wave audio tone
*/

//Serial settings
#define Rx 4
#define Tx 3

//PPS attacted to input capture pin 8

//Tone pin
#define tonePin 13

//Real frequency
#define realFreq 5000

//TinyGPSPlus object
TinyGPSPlus gps;

//GPS timer object
GPSTimer timer = GPSTimer(&gps);

void setup() {
  Serial.begin(115200);
  sssBegin();
  GPSTimer::enableWave(tonePin, realFreq);
  timer.begin();
}

void loop() {
  Serial.println(timer.getCyclesPerSecond());
  updateDelay(500);
}

static void updateDelay(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    while (sssAvailable())
      //Feeds gps object
      gps.encode(sssRead());

      //Updates timer object
      timer.update();
  } while (millis() - start < ms);
}