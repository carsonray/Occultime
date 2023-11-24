#include <TinyGPSPlus.h>
#include <GPSTimer.h>

/*
  Encodes GPS time into square wave audio tone
  Transmits serial data about time and location
*/

//Serial settings
#define Rx 4
#define Tx 3

//PPS attacted to input capture pin 8

//Tone pin
#define tonePin 12

//Data pin
#define dataPin 13

//Real frequency
#define realFreq 500

//TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  sssBegin();
  GPSTimer::setGPS(&gps);
  GPSTimer::enableWave(tonePin, realFreq);
  GPSTimer::enableData(dataPin);
  GPSTimer::begin();
}

void loop() {
  /*
  Serial.print("Cycles: ");
  Serial.println(GPSTimer::getCyclesPerSecond());
  Serial.print("Length: ");
  Serial.println(GPSTimer::getPulseLength());
  Serial.print("Error: ");
  Serial.println(GPSTimer::getPulseLengthError());
  */
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

      //Updates timer
      GPSTimer::update();
  } while (millis() - start < ms);
}