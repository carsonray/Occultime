#include <TinyGPSPlus.h>
#include <GPSTimer.h>

/*
  Encodes GPS time into square wave audio tone
  Transmits serial data about time and location
*/

//Serial settings
#define Rx 3
#define Tx 4

//PPS attacted to input capture pin 8

//Tone pin
#define tonePin 10

//Data pin
#define dataPin 11

//Real frequency
#define realFreq 1000

//Data interval
#define dataInterval 4

//TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  //Serial.begin(115200);
  sssBegin();
  GPSTimer::setGPS(&gps);
  GPSTimer::enableWave(tonePin, realFreq);
  GPSTimer::enableData(dataPin, dataInterval);
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