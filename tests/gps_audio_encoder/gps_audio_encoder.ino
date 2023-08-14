#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <GPSTimer.h>

/*
  Encodes GPS time into square wave audio tone
*/

//Serial settings
#define Rx 8
#define Tx 9

//PPS pin
#define ppsPin 2

//Tone pin
#define tonePin 5

//Real frequency
#define realFreq 2000

//Serial connection object
SoftwareSerial ss(Rx, Tx);

//TinyGPSPlus object
TinyGPSPlus gps;

//GPS timer object
GPSTimer timer = GPSTimer(&gps);

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  GPSTimer::attachPPS(ppsPin);
  GPSTimer::enableWave(tonePin, realFreq);
  timer.begin();
}

void loop() {
  while (ss.available())
      //Feeds gps object
      gps.encode(ss.read());

  //Updates timer object
  timer.update();
}