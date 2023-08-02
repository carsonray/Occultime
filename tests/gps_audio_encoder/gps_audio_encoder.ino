#include <TinyGPSPlus.h>
#include <AltSoftSerial.h>
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
AltSoftSerial ss(Rx, Tx);

//TinyGPSPlus object
TinyGPSPlus gps;

//GPS timer object
GPSTimer timer = GPSTimer(&gps);

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  
  timer.attachPPS(ppsPin);
}

void loop() {
  while (ss.available())
      //Feeds gps object
      gps.encode(ss.read());

  //Updates timer object
  timer.update();

  if (timer.isUpdated()) {
    uint16_t toneFreq = (uint16_t) realFreq*1000000/timer.getMicrosPerSecond();
    tone(tonePin, toneFreq);
    Serial.println(toneFreq);
  }
}