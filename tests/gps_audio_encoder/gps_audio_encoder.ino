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

  tone(tonePin, 2000);
}

void loop() {
  
  //Delays and updates
  updateDelay(500);
}

static void updateDelay(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    while (ss.available())
      //Feeds gps object
      gps.encode(ss.read());

      //Updates timer object
      timer.update();
  } while (millis() - start < ms);
}