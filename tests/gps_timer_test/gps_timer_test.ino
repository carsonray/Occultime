#include <TinyGPSPlus.h>
#include <GPSTimer.h>

/*
  Demonstrates microsecond-accuracy timing using GPS module
*/

//Serial settings
#define Rx 4
#define Tx 3

//PPS attacted to input capture pin 8

//TinyGPSPlus object
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  sssBegin();
  GPSTimer::begin();
}

void loop() {
  //Prints number of satellites
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  //Prints current date if valid
  Serial.print("Date: ");
  if (gps.date.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d/%02d/%02d", GPSTimer::month(), GPSTimer::day(), GPSTimer::year());
    Serial.println(buffer);
  } else {
    Serial.println();
  }

  //Prints current time if valid
  Serial.print("Time: ");
  if (gps.time.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d:%02d:%02d:%06lu", GPSTimer::hour(), GPSTimer::minute(), GPSTimer::second(), GPSTimer::microsecond());
    Serial.println(buffer);
  } else {
    Serial.println();
  }
  Serial.print("Cycles per Second: ");
  Serial.println(GPSTimer::getCyclesPerSecond());

  //Notifies if GPS data not recieved
  if (millis() > 500 && gps.charsProcessed() < 10)
    Serial.println("No GPS data received");

  //Delays and updates
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
      GPSTimer::update();
  } while (millis() - start < ms);
}