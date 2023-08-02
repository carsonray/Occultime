#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Button.h>
#include <GPSTimer.h>

/*
  Demonstrates microsecond-accuracy timing using GPS module
*/

//Serial settings
#define Rx 4
#define Tx 3
static const uint32_t GPSBaud = 9600;

//PPS pin
#define ppsPin 2




//Serial connection object
SoftwareSerial ss(Rx, Tx);

//PPS monitor
Button pps = Button(ppsPin, 0);

//TinyGPSPlus object
TinyGPSPlus gps;

//GPS timer object
GPSTimer timer = GPSTimer(&gps);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  timer.attachPPS(&pps);
}

void loop() {
  //Prints number of satellites
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  //Prints current date if valid
  Serial.print("Date: ");
  if (gps.date.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d/%02d/%02d", timer.month(), timer.day(), timer.year());
    Serial.println(buffer);
  } else {
    Serial.println();
  }

  //Prints current time if valid
  Serial.print("Time: ");
  if (gps.time.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d:%02d:%02d:%06lu", timer.hour(), timer.minute(), timer.second(), timer.microsecond());
    Serial.println(buffer);
  } else {
    Serial.println();
  }
  Serial.print("us per Second: ");
  Serial.println(timer.getMicrosPerSecond());
  Serial.print("Second Error: ");
  Serial.println(timer.getSecondError());

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
    while (ss.available())
      //Feeds gps object
      gps.encode(ss.read());

      //Updates timer object
      timer.update();
  } while (millis() - start < ms);
}