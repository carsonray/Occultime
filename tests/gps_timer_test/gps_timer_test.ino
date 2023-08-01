#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Button.h>

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




//Last Arduino microsecond reading
uint32_t prevMicros = 0;

//Average Arduino microsecond error per arduino second
int32_t microError = 0;

//Flag to begin calibration
bool calibrateFlag = false;

//Flag when GPS data is updated
bool updateFlag = false;





void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop() {
  //Prints number of satellites
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  //Prints current date if valid
  Serial.print("Date: ");
  if (gps.date.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d/%02d/%02d", gps.date.month(), gps.date.day(), gps.date.year());
    Serial.println(buffer);
  } else {
    Serial.println();
  }

  //Prints current time if valid
  Serial.print("Time: ");
  if (gps.time.isValid()) {
    char buffer[32];
    sprintf(buffer, "%02d:%02d:%02d:%06lu", gps.time.hour(), gps.time.minute(), gps.time.second(), realMicros());
    Serial.println(buffer);
  } else {
    Serial.println();
  }
  Serial.print("Error: ");
  Serial.println(microError);

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

      //Calibrates Arduino microseconds
      if (pps.changeTo(HIGH)) {
        //Based on PPS signal
        calibrateMicros();
        calibrateFlag = true;
      } else if ((gps.satellites.value() == 0) && (gps.time.isUpdated()) && (!updateFlag)) {
        //Based on GPS updates
        calibrateMicros();
        calibrateFlag = true;
        updateFlag = true;
      } else if (micros() - prevMicros > 2000000) {
        //Shifts reference frame to avoid overflow
        shiftMicros();
        calibrateFlag = false;
        updateFlag = false;
      } else if (!gps.time.isUpdated()) {
        //Resets update flag
        updateFlag = false;
      }

      //Updates pps monitor
      pps.update();
  } while (millis() - start < ms);
}

//Gets adjusted microseconds
static uint32_t realMicros() {
  //Gets elapsed time
  int32_t elapsed = (int32_t) (micros() - prevMicros);
  
  //Subtracts microsecond error every second
  return (elapsed - elapsed*microError/1000000) % 1000000;
}

//Calculates error in Arduino clock based on PPS signal
static void calibrateMicros() {
  //Gets elapsed time
  uint32_t elapsed = micros() - prevMicros;

  //Only calibrates if two PPS signals are received
  if (calibrateFlag) {
    //Gets error in microseconds every second
    microError = ((int32_t) elapsed - 1000000)*1000000/(int32_t) elapsed;
  }

  //Resets microsecond counter
  prevMicros += elapsed;
}

//Shifts microsecond reference frame to avoid overflow while accounting for error
static void shiftMicros() {
  prevMicros = micros() - realMicros();
}
