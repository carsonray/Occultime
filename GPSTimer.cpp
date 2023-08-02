/*
  GPSTimer.h - Handles synchronized microsecond timing based on GPS PPS signal
  Created by Carson G. Ray, August 12 2022.
*/

#include <Arduino.h>
#include "GPSTimer.h"

//Attaches gps object
GPSTimer::GPSTimer(TinyGPSPlus* gps) {
	this->gps = gps;
}

//Attaches pps pin monitor
void GPSTimer::attachPPS(Button* pps) {
	this->pps = pps;

	//Raises pps attachment flag
	ppsFlag = true;
}

//Calibrates timing using GPS
void GPSTimer::update() {
	if (ppsFlag && pps->changeTo(HIGH)) {
		//Calibrates on PPS rising edge
		calibrateSecond();

		//Raises pps active flag
		ppsActive = true;

		//Enables calibration after a reference PPS signal
		calibrateFlag = true;
	} else if (gps->time.second() != currSecond) {
		//Updates current second
		currSecond = gps->time.second();

		//Sets time
		setTime();

		//Calibrates on GPS time update without satellites
		if (!ppsActive) {
			calibrateSecond();

			//Enables calibration after a reference time update
			calibrateFlag = true;
		}
	} else if (rawMicros() > 2000000) {
		//Shifts time reference to avoid overflow
		microStart = micros();

		//Resets flags
		calibrateFlag = false;
		updateFlag = false;
		ppsActive = false;
	}

	//Updates pps monitor
	pps->update();
}

uint32_t GPSTimer::rawMicros() {
	return micros() - microStart;
}

//Gets adjusted microseconds
uint32_t GPSTimer::realMicros() {
  //Gets elapsed time
  int32_t elapsed = (int32_t) rawMicros();
  
  //Subtracts microsecond error every second
  return elapsed - elapsed*secondError/1000000;
}

//Calculates error in Arduino clock every second
void GPSTimer::calibrateSecond() {
	//Gets elapsed time since last calibration
	uint32_t current = micros();
	uint32_t elapsed = current - microStart;

	//Increments seconds and resets milliseconds
	addSeconds(1);
	microStart = current;

	//Only calibrates if two PPS signals are received
	if (calibrateFlag) {
		//Gets error in microseconds every second
		microsPerSecond = elapsed;
		secondError = ((int32_t) elapsed - 1000000)*1000000/(int32_t) elapsed;
	}

	//Sets update flag
	updateFlag = true;
}

void GPSTimer::setTime() {
	years = gps->date.year();
	months = gps->date.month() - 1;
	days = gps->date.day() - 1;

	hours = gps->time.hour();
	minutes = gps->time.minute();
	seconds = gps->time.second();
}

void GPSTimer::addSeconds(uint8_t secondDiff) {
	seconds += secondDiff;
	uint8_t minuteDiff = seconds / 60;
	seconds = seconds % 60;
	if (minuteDiff > 0) {
		addMinutes(minuteDiff);
	}
}

void GPSTimer::addMinutes(uint8_t minuteDiff) {
	minutes += minuteDiff;
	uint8_t hourDiff = minutes / 60;
	minutes = minutes % 60;
	if (hourDiff > 0) {
		addHours(hourDiff);
	}
}

void GPSTimer::addHours(uint8_t hourDiff) {
	hours += hourDiff;
	uint8_t dayDiff = hours / 24;
	hours = hours % 24;
	if (dayDiff > 0) {
		addDays(dayDiff);
	}
}

void GPSTimer::addDays(uint8_t dayDiff) {
	days += dayDiff;
	while (true) {
		uint8_t maxDays = monthDays[months];
		if (days >= maxDays) {
			days -= maxDays;
			months++;
			if (months >= 12) {
				addYears(1);
			}
			months %= 12;
		} else {
			break;
		}
	}
}

void GPSTimer::addYears(uint16_t yearDiff) {
	years += yearDiff;
}

uint16_t GPSTimer::year() {
	updateFlag = false;
	return years;
}

uint8_t GPSTimer::month() {
	updateFlag = false;
	return months + 1;
}

uint8_t GPSTimer::day() {
	updateFlag = false;
	return days + 1;
}

uint8_t GPSTimer::hour() {
	updateFlag = false;
	return hours;
}	

uint8_t GPSTimer::minute() {
	updateFlag = false;
	return minutes;
}

uint8_t GPSTimer::second() {
	updateFlag = false;
	return seconds;
}
uint32_t GPSTimer::microsecond() {
	updateFlag = false;
	uint32_t microTime = realMicros();
	return (microTime < 1000000) ? microTime : 999999;
}

uint32_t GPSTimer::getMicrosPerSecond() {
	updateFlag = false;
	return microsPerSecond;
}

int32_t GPSTimer::getSecondError() {
	updateFlag = false;
	return secondError;
}

//Whether calibration has been updated since last query
bool GPSTimer::isUpdated() {
	return updateFlag;
}

