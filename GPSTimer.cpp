/*
  GPSTimer.h - Handles synchronized microsecond timing based on GPS PPS signal
  Created by Carson G. Ray, August 12 2022.
*/

#include <Arduino.h>
#include "GPSTimer.h"

//Attaches gps object
GPSTimer::GPSTimer(TinyGPSPlus* gps, uint8_t ppsPin) {
	this->gps = gps;
	this->ppsPin = ppsPin;
	pinMode(ppsPin, INPUT);
}

//Initializes timer and interrupts
void GPSTimer::begin() {
	//Disables interrupts
	cli();
	
	TCCR1A = 0;           // Init Timer1A
  	TCCR1B = 0;           // Init Timer1B
  	TCCR1B |= B11000001;  // Internal Clock, Prescaler = 1, ICU Filter EN, ICU Pin RISING
  	TIMSK1 |= B00100001;  // Enable Timer CAPT and OVF Interrupts

	//Enables interrupts
	sei();
}

//Calibrates timing using GPS
void GPSTimer::update() {
	//If PPS is active, only do expensive time check after calibration
	if ((!ppsActive || (timerCount < 4000000)) && (gps->time.second() != currSecond)) {
		//Calibrates on GPS time update without satellites
		if (!ppsActive) {
			calibrateSecond();
		}

		//Sets time
		setTime();

		//Updates current second
		currSecond = gps->time.second();
	}

	if (totalCycles() > 32000000) {
		//Resets flags
		calibrateFlag = false;
		updateFlag = false;
		ppsActive = false;
	}
}

//Enables calibrated square wave output
void GPSTimer::enableWave() {
	waveEnabled = true;
}
void GPSTimer::enableWave(uint8_t wavePin, uint16_t frequency) {
	this->wavePin = wavePin;
	this->frequency = frequency;

	//Raises square wave flag
	waveEnabled = true;
}

//Disables square wave output
void GPSTimer::disableWave() {
	waveEnabled = false;
}

//Gets total clock cycles since last second
uint32_t GPSTimer::totalCycles() {
	return totalCycles(timer1Val);
}

//Gets total clock cycles based on timestamp
uint32_t GPSTimer::totalCycles(uint16_t timestamp) {
	return ovfCount*65536 + timestamp;
}

//Gets adjusted microseconds
uint32_t GPSTimer::adjustedMicros() {
  //Gets elapsed clock cycles
  int32_t cycles = (int32_t) totalCycles();
  
  //Converts to microseconds
  return cycles*1000000/cyclesPerSecond;
}

//Calculates error in Arduino clock every second
void GPSTimer::calibrateSecond(uint32_t microsPerSecond) {
	//Increments seconds
	addSeconds(1);

	//Only calibrates if two PPS signals are received
	if (calibrateFlag) {
		//Gets error in microseconds every second
		this->microsPerSecond = microsPerSecond;

		//Sets update flag
		updateFlag = true;
	}

	//Enables calibration after a reference time update
	calibrateFlag = true;
}

//Sets next square wave interrupt
void GPSTimer::nextWaveInterrupt() {
	//Writes current state
	digitalWrite(wavePin, waveState);

	//Increments half pulse counter
	halfPulseCount++;

	//Sets interrupt point at next half pulse
	OCR1A = (cyclesPerSecond*halfPulseCount/(frequency*2)) % 65536;
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
	uint32_t microTime = adjustedMicros();
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

//Whether PPS is active
bool GPSTimer::isPPSActive() {
	updateFlag = false;
	return ppsActive;
}

//Checks for rising edge of PPS signal
ISR(TIMER1_CAPT_vect) {
	cli();

	//Resets timer
	TCNT1 = 0;

	//Calibrates second
	calibrateSecond(totalCycles(ICR1));

	//Updates sqaure wave
	if (waveEnabled && updateFlag) {
		//Enables COMPA interrupt
		TIMSK1 |= B00000010;
		
		//Starts new square wave
		waveState = true;
		halfPulseCount = 0;
		nextWaveInterrupt();
	} else {
		//Disables COMPA interrupt
		TIMSK1 &= B11111101;
	}

	//Resets overflow counter
	ovfCount = 0;

	//Raises pps active flag
	ppsActive = true;

	sei();
}

ISR(TIMER1_COMPA_vect) {
	if (waveEnabled) {
		waveState = !waveState;
		nextWaveInterrupt();
	}
}

ISR(TIMER1_OVF_vect) {
	ovfCount++;
}


