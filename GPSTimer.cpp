/*
  GPSTimer.h - Handles synchronized microsecond timing based on GPS PPS signal
  Created by Carson G. Ray, August 12 2022.
*/

#include <Arduino.h>
#include "GPSTimer.h"

//Initializes static members
bool GPSTimer::ppsActive = false;
uint8_t GPSTimer::wavePin = 5;
bool GPSTimer::waveEnabled = false;
uint16_t GPSTimer::frequency = 1;
bool GPSTimer::waveState = false;
uint16_t GPSTimer::ovfCount = 0;
uint32_t GPSTimer::pulseCount = 0;
uint16_t GPSTimer::ppsStamp = 0;
uint32_t GPSTimer::cyclesPerSecond = 16000000;
uint16_t GPSTimer:: pulseLength = 0;
uint16_t GPSTimer::pulseLengthError = 0;
bool GPSTimer::calibrateFlag = false;
bool GPSTimer::updateFlag = false;
bool GPSTimer::calcFlag = false;
uint16_t GPSTimer::years = 2000;
uint8_t GPSTimer::months = 0;
uint8_t GPSTimer::days = 0;
uint8_t GPSTimer::monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint8_t GPSTimer::hours = 0;
uint8_t GPSTimer::minutes = 0;
uint8_t GPSTimer::seconds = 0;

//Attaches gps object
GPSTimer::GPSTimer(TinyGPSPlus* gps) {
	this->gps = gps;
}

//Initializes timer and interrupts
void GPSTimer::begin() {
	//Disables interrupts
	cli();
	
	TCCR1A = 0;           // Init Timer1A
  	TCCR1B = 0;           // Init Timer1B
  	TCCR1B |= B11000001;  // Internal Clock, Prescaler = 1, ICU Noise Filter, ICU Pin RISING
  	TIMSK1 |= B00100011;  // Enable Timer CAPT, COMPA, and OVF Interrupts
	OCR1A = 0;

	//Enables interrupts
	sei();
}

//Calibrates timing using GPS
void GPSTimer::update() {
	//If PPS is active, only do expensive time check after calibration
	if ((!ppsActive || (totalCycles() < 4000000)) && (gps->time.second() != seconds)) {
		//Calibrates on GPS time update without satellites
		if (!ppsActive) {
			calibrateSecond();

			//Resets timer and overflow counter
			TCNT1 = 0;
			ovfCount = 0;
		}

		//Sets time
		setTime();
	}

	if (calcFlag) {
		//Runs expensive time error calculation
		calibrateWave();

		calcFlag = false;
	}

	if (totalCycles() > 32000000) {
		//Resets overflow count
		ovfCount = 0;

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
	GPSTimer::wavePin = wavePin;
	GPSTimer::frequency = frequency;

	//Raises square wave flag
	waveEnabled = true;

	//Calibrates wave parameters
	calibrateWave();
}

//Disables square wave output
void GPSTimer::disableWave() {
	waveEnabled = false;
}

//Calibrates pulseLength and error from cyclesPerSecond
void GPSTimer::calibrateWave() {
	pulseLength = cyclesPerSecond/(frequency*2);
	pulseLengthError = (cyclesPerSecond - pulseLength*(frequency*2));
}

//Gets total clock cycles since last second
uint32_t GPSTimer::totalCycles() {
	return totalCycles(TCNT1);
}

//Gets total clock cycles based on timestamp
uint32_t GPSTimer::totalCycles(uint32_t timestamp) {
	return ovfCount*65536 + timestamp - ppsStamp;
}

//Gets adjusted microseconds
uint32_t GPSTimer::adjustedMicros() {
  //Gets elapsed clock cycles
  int64_t cycles = (uint64_t) totalCycles();
  
  //Converts to microseconds
  return (uint32_t) (cycles*1000000/cyclesPerSecond);
}

//Calculates error in Arduino clock every second
void GPSTimer::calibrateSecond() {
	calibrateSecond(totalCycles());
}
void GPSTimer::calibrateSecond(uint32_t cyclesPerSecond) {
	//Increments seconds
	addSeconds(1);

	//Only calibrates if two PPS signals are received
	if (calibrateFlag) {
		//Gets error in microseconds every second
		GPSTimer::cyclesPerSecond = cyclesPerSecond;

		//Sets update flag
		updateFlag = true;

		//Sets calculation flag
		calcFlag = true;
	}

	//Enables calibration after a reference time update
	calibrateFlag = true;
}

//Sets next square wave interrupt
void GPSTimer::nextWaveInterrupt() {
	//Writes current state
	digitalWrite(wavePin, waveState);

	//Increments half pulse counter
	pulseCount++;

	//Sets interrupt point at next half pulse
	OCR1A = (uint16_t) (pulseLength*pulseCount + (pulseLengthError*pulseCount)/(frequency*2));
}

void GPSTimer::setWaveState(bool waveState) {
	GPSTimer::waveState = waveState;
}

boolean GPSTimer::getWaveState() {
	return waveState;
}

boolean GPSTimer::getWaveEnabled() {
	return waveEnabled;
}

void GPSTimer::setOvfCount(uint16_t ovfCount) {
	GPSTimer::ovfCount = ovfCount;
}

uint16_t GPSTimer::getOvfCount() {
	return ovfCount;
}

void GPSTimer::setPulseCount(uint32_t pulseCount) {
	GPSTimer::pulseCount = pulseCount;
}

void GPSTimer::setPPSActive(bool ppsActive) {
	GPSTimer::ppsActive = ppsActive;
}

void GPSTimer::setPPSStamp(uint16_t ppsStamp) {
	GPSTimer::ppsStamp = ppsStamp;
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

uint32_t GPSTimer::getCyclesPerSecond() {
	updateFlag = false;
	return cyclesPerSecond;
}

uint16_t GPSTimer::getPulseLength() {
	return pulseLength;
}

uint16_t GPSTimer::getPulseLengthError() {
	return pulseLengthError;
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

	//Calibrates second
	GPSTimer::calibrateSecond(GPSTimer::totalCycles(ICR1));

	//Stamps time
	GPSTimer::setPPSStamp(ICR1);

	//Updates sqaure wave
	if (GPSTimer::getWaveEnabled()) {
		//Starts new square wave
		GPSTimer::setWaveState(true);
		GPSTimer::setPulseCount(0);
		GPSTimer::nextWaveInterrupt();
	}

	//Resets overflow counter
	GPSTimer::setOvfCount(0);

	//Raises pps active flag
	GPSTimer::setPPSActive(true);

	sei();
}

ISR(TIMER1_COMPA_vect) {
	if (GPSTimer::getWaveEnabled()) {
		GPSTimer::setWaveState(!GPSTimer::getWaveState());
		GPSTimer::nextWaveInterrupt();
	}
}

ISR(TIMER1_OVF_vect) {
	GPSTimer::setOvfCount(GPSTimer::getOvfCount() + 1);
}


