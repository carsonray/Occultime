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
uint16_t GPSTimer::pulseLength = 0;
uint16_t GPSTimer::pulseLengthError = 0;
bool GPSTimer::calibrateFlag = false;
bool GPSTimer::calcFlag = false;
uint8_t GPSTimer::dataPin = 6;
bool GPSTimer::dataEnabled = false;
uint64_t GPSTimer::dataBuffer = 0;
uint8_t GPSTimer::dataRemaining = 0;
uint8_t GPSTimer::dataType = 4;
uint8_t GPSTimer::dataInterval = 1;
bool GPSTimer::locValid = false;
float GPSTimer::lat;
float GPSTimer::lng;
uint32_t GPSTimer::lngBin;
uint32_t GPSTimer::latBin;
bool GPSTimer::timeValid = false;
uint16_t GPSTimer::years = 2000;
uint8_t GPSTimer::months = 0;
uint8_t GPSTimer::days = 0;
uint8_t GPSTimer::monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint8_t GPSTimer::hours = 0;
uint8_t GPSTimer::minutes = 0;
uint8_t GPSTimer::seconds = 0;
TinyGPSPlus* GPSTimer::gps;

//Float to int converter
union {
	float input;
	uint32_t output;
} data;

//Attaches gps object
void GPSTimer::setGPS(TinyGPSPlus* gps) {
	GPSTimer::gps = gps;
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
	if ((!ppsActive || (totalCycles() < 4000000)) && gps->time.isValid() && (gps->time.second() != seconds)) {
		//Calibrates on GPS time update without satellites
		if (!ppsActive) {
			calibrateSecond();

			//Resets timer and overflow counter
			TCNT1 = 0;
			ovfCount = 0;
		}

		//Sets gps information
		setGPSInfo();
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
	enableWave();

	//Calibrates wave parameters
	calibrateWave();
}

//Disables square wave output
void GPSTimer::disableWave() {
	waveEnabled = false;
}

//Updates wave signal parameters every second
void GPSTimer::ppsEvent() {
	//Calibrates second
	calibrateSecond(totalCycles(ICR1));

	//Stamps time
	ppsStamp = ICR1;

	//Updates sqaure wave
	if (waveEnabled && ppsActive) {
		//Starts new square wave
		waveState = true;
		pulseCount = 0;
		GPSTimer::nextWaveInterrupt();
	}

	//Resets overflow counter
	ovfCount = 0;

	//Raises pps active flag
	ppsActive = true;

	//Starts sending new time and location data
	GPSTimer::resetData();
}

//Sends wave signal
void GPSTimer::sendWave() {
	waveState = !waveState;
	nextWaveInterrupt();
}

//Enables serial time and location output every second synchronized with square wave
void GPSTimer::enableData() {
	enableWave();
	dataEnabled = true;
}
void GPSTimer::enableData(uint8_t dataPin) {
	//Sets output pin
	GPSTimer::dataPin = dataPin;

	//Raises wave and data flags
	enableData();

	//Calibrates wave parameters
	calibrateWave();
}

//Disables data output
void GPSTimer::disableData() {
	dataEnabled = false;
}

//Sends bit from data buffer
void GPSTimer::sendDataBit() {
	//If wave state is on
	if (waveState) {
		//If there is a data bit available
		if (dataRemaining > 0) {
			//Writes least signficant bit to data pin
			digitalWrite(dataPin, dataBuffer & 1);
			
			if ((pulseCount/2) % dataInterval == 0) {
				//Shifts out least significant bit and adds to open data
				dataBuffer = dataBuffer >> 1;
				dataRemaining--;
			}
		} else if (dataType < 4) {
			//Adds next data type to buffer
			switch(dataType) {
				case 0:
					//Start bit
					dataBuffer = 1;
					dataRemaining = 1;
					break;
				case 1:
					if (locValid) {
						dataBuffer = (uint64_t)((uint64_t)lngBin<<32)+(uint64_t)latBin;
					} else {
						dataBuffer = 0;
					}

					dataRemaining = 64;
					break;
				case 2:
					//Time information
					if (timeValid) {
						dataBuffer = (uint64_t)((uint64_t)year()<<28);
						dataBuffer += (uint64_t)((uint64_t)month()<<24);
						dataBuffer += (uint64_t)((uint64_t)day()<<18);
						dataBuffer += (uint64_t)((uint64_t)hour()<<12);
						dataBuffer += (uint64_t)((uint64_t)minute()<<6);
						dataBuffer += (uint64_t)second();
					} else {
						dataBuffer = 0;
					}

					dataRemaining = 44;
					break;
				case 3:
					//End bit
					dataBuffer = 1;
					dataRemaining = 1;
					break;
			}
			
			//Sends new bit
			sendDataBit();

			//Increments data type
			dataType++;
		}
	} else {
		digitalWrite(dataPin, false);
	}
}

void GPSTimer::resetData() {
	dataType = 0;
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

boolean GPSTimer::getWaveEnabled() {
	return waveEnabled;
}

boolean GPSTimer::getDataEnabled() {
	return waveEnabled;
}

void GPSTimer::incrementOvf() {
	ovfCount++;
}

void GPSTimer::setGPSInfo() {
	//Time data
	timeValid = gps->time.isValid();
	years = gps->date.year();
	months = gps->date.month();
	days = gps->date.day();

	hours = gps->time.hour();
	minutes = gps->time.minute();
	seconds = gps->time.second();
	
	//Location data
	locValid = gps->location.isValid();
	lat = gps->location.lat();
	lng = gps->location.lng();
	data.input = lat;
	latBin = data.output;
	data.input = lng;
	lngBin = data.output;
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
	return years;
}

uint8_t GPSTimer::month() {
	return months;
}

uint8_t GPSTimer::day() {
	return days;
}

uint8_t GPSTimer::hour() {
	return hours;
}	

uint8_t GPSTimer::minute() {
	return minutes;
}

uint8_t GPSTimer::second() {
	return seconds;
}
uint32_t GPSTimer::microsecond() {
	uint32_t microTime = adjustedMicros();
	return (microTime < 1000000) ? microTime : 999999;
}

uint32_t GPSTimer::getCyclesPerSecond() {
	return cyclesPerSecond;
}

uint16_t GPSTimer::getPulseLength() {
	return pulseLength;
}

uint16_t GPSTimer::getPulseLengthError() {
	return pulseLengthError;
}

//Whether PPS is active
bool GPSTimer::isPPSActive() {
	return ppsActive;
}

//Whether time data is valid
bool GPSTimer::isTimeValid() {
	return timeValid;
}

//Checks for rising edge of PPS signal
ISR(TIMER1_CAPT_vect) {
	cli();
	GPSTimer::ppsEvent();
	sei();
}

ISR(TIMER1_COMPA_vect) {
	if (GPSTimer::getWaveEnabled() && GPSTimer::isPPSActive() && GPSTimer::isTimeValid()) {
		GPSTimer::sendWave();
		if (GPSTimer::getDataEnabled()) {
			GPSTimer::sendDataBit();
		}
	}
}

ISR(TIMER1_OVF_vect) {
	GPSTimer::incrementOvf();
}


