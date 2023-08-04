/*
  GPSTimer.h - Handles synchronized microsecond timing based on GPS PPS signal
  Created by Carson G. Ray, August 12 2022.
*/

#ifndef GPSTimer_h
#define GPSTime_h

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPSTimer {
	private:
		//TinyGPSPlus object
		TinyGPSPlus* gps;

		//PPS pin
		uint8_t ppsPin;

		//Flag when pps is active
		bool ppsActive = false;

		//Square wave pin
		uint8_t wavePin;

		//Flag when wave pin is enabled
		uint8_t waveEnabled = false;

		//Square wave frequency (Hz)
		uint16_t frequency = 1;

		//Current wave pin state
		bool waveState = false;

		//Counts timer overflows
		uint16_t ovfCount = 0;

		//Counts square wave half-pulses
		uint16_t halfPulseCount = 0;

		//Average Arduino clock cycles per real second
		uint32_t cyclesPerSecond = 16000000;

		//Tracks current second value to determine updates
		uint8_t currSecond = 0;

		//Flag to begin calibration
		bool calibrateFlag = false;

		//Flag when data is updated
		bool updateFlag = false;

		//Date
		uint16_t years = 2000;
		uint8_t months = 0;
		uint8_t days = 0;

		//Days in each month
		uint8_t monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

		//Time
		uint8_t hours = 0;
		uint8_t minutes = 0;
		uint8_t seconds = 0;

		uint32_t totalCycles();
		uint32_t totalCycles(uint16_t timestamp);
		uint32_t adjustedMicros();
		void calibrateSecond(uint32_t microsPerSecond);
		void nextWaveInterrupt();

		void setTime();

		void addSeconds(uint8_t secondDiff);
		void addMinutes(uint8_t minuteDiff);
		void addHours(uint8_t hourDiff);
		void addDays(uint8_t dayDiff);
		void addYears(uint16_t yearDiff);
	public:
    	GPSTimer(TinyGPSPlus* gps, uint8_t ppsPin);

		void begin();
		void update();

		void enableWave();
		void enableWave(uint8_t wavePin, uint16_t wavelength);
		void disableWave();
		
		uint16_t year();
		uint8_t month();
		uint8_t day();

		uint8_t hour();
		uint8_t minute();
		uint8_t second();
		uint32_t microsecond();

		uint32_t getMicrosPerSecond();
		int32_t getSecondError();

		bool isUpdated();
		bool isPPSActive();
};

#endif
