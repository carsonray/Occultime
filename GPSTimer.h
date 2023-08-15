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
		//Flag when pps is active
		static bool ppsActive;

		//Square wave pin
		static uint8_t wavePin;

		//Flag when wave pin is enabled
		static bool waveEnabled;

		//Square wave frequency (Hz)
		static uint16_t frequency;

		//Current wave pin state
		static bool waveState;

		//Counts timer overflows
		static uint16_t ovfCount;

		//Counts square wave half-pulses
		static uint32_t halfPulseCount;

		//Average Arduino clock cycles per real second
		static uint32_t cyclesPerSecond;

		//Flag to begin calibration
		static bool calibrateFlag;

		//Flag when data is updated
		static bool updateFlag;

		//Date
		static uint16_t years;
		static uint8_t months;
		static uint8_t days;

		//Days in each month
		static uint8_t monthDays[12];

		//Time
		static uint8_t hours;
		static uint8_t minutes;
		static uint8_t seconds;

		uint32_t adjustedMicros();

		void setTime();

		static void addSeconds(uint8_t secondDiff);
		static void addMinutes(uint8_t minuteDiff);
		static void addHours(uint8_t hourDiff);
		static void addDays(uint8_t dayDiff);
		static void addYears(uint16_t yearDiff);

		//TinyGPSPlus object
		TinyGPSPlus* gps;
	public:
    	GPSTimer(TinyGPSPlus* gps);

		void begin();
		void update();

		static void enableWave();
		static void enableWave(uint8_t wavePin, uint16_t wavelength);
		static void disableWave();

		static uint32_t totalCycles();
		static uint32_t totalCycles(uint32_t timestamp);
		static void calibrateSecond();
		static void calibrateSecond(uint32_t microsPerSecond);
		static void nextWaveInterrupt();

		static void setWaveState(bool waveState);
		static bool getWaveState();
		static bool getWaveEnabled();
		static void setOvfCount(uint16_t ovfCount);
		static uint16_t getOvfCount();
		static void setHalfPulseCount(uint32_t halfPulseCount);
		static void setPPSActive(bool ppsActive);
		
		uint16_t year();
		uint8_t month();
		uint8_t day();

		uint8_t hour();
		uint8_t minute();
		uint8_t second();
		uint32_t microsecond();

		uint32_t getCyclesPerSecond();

		bool isUpdated();
		bool isPPSActive();
};

#endif
