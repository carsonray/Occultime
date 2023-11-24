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
		static uint32_t pulseCount;

		//Stamps first pps signal
		static uint16_t ppsStamp;

		//Average Arduino clock cycles per real second
		static uint32_t cyclesPerSecond;

		//Base pulseLength (clock cycles)
		static uint16_t pulseLength;

		//pulseLength error every 2^24 clock cycles
		static uint16_t pulseLengthError;

		//Flag to begin calibration
		static bool calibrateFlag;

		//Flag when data is updated
		static bool updateFlag;

		//Flag to calculate error
		static bool calcFlag;

		//Pin for data transmission
		static uint8_t dataPin;

		//Flag when data transmission is enabled
		static bool dataEnabled;

		//Data transmission buffer
		static uint64_t dataBuffer;

		//Open space in data buffer
		static uint8_t dataOpen;

		//Current data type
		static uint8_t dataType;

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

		static uint32_t adjustedMicros();

		static void setTime();

		static void calibrateWave();

		static void addSeconds(uint8_t secondDiff);
		static void addMinutes(uint8_t minuteDiff);
		static void addHours(uint8_t hourDiff);
		static void addDays(uint8_t dayDiff);
		static void addYears(uint16_t yearDiff);

		//TinyGPSPlus object
		static TinyGPSPlus* gps;
	public:
    	static void setGPS(TinyGPSPlus* gps);

		static void begin();
		static void update();

		static void enableWave();
		static void enableWave(uint8_t wavePin, uint16_t pulseLength);
		static void disableWave();
		static void ppsEvent();
		static void sendWave();

		static void enableData();
		static void enableData(uint8_t dataPin);
		static void disableData();
		static void sendDataBit();
		static void resetData();

		static uint32_t totalCycles();
		static uint32_t totalCycles(uint32_t timestamp);
		static void calibrateSecond();
		static void calibrateSecond(uint32_t microsPerSecond);
		static void nextWaveInterrupt();

		static bool getWaveEnabled();
		static bool getDataEnabled();
		static void incrementOvf();
		
		static uint16_t year();
		static uint8_t month();
		static uint8_t day();

		static uint8_t hour();
		static uint8_t minute();
		static uint8_t second();
		static uint32_t microsecond();

		static uint32_t getCyclesPerSecond();
		static uint16_t getPulseLength();
		static uint16_t getPulseLengthError();

		static bool isUpdated();
		static bool isPPSActive();
};

#endif
