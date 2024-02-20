/*
  GPSTimer.h - Handles synchronized microsecond timing based on GPS PPS signal
  Created by Carson G. Ray, August 12 2022.
*/

#ifndef GPSTimer_h
#define GPSTime_h

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>

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

		static uint16_t prevOvfCount;

		//Counts square wave half-pulses
		static uint16_t pulseCount;

		//Stamps first pps signal
		static uint16_t ppsStamp;

		//Average Arduino clock cycles per real second
		static uint32_t cyclesPerSecond;

		//Base pulseLength (clock cycles)
		static uint16_t pulseLength;

		//pulseLength error every 2^24 clock cycles
		static uint16_t pulseLengthError;

		//Accumulated error correction
		static uint16_t correctionSum;

		//Current error correction
		static uint16_t errorCorrection;

		//Projected pulse length correction
		static uint16_t projectedCorrection;

		//Flag to begin calibration
		static bool calibrateFlag;

		//Flag to calculate error
		static bool calcFlag;

		//Pin for data transmission
		static uint8_t dataPin;

		static uint8_t dataCount;

		//Flag when data transmission is enabled
		static bool dataEnabled;

		//Data transmission buffer
		static uint64_t dataBuffer;

		//Remaining data in data buffer
		static uint8_t dataRemaining;

		//Current data type
		static uint8_t dataType;

		//How often unique data bit is sent
		static uint8_t dataInterval;

		static boolean dataFinished;

		//GPS latitude and longitude data
		static bool locValid;
		static float lat;
		static float lng;
		static uint32_t lngBin;
		static uint32_t latBin;

		//Date
		static bool timeValid;

		static uint32_t adjustedMicros();

		static void setGPSInfo();

		static void calibrateWave();

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
		static void enableData(uint8_t dataPin, uint16_t dataInterval);
		static void disableData();
		static void sendDataBit();
		
		static uint32_t totalCycles();
		static uint32_t totalCycles(uint32_t timestamp);
		static uint32_t totalCycles(uint32_t timestamp, uint16_t ovf);
		static void calibrateSecond();
		static void calibrateSecond(uint32_t microsPerSecond);
		static void nextWaveInterrupt();

		static bool getWaveEnabled();
		static bool getDataEnabled();
		static void incrementOvf();
		
		static uint32_t microsecond();

		static uint32_t getCyclesPerSecond();
		static uint16_t getPulseLength();
		static uint16_t getPulseLengthError();

		static bool isPPSActive();
		static bool isTimeValid();
};

#endif
