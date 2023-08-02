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

		//Whether PPS is attached
		bool ppsFlag = false;

		//Flag when pps is active
		bool ppsActive = false;

		//Last pps value
		bool prevPPS = false;

		//Current pps value
		bool currPPS = false;


		//Arduino microsecond reference frame
		uint32_t microStart = 0;

		//Average Arduino microseconds per real second
		uint32_t microsPerSecond = 1000000;

		//Average Arduino microsecond error per arduino second
		int32_t secondError = 0;

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

		uint32_t rawMicros();
		uint32_t realMicros();
		void calibrateSecond();

		void setTime();

		void addSeconds(uint8_t secondDiff);
		void addMinutes(uint8_t minuteDiff);
		void addHours(uint8_t hourDiff);
		void addDays(uint8_t dayDiff);
		void addYears(uint16_t yearDiff);
	public:
    	GPSTimer(TinyGPSPlus* gps);

		void attachPPS(uint8_t ppsPin);

		void update();
		
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
};

#endif
