/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ServoBlaster.h
 * Author: yasufumi
 *
 * Created on 2017/05/30, 17:01
 */

#ifndef SERVOBLASTER_H
#define SERVOBLASTER_H

#include <vector>


namespace nsServoBlaster{
	
	const char DEV_NAME_OF_SERVO_BLASTER[30] = "/dev/servoblaster";

	class ServoBlaster {
	private:
		//char DEV_NAME_OF_SERVO_BLASTER[30];
		int FileDesc;
		int NumberOfServo;
		
		unsigned int MinPalseWidth_us;	//500us
		unsigned int MaxPalseWidth_us;	//2500us
		unsigned int MinPalseWidth;
		unsigned int MaxPalseWidth;
		std::vector<unsigned int> P1Pins;

	public:
		ServoBlaster();
		ServoBlaster(const ServoBlaster& orig);
		virtual ~ServoBlaster();
		
		void Set_MinPalseWidth_us(unsigned int min_palse_width_us);
		void Set_MaxPalseWidth_us(unsigned int max_palse_width_us);
		void Set_P1Pins(std::vector<unsigned int> p1pins);
		void ServoBlasterSettings();

		void OpenDeviceFile();
		void CloseDeviceFile(void);

		void SendPulseWidth(int servo_no, int pulse_width);
		void SendPulseWidth_us(int servo_no, int pulse_width);
		void SendPercentage(int servo_no, double percentage);
	};
}

#endif /* SERVOBLASTER_H */


