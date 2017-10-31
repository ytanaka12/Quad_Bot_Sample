/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Serial_CSV_Format.h
 * Author: yasufumi
 *
 * Created on 2017/07/26, 21:45
 */

#ifndef SERIAL_CSV_FORMAT_H
#define SERIAL_CSV_FORMAT_H

#include <vector>

namespace nsSerial_CSV_Format{
	
	//const char DEVICE_FILE_NAME[30] = "/dev/ttyACM0";
	const char DEVICE_FILE_NAME[30] = "/dev/ttyUSB0";
	const int WAIT_TIME_FOR_RECEIVE = 8000;	//[usec]//acc: 3000 ///jyro:5000
	#define BAUD_RATE B115200

	class Serial_CSV_Format {
	private:
		int FileDesc;
		
		std::vector<double> Value;
		
	public:
		Serial_CSV_Format();
		Serial_CSV_Format(const Serial_CSV_Format& orig);
		virtual ~Serial_CSV_Format();
		
		void OpenDeviceFile();
		void CloseDeviceFile();
		
		void Serial_init();
		
		void Update();
		
		std::vector<double> GetValues();

	};

}

#endif /* SERIAL_CSV_FORMAT_H */

