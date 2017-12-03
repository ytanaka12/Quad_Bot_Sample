/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Serial_CSV_Format.cpp
 * Author: yasufumi
 * 
 * Created on 2017/07/26, 21:45
 */

#include "Serial_CSV_Format.h"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <string.h>
#include <string>
#include <termios.h>  


using namespace std;

namespace nsSerial_CSV_Format{

	Serial_CSV_Format::Serial_CSV_Format() {
		OpenDeviceFile();
		Serial_init();
	}

	Serial_CSV_Format::Serial_CSV_Format(const Serial_CSV_Format& orig) {
	}

	Serial_CSV_Format::~Serial_CSV_Format() {
		CloseDeviceFile();
	}

	/*------------------*/
	/* Open Device File */
	/*------------------*/
	void Serial_CSV_Format::OpenDeviceFile(){
		/* Open Device File */
		FileDesc = open(DEVICE_FILE_NAME, O_RDWR | O_NDELAY);
		if(FileDesc < 0){
			cout << "Can not open " << DEVICE_FILE_NAME << endl;
		}else{
			cout << "Opend: " << DEVICE_FILE_NAME << endl;
		}
	}
	
	/*-------------------*/
	/* Close Device File */
	/*-------------------*/
	void Serial_CSV_Format::CloseDeviceFile(){
		close(FileDesc);
		cout << "Closed: " << DEVICE_FILE_NAME << endl;
	}

	/*-----------------*/
	/* Serial Settings */
	/*-----------------*/
	//https://linuxjm.osdn.jp/html/LDP_man-pages/man3/termios.3.html
	void Serial_CSV_Format::Serial_init(){  
		struct termios tio;  
		memset(&tio, 0, sizeof(tio));  
		tio.c_cflag = CS8 | CLOCAL | CREAD;
		tio.c_cc[VTIME] = 0;
		tio.c_lflag = ICANON;
		tio.c_iflag = IGNPAR | ICRNL;

		cfsetispeed(&tio, BAUD_RATE);
		cfsetospeed(&tio, BAUD_RATE);
		tcsetattr(FileDesc, TCSANOW, &tio);
		cout << "initialized" << endl;
	}

	/*---------------------------*/
	/* Read Serial as CSV Format */
	/*---------------------------*/
	void Serial_CSV_Format::Update(){
		if(FileDesc < 0){
			//cout << "Device file is not available. (ReadSerial as CSV Format)" << endl;
			return;
		}
		
		Value.clear();
		
		/* Request to Arduino */
//		char request_msg[10] = "acc";
//		write(FileDesc, request_msg, 3);
//		char request_msg[10] = "jyro";
//		write(FileDesc, request_msg, 4);
		char request_msg[10] = "all";
		write(FileDesc, request_msg, 3);
		
		usleep(WAIT_TIME_FOR_RECEIVE);

		/* Read Info */
		char buf[1];
		string receive_msg;
		while(true){
			int length = read(FileDesc, buf, 1);

			if(length < 0 || buf[0] == '\n'){
				break;
			}
			
			if(buf[0] == ','){
				double val = stod(receive_msg);
				Value.push_back(val);
				receive_msg.clear();
			}else{
				receive_msg += buf[0];
			}
		}

		//cout << Value.size() << endl;
	}

	/*------------*/
	/* Get Values */
	/*------------*/
	std::vector<double> Serial_CSV_Format::GetValues(){
		return Value;
	}
		
}