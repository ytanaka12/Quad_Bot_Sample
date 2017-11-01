/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ServoBlaster.cpp
 * Author: yasufumi
 * 
 * Created on 2017/05/30, 17:01
 */

#include "ServoBlaster.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>	//open, write, read, close
#include <string.h>
#include <sstream>
#include <vector>

using namespace std;

namespace nsServoBlaster{

	ServoBlaster::ServoBlaster() {
		MinPalseWidth_us = 500;		//default
		MaxPalseWidth_us = 3000;	//default
		MinPalseWidth = MinPalseWidth_us / 10;	//default
		MaxPalseWidth = MaxPalseWidth_us / 10;	//default
		OpenDeviceFile();
	}

	ServoBlaster::ServoBlaster(const ServoBlaster& orig) {
	}

	ServoBlaster::~ServoBlaster() {
		CloseDeviceFile();
	}

	/*------------------------------*/
	/* Set Minimum palse width [us] */
	/*------------------------------*/
	void ServoBlaster::Set_MinPalseWidth_us(unsigned int min_palse_width_us){
		MinPalseWidth_us = min_palse_width_us;
		MinPalseWidth = min_palse_width_us / 10;
	}
	
	/*------------------------------*/
	/* Set Maximum palse width [us] */
	/*------------------------------*/
	void ServoBlaster::Set_MaxPalseWidth_us(unsigned int max_palse_width_us){
		MaxPalseWidth_us = max_palse_width_us;
		MaxPalseWidth = max_palse_width_us / 10;
	}
	
	/*------------*/
	/* Set p1pins */
	/*------------*/
	void ServoBlaster::Set_P1Pins(std::vector<unsigned int> p1pins){
		P1Pins.clear();
		P1Pins = p1pins;
	}
	
	/*-----------------------*/
	/* ServoBalster Settings */
	/*-----------------------*/
	void ServoBlaster::ServoBlasterSettings(){
		
		system("sh ServoBlaster_RunKill/runSB.sh");	//run this script to setup
		usleep(500000);
		cout << "run servod" << endl;
	}
	
	/*------------------*/
	/* Open Device File */
	/*------------------*/
	void ServoBlaster::OpenDeviceFile(){
		/* setting and run */
		ServoBlasterSettings();
		
		/* Open Device File */
		FileDesc = open(DEV_NAME_OF_SERVO_BLASTER, O_RDWR);
		if(FileDesc == -1){
			cout << "Can not open " << DEV_NAME_OF_SERVO_BLASTER << endl;
		}else{
			cout << "Opend: " << DEV_NAME_OF_SERVO_BLASTER << endl;
		}
	}

	/*-----------*/
	/* Terminate */
	/*-----------*/
	void ServoBlaster::CloseDeviceFile(){
		close(FileDesc);
		cout << "Closed: " << DEV_NAME_OF_SERVO_BLASTER << endl;
		system("sh ServoBlaster_RunKill/killSB.sh");
	}
	
	/*-------*/
	/* Clamp */
	/*-------*/
	double Clamp(double value, double min, double max){
		if(value < min){
			value = min;
		}else if (max < value){
			value = max;
		}
		return value;
	}

	/*------*/
	/* Send */
	/*------*/
	void ServoBlaster::SendPulseWidth(int servo_no, int pulse_width){
		/* Clamp */
		pulse_width = Clamp(pulse_width, MinPalseWidth * 0.1, MaxPalseWidth * 0.1);

		/* send */
		char buf[10];
		sprintf(buf, "%d=%d\n", servo_no, pulse_width);
		int length = strlen(buf);
		write(FileDesc, buf, length);
	}
	
	/*------*/
	/* Send */
	/*------*/
	void ServoBlaster::SendPulseWidth_us(int servo_no, int pulse_width){
		/* Clamp */
		pulse_width = Clamp(pulse_width, MinPalseWidth_us, MaxPalseWidth_us);

		/* send */
		char buf[10];
		sprintf(buf, "%d=%dus\n", servo_no, pulse_width);
		int length = strlen(buf);
		write(FileDesc, buf, length);
	}

	/*------*/
	/* Send */
	/*------*/
	void ServoBlaster::SendPercentage(int servo_no, double percentage){
		/* clamp */
		percentage = Clamp(percentage, 0.0, 100.0);

		/* send */
		char buf[10];
		sprintf(buf, "%d=%d%%\n", servo_no, (int)percentage);
		cout << buf << endl;
		int length = strlen(buf);
		write(FileDesc, buf, length);
	}

}




