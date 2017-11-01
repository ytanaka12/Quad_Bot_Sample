/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ServoAdjuster.cpp
 * Author: yasufumi
 * 
 * Created on 2017/06/28, 19:00
 */

#include "ServoAdjuster.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
//#include <sstream>

using namespace std;

namespace nsServoAdjuster{

	ServoAdjuster::ServoAdjuster() {
		FileName = "setting/ServoAdjustParameter.csv";
	}

	ServoAdjuster::ServoAdjuster(const ServoAdjuster& orig) {
	}

	ServoAdjuster::~ServoAdjuster() {
	}
	
	/* Set File Name */
	void ServoAdjuster::SetFileName(std::string file_name){
		FileName = file_name;
	}
	
	/*---------------------*/
	/* Read Parameter File */
	/*---------------------*/
	void ServoAdjuster::ReadParameterFile(){
		ifstream ifs(FileName);
		if(!ifs){
			cout << "cannot open Parameter file: " << FileName << endl;
			exit(EXIT_FAILURE);
		}
		
		string buf;
		while(getline(ifs, buf)){
			istringstream iss(buf);
			
			string str_p1pin;
			string str_zero_pw;
			string str_adjust_gain;
			
			unsigned int p1pin;
			unsigned int zero_pw;
			double adjust_gain;
			
			getline(iss, str_p1pin, ',');
			getline(iss, str_zero_pw, ',');
			getline(iss, str_adjust_gain, ',');
			
			p1pin = stoi(str_p1pin);
			zero_pw = stoi(str_zero_pw);
			adjust_gain = stod(str_adjust_gain);
			
			P1Pins.push_back(p1pin);
			ZeroPWs.push_back(zero_pw);
			AdjustGains.push_back(adjust_gain);
		}
	}
	
	std::vector<unsigned int> ServoAdjuster::GetP1Pins(){
		return P1Pins;
	}

	std::vector<int> ServoAdjuster::GetZeroPWs(){
		return ZeroPWs;
	}
	
	unsigned int ServoAdjuster::Angle2PW_us(int pin_number, double angle){
		unsigned int pw;
		
		pw = (int)(angle * AdjustGains[pin_number])
			+ ZeroPWs[pin_number];
		
		return pw;
	}

}

