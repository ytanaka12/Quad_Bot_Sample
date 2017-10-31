/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ServoAdjuster.h
 * Author: yasufumi
 *
 * Created on 2017/06/28, 19:00
 */

#ifndef SERVOADJUSTER_H
#define SERVOADJUSTER_H

#include "ServoBlaster.h"
#include <string>

namespace nsServoAdjuster{

	class ServoAdjuster {
	private:		
		std::string FileName;
		
		std::vector<unsigned int> P1Pins;
		std::vector<int> ZeroPWs;
		std::vector<double> AdjustGains;

	public:
		ServoAdjuster();
		ServoAdjuster(const ServoAdjuster& orig);
		virtual ~ServoAdjuster();
		
		void SetFileName(std::string file_name);
		
		void ReadParameterFile();
		
		std::vector<unsigned int> GetP1Pins();
		std::vector<int> GetZeroPWs();
		unsigned int Angle2PW_us(int pin_number, double angle);
	};

}

#endif /* SERVOADJUSTER_H */

