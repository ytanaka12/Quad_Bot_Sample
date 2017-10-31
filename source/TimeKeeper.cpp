/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TimeKeeper.cpp
 * Author: yasufumi
 * 
 * Created on 2017/05/31, 18:51
 */

#include "TimeKeeper.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

namespace nsTimeKeeper{

	TimeKeeper::TimeKeeper() {
	}

	TimeKeeper::TimeKeeper(const TimeKeeper& orig) {
	}

	TimeKeeper::~TimeKeeper() {
	}

	/*--------------------*/
	/* Time Keep Function */
	/*--------------------*/
	void TimeKeeper::TimeKeep() {
		Set_timevals();
		CalcProcessingTime();
		CalcElapsedTime();
		double proc_time = GetProcessingTime();
		
		int wait_time_usec = (int)( (SAMPLING_TIME - proc_time) * 1000000.0);
		if(wait_time_usec < 1000){
			wait_time_usec = 1000;
		}
		//std::cout << proc_time << std::endl;
		//std::cout << wait_time_usec << std::endl;
		usleep(wait_time_usec);
		
		Set_timevals();
	}


}