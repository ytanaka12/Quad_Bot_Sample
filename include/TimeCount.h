/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TimeCount.h
 * Author: yasufumi
 *
 * Created on 2017/05/31, 16:43
 */

#ifndef TIMECOUNT_H
#define TIMECOUNT_H

#include <iostream>
#include <sys/time.h>

namespace nsTimeCount{

	class TimeCount {
	private:
		struct timeval StartTimeval;
		struct timeval CurTimeval;
		struct timeval BefTimeval;
		double ProcessingTime;
		double ElapsedTime;
		
	public:
		TimeCount();
		TimeCount(const TimeCount& orig);
		virtual ~TimeCount();

		void Set_timevals();

		void CalcProcessingTime();
		void CalcElapsedTime();

		double GetProcessingTime();	//sec
		double GetElapsedTime();

	};

}

#endif /* TIMECOUNT_H */

