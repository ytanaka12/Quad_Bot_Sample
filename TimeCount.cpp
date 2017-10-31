/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TimeCount.cpp
 * Author: yasufumi
 * 
 * Created on 2017/05/31, 16:43
 */

#include "TimeCount.h"
#include <iostream>

using namespace std;

namespace nsTimeCount{

	TimeCount::TimeCount() {
		ProcessingTime = 0.0;
		ElapsedTime = 0.0;

		gettimeofday(&CurTimeval, NULL);
		StartTimeval = CurTimeval;
		BefTimeval = CurTimeval;
	}

	TimeCount::TimeCount(const TimeCount& orig) {
	}

	TimeCount::~TimeCount() {
	}

	/*-----*/
	/* Set */
	/*-----*/
	void TimeCount::Set_timevals(void){
		BefTimeval = CurTimeval;
		gettimeofday(&CurTimeval, NULL);
	}

	/*------*/
	/* Calc */
	/*------*/
	void TimeCount::CalcProcessingTime(){
		double bef = (double)BefTimeval.tv_sec 
					+ (double)BefTimeval.tv_usec * 0.000001;
		double cur = (double)CurTimeval.tv_sec 
					+ (double)CurTimeval.tv_usec * 0.000001;
		ProcessingTime = cur - bef;
	}
	
	/*------*/
	/* Calc */
	/*------*/
	void TimeCount::CalcElapsedTime(){
		double start = (double)StartTimeval.tv_sec 
						+ (double)StartTimeval.tv_usec * 0.000001;
		double cur = (double)CurTimeval.tv_sec 
					+ (double)CurTimeval.tv_usec * 0.000001;
		ElapsedTime = cur - start;
	}

	/*-----*/
	/* Get */
	/*-----*/	
	double TimeCount::GetProcessingTime(){
		return ProcessingTime;
	}

	/*-----*/
	/* Get */
	/*-----*/	
	double TimeCount::GetElapsedTime(){
		return ElapsedTime;
	}

}




