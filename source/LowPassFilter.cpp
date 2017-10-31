/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LowPassFilter.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/16, 18:32
 */

#include "LowPassFilter.h"

#include <math.h>

//Reference
//http://vstcpp.wpblog.jp/?page_id=523

LowPassFilter::LowPassFilter(){
	SampleRate = 100.0;
	Freq = 10.0;
	Q = 0.7;
	
	Calc_Coef();
	
	InitTrig = false;
}

LowPassFilter::LowPassFilter(const LowPassFilter& orig) {
}

LowPassFilter::~LowPassFilter() {
}

/*-----------------*/
/* Set Sample Rate */
/*-----------------*/
void LowPassFilter::Set_SampleRate(double sample_rate) {
	SampleRate = sample_rate;
}

/*----------*/
/* Set Freq */
/*----------*/
void LowPassFilter::Set_Freq(double freq) {
	Freq = freq;
}

/*-------*/
/* Set Q */
/*-------*/
void LowPassFilter::Set_Q(double q) {
	Q = q;
}

/*------------------------*/
/* Calculate Coefficients */
/*------------------------*/
void LowPassFilter::Calc_Coef() {
	Omega = 2.0 * 3.14159265 * Freq / SampleRate;
	Alpha = sin(Omega) / (2.0 * Q);

	A0 =  1.0 + Alpha;
	A1 = -2.0 * cos(Omega);
	A2 =  1.0 - Alpha;
	B0 = (1.0 - cos(Omega)) / 2.0;
	B1 =  1.0 - cos(Omega);
	B2 = (1.0 - cos(Omega)) / 2.0;
}

/*-----------*/
/* Set Input */
/*-----------*/
void LowPassFilter::Set_Input(double input) {
	if(InitTrig == false){
		InitTrig = true;
		In2 = input;
		In1 = input;
		Out2 = input;
		Out1 = input;
	}
	Input = input;
}

/*--------*/
/* Update */
/*--------*/
void LowPassFilter::Update() {
		Output = B0/A0 * Input + B1/A0 * In1  + B2/A0 * In2
									 - A1/A0 * Out1 - A2/A0 * Out2;

		In2  = In1;
		In1  = Input;

		Out2 = Out1;
		Out1 = Output;
}

/*------------*/
/* Get Output */
/*------------*/
double LowPassFilter::Get_Output() {
	return Output;
}


