/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LowPassFilter.h
 * Author: yasufumi
 *
 * Created on 2017/08/16, 18:32
 */

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
private:
	double SampleRate;
	double Freq;
	double Q;
	
	double Omega;
	double Alpha;
	
	double A0, A1, A2;
	double B0, B1, B2;
	
	double Input, Output;
	double In1, In2, Out1, Out2;
	
	bool InitTrig;
	
public:
	LowPassFilter();
	LowPassFilter(const LowPassFilter& orig);
	virtual ~LowPassFilter();

	void Set_SampleRate(double sample_rate);
	void Set_Freq(double freq);
	void Set_Q(double q);
	void Set_Input(double input);
	
	void Calc_Coef();
	void Update();
	
	double Get_Output();
private:

};

#endif /* LOWPASSFILTER_H */

