/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Interpolations.cpp
 * Author: yasufumi
 * 
 * Created on 2017/11/02, 20:33
 */

#include "Interpolations.h"
#include <math.h>
#include "TimeKeeper.h"

Interpolations::Interpolations() {
	Duration = 0.001;
}

Interpolations::Interpolations(const Interpolations& orig) {
}

Interpolations::~Interpolations() {
}


void Interpolations::CalcCoefs(){
	A = 2.0 * (Xstart - Xgoal) / pow(Duration, 3.0);
	B = - 3.0 / 2.0 * Duration * A;
	C = 0.0;
	D = Xstart;
}

/*----------------*/
/* Set Conditions */
/*----------------*/
void Interpolations::Set_Conditions(double duration, double x_start, double x_goal){
	Duration = duration;
	if(Duration < 0.001){
		Duration = 0.001;
	}
	Xstart = x_start;
	Xgoal = x_goal;
	
	CalcCoefs();
	ResetTimer();
}

/*-------------*/
/* Reset Timer */
/*-------------*/
void Interpolations::ResetTimer(){
	Time = 0.0;
}

/*--------*/
/* Update */
/*--------*/
void Interpolations::Update(){
	if(Duration < Time){
		Time = Duration;
	}
	
	X = A * pow(Time, 3.0) + B * pow(Time, 2.0) + C * Time + D;
	dX = 3.0 * A * pow(Time, 2.0) + 2.0 * B * Time + C;
	
	Time += nsTimeKeeper::SAMPLING_TIME;
}

/*------------------------*/
/* Get Interpolated value */
/*------------------------*/
double Interpolations::Get_Interpolated_X(){
	return X;
}

/*------------------------*/
/* Get Interpolated value */
/*------------------------*/
double Interpolations::Get_Interpolated_dX(){
	return dX;
}

