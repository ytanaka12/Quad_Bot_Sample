/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/17, 15:08
 */

#include "KalmanFilter.h"

#include "TimeKeeper.h"

KalmanFilter::KalmanFilter() {
	X = 0.0;
	Y = 0.0;
	U = 0.0;
	P = 0.0;
	Sigma_v2 = 1.0;
	Sigma_w2 = 2.0;
	InitTrig = true;
}

KalmanFilter::KalmanFilter(const KalmanFilter& orig) {
}

KalmanFilter::~KalmanFilter() {
}

/*-------*/
/* Set Y */
/*-------*/
void KalmanFilter::Set_Y(double y) {
	Y = y;
	
	if(InitTrig == true){
		InitTrig = false;
		X = y;
	}
}

/*-------*/
/* Set U */
/*-------*/
void KalmanFilter::Set_U(double u) {
	U = u;
}

/*--------------*/
/* Set Sigma v2 */
/*--------------*/
void KalmanFilter::Set_Sigma_v2(double sigma_v2) {
	Sigma_v2 = sigma_v2;
}

/*--------------*/
/* Set Sigma w2 */
/*--------------*/
void KalmanFilter::Set_Sigma_w2(double sigma_w2) {
	Sigma_w2 = sigma_w2;
}

/*------------------*/
/* Kalman Filtering */
/*------------------*/
void KalmanFilter::Filtering() {
	X = X + U * nsTimeKeeper::SAMPLING_TIME;
	P = P + Sigma_v2;
	double G = P / (P + Sigma_w2);
	X = X + G * (Y - X);
	P = (1 - G) * P;
}

/*-------*/
/* Get X */
/*-------*/
double KalmanFilter::Get_X() {
	return X;
}

/*-------*/
/* Get P */
/*-------*/
double KalmanFilter::Get_P() {
	return P;
}

