/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KalmanFilter.h
 * Author: yasufumi
 *
 * Created on 2017/08/17, 15:08
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
private:
	double X;	//estimated
	double Y;	//measurement
	double U;	//input(velocity)
	double P;	//
	
	double Sigma_v2;
	double Sigma_w2;
	
	bool InitTrig;
	
public:
	KalmanFilter();
	KalmanFilter(const KalmanFilter& orig);
	virtual ~KalmanFilter();
	
	void Set_Y(double y);
	void Set_U(double u);
	void Set_Sigma_v2(double sigma_v2);
	void Set_Sigma_w2(double sigma_w2);
	
	void Filtering();
	
	double Get_X();
	double Get_P();

};

#endif /* KALMANFILTER_H */

