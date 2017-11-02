/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Interpolations.h
 * Author: yasufumi
 *
 * Created on 2017/11/02, 20:33
 */

#ifndef INTERPOLATIONS_H
#define INTERPOLATIONS_H

class Interpolations {
private:
	double Duration;
	double Xstart;
	double Xgoal;
	
	double Time;
	double X;
	double dX;

	//interpolation of 3 dimentional equation
	double A;
	double B;
	double C;
	double D;
	
	void CalcCoefs();
	
public:
	Interpolations();
	Interpolations(const Interpolations& orig);
	virtual ~Interpolations();

	void Set_Conditions(double duration, double x_start, double x_goal);
	void ResetTimer();
	
	void Update();
	
	double Get_Interpolated_X();
	double Get_Interpolated_dX();
};

#endif /* INTERPOLATIONS_H */

