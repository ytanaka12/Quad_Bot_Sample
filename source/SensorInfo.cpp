/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SensorInfo.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/14, 20:20
 */

#include "SensorInfo.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "TimeKeeper.h"

const double DEG2RAD = 3.1415 / 180.0;
const double RAD2DEG = 180.0 / 3.1415;

using namespace std;

static string SENSOR_GAIN_ACC_CSV = "setting/SensorGain_Acc.csv";
static string SENSOR_GAIN_ANGVEL_CSV = "setting/SensorGain_AngVel.csv";

namespace nsSensorInfo{

	SensorInfo::SensorInfo() {
		SamplingTime = nsTimeKeeper::SAMPLING_TIME;
		
		ReadGainParameter_Acc();
		ReadGainParameter_AngVel();
		
		printf("Gain Acc: %.3f / %.3f, %.3f\n", GainAcc.X, GainAcc.Y, GainAcc.Z);
		printf("Bias Acc: %.3f / %.3f, %.3f\n", BiasAcc.X, BiasAcc.Y, BiasAcc.Z);
		printf("Gain Ang Vel: %.3f / %.3f, %.3f\n", GainAngVel.X, GainAngVel.Y, GainAngVel.Z);
		printf("Bias Ang Vel: %.3f / %.3f, %.3f\n", BiasAngVel.X, BiasAngVel.Y, BiasAngVel.Z);
		
		Acc.X = 0.0;
		Acc.Y = 0.0;
		Acc.Z = -1.0;
		
		AngVel.X = 0.0;
		AngVel.Y = 0.0;
		AngVel.Z = 0.0;
		
		double sample_rate = 1000.0;
		double freq = 10.0;
		double q = 0.7;
		LPF_AccX.Set_SampleRate(sample_rate);	LPF_AccX.Set_Freq(freq);	LPF_AccX.Set_Q(q);
		LPF_AccY.Set_SampleRate(sample_rate);	LPF_AccY.Set_Freq(freq);	LPF_AccY.Set_Q(q);
		LPF_AccZ.Set_SampleRate(sample_rate);	LPF_AccZ.Set_Freq(freq);	LPF_AccZ.Set_Q(q);
		LPF_AccX.Calc_Coef();
		LPF_AccY.Calc_Coef();
		LPF_AccZ.Calc_Coef();

		double sample_rate_a = 1000.0;
		double freq_a = 10.0;
		double q_a = 0.7;		
		LPF_AngVelX.Set_SampleRate(sample_rate_a);	LPF_AngVelX.Set_Freq(freq_a);	LPF_AngVelX.Set_Q(q_a);
		LPF_AngVelY.Set_SampleRate(sample_rate_a);	LPF_AngVelY.Set_Freq(freq_a);	LPF_AngVelY.Set_Q(q_a);
		LPF_AngVelZ.Set_SampleRate(sample_rate_a);	LPF_AngVelZ.Set_Freq(freq_a);	LPF_AngVelZ.Set_Q(q_a);
		LPF_AngVelX.Calc_Coef();
		LPF_AngVelY.Calc_Coef();
		LPF_AngVelZ.Calc_Coef();
	}

	SensorInfo::SensorInfo(const SensorInfo& orig) {
	}

	SensorInfo::~SensorInfo() {
	}

	/*------------------------*/
	/* Read Gain Paramter Acc */
	/*------------------------*/
	void SensorInfo::ReadGainParameter_Acc(){
		ifstream ifs(SENSOR_GAIN_ACC_CSV);
		if(!ifs){
			cout << "cannot open Gain Parameter file: " << SENSOR_GAIN_ACC_CSV << endl;
			exit(EXIT_FAILURE);
		}
		
		string buf;
		getline(ifs, buf);
		istringstream iss(buf);

		string str_x;
		string str_y;
		string str_z;
		string str_bias_x;
		string str_bias_y;
		string str_bias_z;

		getline(iss, str_x, ',');
		getline(iss, str_y, ',');
		getline(iss, str_z, ',');
		getline(iss, str_bias_x, ',');
		getline(iss, str_bias_y, ',');
		getline(iss, str_bias_z, ',');

		try{
			GainAcc.X = stod(str_x);
			GainAcc.Y = stod(str_y);
			GainAcc.Z = stod(str_z);
			BiasAcc.X = stod(str_bias_x);
			BiasAcc.Y = stod(str_bias_y);
			BiasAcc.Z = stod(str_bias_z);
			throw 1;
		}
		catch(int num){
			cout << "err read gain parameter acc" << endl;
		}
	}
	
	/*---------------------------*/
	/* Read Gain Paramter AngVel */
	/*---------------------------*/
	void SensorInfo::ReadGainParameter_AngVel(){
		ifstream ifs(SENSOR_GAIN_ANGVEL_CSV);
		if(!ifs){
			cout << "cannot open Gain Parameter file: " << SENSOR_GAIN_ANGVEL_CSV << endl;
			exit(EXIT_FAILURE);
		}
		
		string buf;
		getline(ifs, buf);
		istringstream iss(buf);

		string str_x;
		string str_y;
		string str_z;
		string str_bias_x;
		string str_bias_y;
		string str_bias_z;

		getline(iss, str_x, ',');
		getline(iss, str_y, ',');
		getline(iss, str_z, ',');
		getline(iss, str_bias_x, ',');
		getline(iss, str_bias_y, ',');
		getline(iss, str_bias_z, ',');

		try{
			GainAngVel.X = stod(str_x);
			GainAngVel.Y = stod(str_y);
			GainAngVel.Z = stod(str_z);
			BiasAngVel.X = stod(str_bias_x);
			BiasAngVel.Y = stod(str_bias_y);
			BiasAngVel.Z = stod(str_bias_z);
			throw 1;
		}
		catch(int num){
			cout << "err read gain parameter angvel" << endl;
		}
	}

	/*--------*/
	/* Update */
	/*--------*/
	void SensorInfo::Update(){
		sensorMod.Update();
		vector<double> sensorVal = sensorMod.GetValues();
		
		if(sensorVal.size() < 6){
			return;
		}

//		for(int i = 0 ; i < sensorVal.size() ; i++){
//			printf("%8.5f\t", sensorVal[i]);
//		}
//		cout << endl;
		
		Acc.X = (sensorVal[0] + BiasAcc.X) * GainAcc.X;
		Acc.Y = (sensorVal[1] + BiasAcc.Y) * GainAcc.Y;
		Acc.Z = (sensorVal[2] + BiasAcc.Z) * GainAcc.Z;
		AngVel.X = (sensorVal[3] + BiasAngVel.X) * GainAngVel.X;
		AngVel.Y = (sensorVal[4] + BiasAngVel.Y) * GainAngVel.Y;
		AngVel.Z = (sensorVal[5] + BiasAngVel.Z) * GainAngVel.Z;
		
		Acc.X = Acc.X;
		Acc.Y = Acc.Y;
		Acc.Z = - Acc.Z;
		
		AngVel.X = - AngVel.X * DEG2RAD;
		AngVel.Y = - AngVel.Y * DEG2RAD;
		AngVel.Z = AngVel.Z * DEG2RAD;
		
		/* Filtering */
		LPF_AccX.Set_Input(Acc.X);
		LPF_AccY.Set_Input(Acc.Y);
		LPF_AccZ.Set_Input(Acc.Z);
		LPF_AccX.Update();
		LPF_AccY.Update();
		LPF_AccZ.Update();
		Acc.X = LPF_AccX.Get_Output();
		Acc.Y = LPF_AccY.Get_Output();
		Acc.Z = LPF_AccZ.Get_Output();
		
		LPF_AngVelX.Set_Input(AngVel.X);
		LPF_AngVelY.Set_Input(AngVel.Y);
		LPF_AngVelZ.Set_Input(AngVel.Z);
		LPF_AngVelX.Update();
		LPF_AngVelY.Update();
		LPF_AngVelZ.Update();
		AngVel.X = LPF_AngVelX.Get_Output();
		AngVel.Y = LPF_AngVelY.Get_Output();
		AngVel.Z = LPF_AngVelZ.Get_Output();
		
		
		//printf("Acc: %.3f\t%.3f\t%.3f\n", Acc.X, Acc.Y, Acc.Z);
		//printf("Ang Vel: %.3f\t%.3f\t%.3f\n", AngVel.X, AngVel.Y, AngVel.Z);

//		static double angle;
//		angle += AngVel.Y * SamplingTime;
//		printf("Ang: %.3f\n", angle);
	}
	
	/*---------------*/
	/* Get Functions */
	/*---------------*/
	double SensorInfo::Get_Acc_X(){
		return Acc.X;
	}
	
	double SensorInfo::Get_Acc_Y(){
		return Acc.Y;
	}
	
	double SensorInfo::Get_Acc_Z(){
		return Acc.Z;
	}

	double SensorInfo::Get_AngVel_X(){
		return AngVel.X;
	}
	
	double SensorInfo::Get_AngVel_Y(){
		return AngVel.Y;
	}
	
	double SensorInfo::Get_AngVel_Z(){
		return AngVel.Z;
	}
}

