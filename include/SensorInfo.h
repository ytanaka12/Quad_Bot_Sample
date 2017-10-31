/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SensorInfo.h
 * Author: yasufumi
 *
 * Created on 2017/08/14, 20:20
 */

#ifndef SENSORINFO_H
#define SENSORINFO_H

#include "Serial_CSV_Format.h"
#include "LowPassFilter.h"

namespace nsSensorInfo{
	class SensorInfo {
	private:
		nsSerial_CSV_Format::Serial_CSV_Format sensorMod;
		double SamplingTime;
		
		struct XYZ{
			double X;
			double Y;
			double Z;
		};
		
		XYZ GainAngVel;
		XYZ GainAcc;
		XYZ BiasAngVel;
		XYZ BiasAcc;
		
		XYZ AngVel;
		XYZ Acc;

		LowPassFilter LPF_AccX;
		LowPassFilter LPF_AccY;
		LowPassFilter LPF_AccZ;
		
		LowPassFilter LPF_AngVelX;
		LowPassFilter LPF_AngVelY;
		LowPassFilter LPF_AngVelZ;
		
	public:
		SensorInfo();
		SensorInfo(const SensorInfo& orig);
		virtual ~SensorInfo();
		
		void ReadGainParameter_Acc();
		void ReadGainParameter_AngVel();
		
		void Update();
		
		double Get_Acc_X();
		double Get_Acc_Y();
		double Get_Acc_Z();
		
		double Get_AngVel_X();
		double Get_AngVel_Y();
		double Get_AngVel_Z();

	};
}

#endif /* SENSORINFO_H */

