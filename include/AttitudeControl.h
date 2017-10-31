/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   AttitudeControl.h
 * Author: yasufumi
 *
 * Created on 2017/08/16, 14:11
 */

#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include "Bot_Configuration.h"
#include "KalmanFilter.h"

namespace nsAttitudeControl{
	class AttitudeControl {
		
	private:
		nsBot_Configuration::Bot_Configuration BotConf;
		nsBot_Configuration::Bot_Configuration BotConf_Modified;
		nsBot_Configuration::Bot_Configuration::XYZ Acc;
		nsBot_Configuration::Bot_Configuration::XYZ AngVel;
		nsBot_Configuration::Bot_Configuration::XYZ Rot;
		
		KalmanFilter KF_RotX;
		KalmanFilter KF_RotY;
		
		nsBot_Configuration::Bot_Configuration::XYZ Gain_P;
		nsBot_Configuration::Bot_Configuration::XYZ Gain_D;
		nsBot_Configuration::Bot_Configuration::XYZ Gain_I;
		nsBot_Configuration::Bot_Configuration::XYZ Rot_forCont;
		
		void Calc_Rot();
		void PD_Control();
		void I_Control();
		void PI_Control();
		
	public:
		AttitudeControl();
		AttitudeControl(const AttitudeControl& orig);
		virtual ~AttitudeControl();
		
		void Set_BotConf(nsBot_Configuration::Bot_Configuration bot_conf);
		void Set_Acc(nsBot_Configuration::Bot_Configuration::XYZ acc);
		void Set_AngVel(nsBot_Configuration::Bot_Configuration::XYZ ang_vel);
		
		void Calculation();
		
		double Get_Additional_LegAngle(int leg_number, int joint_number);
		nsBot_Configuration::Bot_Configuration 
		Get_Modified_BotConf();

	};
}

#endif /* ATTITUDECONTROL_H */

