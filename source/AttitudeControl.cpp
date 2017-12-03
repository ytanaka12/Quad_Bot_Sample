/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   AttitudeControl.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/16, 14:11
 */

#include "AttitudeControl.h"
#include "Kinematics3DOF.h"
#include "TimeKeeper.h"

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

using namespace std;

namespace nsAttitudeControl{

	AttitudeControl::AttitudeControl() {
		KF_RotX.Set_Sigma_v2(0.05);
		KF_RotX.Set_Sigma_w2(0.1);
		
		KF_RotY.Set_Sigma_v2(0.05);
		KF_RotY.Set_Sigma_w2(0.1);
		
		Gain_P.X = 1.8;
		Gain_P.Y = 1.8;
		Gain_D.X = 0.02;
		Gain_D.Y = 0.02;
		Gain_I.X = 0.02;
		Gain_I.Y = 0.02;
	}

	AttitudeControl::AttitudeControl(const AttitudeControl& orig) {
	}

	AttitudeControl::~AttitudeControl() {
	}

	/*-----------------------*/
	/* Set Bot Configuration */
	/*-----------------------*/	
	void AttitudeControl::Set_BotConf(nsBot_Configuration::Bot_Configuration bot_conf){
		BotConf = bot_conf;
	}
	
	/*---------*/
	/* Set Acc */
	/*---------*/
	void AttitudeControl::Set_Acc(nsBot_Configuration::Bot_Configuration::XYZ acc){
		Acc = acc;
	}

	/*----------------------*/
	/* Set Angular Velocity */
	/*----------------------*/
	void AttitudeControl::Set_AngVel(nsBot_Configuration::Bot_Configuration::XYZ ang_vel) {
		AngVel = ang_vel;
	}

	/*--------------------*/
	/* Calculate Rotation */
	/*--------------------*/
	void AttitudeControl::Calc_Rot() {
		Rot.X = atan2( Acc.Y, -Acc.Z );
		Rot.Y = - atan2( Acc.X, -Acc.Z );

		/* Filtering */
		KF_RotX.Set_U(- AngVel.X);
		KF_RotX.Set_Y(Rot.X);
		KF_RotX.Filtering();
		Rot.X = KF_RotX.Get_X();
		
		KF_RotY.Set_U(- AngVel.Y);
		KF_RotY.Set_Y(Rot.Y);
		KF_RotY.Filtering();
		Rot.Y = KF_RotY.Get_X();
	}
	
	void AttitudeControl::PD_Control() {
		static nsBot_Configuration::Bot_Configuration::XYZ bef_rot;
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			bef_rot.X = 0.0;
			bef_rot.Y = 0.0;
		}
		nsBot_Configuration::Bot_Configuration::XYZ rot_vel;
		rot_vel.X = (Rot.X - bef_rot.X) / nsTimeKeeper::SAMPLING_TIME;
		rot_vel.Y = (Rot.Y - bef_rot.Y) / nsTimeKeeper::SAMPLING_TIME;
		bef_rot.X = Rot.X;
		bef_rot.Y = Rot.Y;
		
		Rot_forCont.X = Gain_P.X * Rot.X - Gain_D.X * rot_vel.X;
		Rot_forCont.Y = Gain_P.Y * Rot.Y - Gain_D.Y * rot_vel.Y;
		
//		Rot.X = Gain_P.X * Rot.X - Gain_D.X * ( - AngVel.X);
//		Rot.Y = Gain_P.Y * Rot.Y - Gain_D.Y * ( - AngVel.Y);
	}

	void AttitudeControl::I_Control() {
		static nsBot_Configuration::Bot_Configuration::XYZ inte;
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			inte.X = 0.0;
			inte.Y = 0.0;
		}
		inte.X += Rot.X;
		inte.Y += Rot.Y;
		
		Rot_forCont.X = Gain_I.X * inte.X;
		Rot_forCont.Y = Gain_I.Y * inte.Y;
	}
	
	void AttitudeControl::PI_Control() {
		static nsBot_Configuration::Bot_Configuration::XYZ inte;
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			inte.X = 0.0;
			inte.Y = 0.0;
		}
		inte.X += Rot.X;
		inte.Y += Rot.Y;
		
		Rot_forCont.X = Gain_P.X * Rot.X + Gain_I.X * inte.X;
		Rot_forCont.Y = Gain_P.Y * Rot.Y + Gain_I.Y * inte.Y;
	}

	/*-----------*/
	/* Calculate */
	/*-----------*/
	void AttitudeControl::Calculation(){
		Calc_Rot();
		//PD_Control();
		I_Control();
		//PI_Control();
		
//		cout << "Rot: " << Rot.X * nsKinematics3DOF::RAD2DEG << 
//				" / " << Rot.Y * nsKinematics3DOF::RAD2DEG << endl;
		
		//cout << "Rot: " << Rot.Y << " Ang vel: " << AngVel.Y << endl;
		
//		cout << "ang vel: "
//			 << AngVel.X << " / "
//			 << AngVel.Y << " / "
//			 << AngVel.Z << endl;
		
		double rate = 1.0;
		/* Rotation */
		for(int i = 0 ; i < 4 ; i++){
			nsBot_Configuration::Bot_Configuration::XYZ buf = BotConf.Get_LegPos(i);
			Eigen::Vector3d pos(buf.X, buf.Y, buf.Z);
			Eigen::Matrix3d rot;
			//rot = Eigen::AngleAxisd( - Rot.X * rate, Eigen::Vector3d(1,0,0));
			rot = Eigen::AngleAxisd( - Rot_forCont.X, Eigen::Vector3d(1,0,0));
			pos = rot * pos;
			//rot = Eigen::AngleAxisd( - Rot.Y * rate, Eigen::Vector3d(0,1,0));
			rot = Eigen::AngleAxisd( - Rot_forCont.Y, Eigen::Vector3d(0,1,0));
			pos = rot * pos;
			buf.X = pos[0];
			buf.Y = pos[1];
			buf.Z = pos[2];
			BotConf_Modified.Set_LegPos(i, buf);
		}
		
		BotConf_Modified.UpdatePrameter();
		
	}

	/*--------------------------*/
	/* Get Additional Leg Angle */
	/*--------------------------*/
	double AttitudeControl::Get_Additional_LegAngle(int leg_number, int joint_number){
		return BotConf_Modified.Get_LegAngle(leg_number, joint_number)
				- BotConf.Get_LegAngle(leg_number, joint_number);
	}

	/*--------------------------------*/
	/* Get Modified Bot Configuration */
	/*--------------------------------*/
	nsBot_Configuration::Bot_Configuration 
	AttitudeControl::Get_Modified_BotConf() {
		return BotConf_Modified;
	}

}


