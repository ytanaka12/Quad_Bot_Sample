/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Bot_Configuration.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/15, 12:42
 */

#include "Bot_Configuration.h"
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "TimeKeeper.h"
#include "Kinematics3DOF.h"

nsKinematics3DOF::Kinematics3DOF kine;

namespace nsBot_Configuration{

	Bot_Configuration::Bot_Configuration() {
	}

	Bot_Configuration::Bot_Configuration(const Bot_Configuration& orig) {
		for(int i = 0 ; i < NUM_LEG ; ++i){
			LegInfo[i] = orig.LegInfo[i];
		}
	}

	Bot_Configuration::~Bot_Configuration() {
	}

	/*-------------------------*/
	/* Set Global Leg Position */
	/*-------------------------*/
	void Bot_Configuration::Set_LegPos(int leg_number, XYZ pos){
		LegInfo[leg_number].LegPos = pos;
		LegInfo[leg_number].LegVel.X = 0.0;
		LegInfo[leg_number].LegVel.Y = 0.0;
		LegInfo[leg_number].LegVel.Z = 0.0;
	}

	/*------------------*/
	/* Set Leg Velocity */
	/*------------------*/
	void Bot_Configuration::Set_LegVel(int leg_number, XYZ vel){
		LegInfo[leg_number].LegVel = vel;
	}

	/*--------------------------*/
	/* Set Leg Velocity for All */
	/*--------------------------*/
	void Bot_Configuration::Set_LegVel_All(XYZ vel){
		for(int i = 0 ; i < NUM_LEG ; i++){
			LegInfo[i].LegVel = vel;
		}
	}
	
	/*--------------------------*/
	/* Set Leg Velocity for All */
	/*--------------------------*/
	void Bot_Configuration::Set_LegVel_AllZero(){
		for(int i = 0 ; i < NUM_LEG ; i++){
			LegInfo[i].LegVel.X = 0.0;
			LegInfo[i].LegVel.Y = 0.0;
			LegInfo[i].LegVel.Z = 0.0;
		}
	}

	void Bot_Configuration::Set_BodyVel(XYZ vel) {
		for(int i = 0 ; i < NUM_LEG ; i++){
			LegInfo[i].LegVel.X = - vel.X;
			LegInfo[i].LegVel.Y = - vel.Y;
			LegInfo[i].LegVel.Z = - vel.Z;

		}
	}
	
	void Bot_Configuration::Set_BodyAngVel(XYZ vel_euler) {
		Eigen::Vector3d omega(vel_euler.X, vel_euler.Y, vel_euler.Z);
		for(int i = 0 ; i < NUM_LEG ; i++){
			Eigen::Vector3d r(LegInfo[i].LegPos.X, LegInfo[i].LegPos.Y, LegInfo[i].LegPos.Z);
			Eigen::Vector3d cross = omega.cross(r);
			LegInfo[i].LegVel.X = cross(0);
			LegInfo[i].LegVel.Y = cross(1);
			LegInfo[i].LegVel.Z = cross(2);
		}
	}


	/*-----------------------------------*/
	/* Global Position to Local Position */
	/*-----------------------------------*/
	void Bot_Configuration::Global2Local(int leg_number){
		XYZ buf;
		double offset_length_x;
		double offset_length_y;
		double offset_angle = 0.0;

		/* Transform to origin */
		if(leg_number == 0){
			offset_length_x = - OFFSET_LENGTH_X;
			offset_length_y = - OFFSET_LENGTH_Y;
			offset_angle = -45.0;
			
		}else if(leg_number == 1){
			offset_length_x = - OFFSET_LENGTH_X;
			offset_length_y = + OFFSET_LENGTH_Y;
			offset_angle = 45.0;
			
		}else if(leg_number == 2){
			offset_length_x = + OFFSET_LENGTH_X;
			offset_length_y = + OFFSET_LENGTH_Y;
			offset_angle = 135.0;
			
		}else if(leg_number == 3){
			offset_length_x = + OFFSET_LENGTH_X;
			offset_length_y = - OFFSET_LENGTH_Y;
			offset_angle = -135.0;
		}
		buf.X = LegInfo[leg_number].LegPos.X + offset_length_x;
		buf.Y = LegInfo[leg_number].LegPos.Y + offset_length_y;
		buf.Z = LegInfo[leg_number].LegPos.Z;
		offset_angle = offset_angle * nsKinematics3DOF::DEG2RAD;

		/* Rotation */
		Eigen::Vector3d pos(buf.X, buf.Y, buf.Z);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(offset_angle, Eigen::Vector3d(0,0,1));
		pos = rot * pos;
		
		LegInfo[leg_number].LegPos_Local.X = pos[0];
		LegInfo[leg_number].LegPos_Local.Y = pos[1];
		LegInfo[leg_number].LegPos_Local.Z = pos[2];
		
//		cout << "no: " << leg_number <<
//			" / pos: " << pos[0] <<
//			" / " << pos[1] << 
//			" / " << pos[2] << endl;
	}
	
	/*---------------------*/
	/* Local to Leg Angles */
	/*---------------------*/
	void Bot_Configuration::Local2LegAngles(int leg_number){
		XYZ buf = LegInfo[leg_number].LegPos_Local;
		
		kine.SetPosition(buf.X, buf.Y, buf.Z);
		kine.CalcInverseKinematics();
		
		for(int i = 0 ; i < NUM_JOINT_PER_LEG ; i++){
			LegInfo[leg_number].Angle[i] = kine.GetAngle(i);
		}
//		cout << "no: " << leg_number <<
//			" / ang: " << LegAngle[leg_number].Angle[0] <<
//			" / " << LegAngle[leg_number].Angle[1] << 
//			" / " << LegAngle[leg_number].Angle[2] << endl;
	}
	
	/*-----------------*/
	/* Update Paramter */
	/*-----------------*/
	void Bot_Configuration::UpdatePrameter() {
		for(int i = 0 ; i < NUM_LEG ; i++){
			LegInfo[i].LegPos.X += LegInfo[i].LegVel.X * nsTimeKeeper::SAMPLING_TIME;
			LegInfo[i].LegPos.Y += LegInfo[i].LegVel.Y * nsTimeKeeper::SAMPLING_TIME;
			LegInfo[i].LegPos.Z += LegInfo[i].LegVel.Z * nsTimeKeeper::SAMPLING_TIME;
			Global2Local(i);
			Local2LegAngles(i);
		}
	}

	
	/*-----------*/
	/* Get Angle */
	/*-----------*/
	double Bot_Configuration::Get_LegAngle(int leg_number, int joint_number){
		return LegInfo[leg_number].Angle[joint_number];
	}

	/*------------------*/
	/* Get Leg Position */
	/*------------------*/
	Bot_Configuration::XYZ Bot_Configuration::Get_LegPos(int leg_number){
		return LegInfo[leg_number].LegPos;
	}
	
	/*------------------*/
	/* Get Leg Velocity */
	/*------------------*/
	Bot_Configuration::XYZ Bot_Configuration::Get_LegVel(int leg_number){
		return LegInfo[leg_number].LegVel;
	}

	/*---------------------*/	
	/* Get Leg Information */
	/*---------------------*/
	Bot_Configuration::LegInfoStruct Bot_Configuration::Get_LegInfo(int leg_number){
		return LegInfo[leg_number];
	}
}
	