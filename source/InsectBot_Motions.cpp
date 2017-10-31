/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InsectBot_Motions.cpp
 * Author: yasufumi
 * 
 * Created on 2017/06/29, 19:38
 */

#include "InsectBot_Motions.h"
#include <iostream>
#include <math.h>
#include "TimeKeeper.h"
#include <Eigen/Dense>

using namespace std;

namespace nsInsectBot_Motions{

	InsectBot_Motions::InsectBot_Motions() {
	}

	InsectBot_Motions::InsectBot_Motions(const InsectBot_Motions& orig) {
	}

	InsectBot_Motions::~InsectBot_Motions() {
	}
	
	/*------------*/
	/* Initialize */
	/*------------*/
	void InsectBot_Motions::Init(){
		XYZ pos;
		pos.X = 0.1;
		pos.Y = 0.0;
		pos.Z = 0.05;
		
		kine.SetPosition(pos.X, pos.Y, pos.Z);
		kine.CalcInverseKinematics();

		for(int i = 0 ; i < NUM_LEG ; i++){
			LegAngle[i].Angle[0] = kine.GetAngle(0);
			LegAngle[i].Angle[1] = kine.GetAngle(1);
			LegAngle[i].Angle[2] = kine.GetAngle(2);
			LegVel[i].X = 0.0;
			LegVel[i].Y = 0.0;
			LegVel[i].Z = 0.0;
		}
	}

	/*-----------------------------------*/
	/* Set Global Leg End point Position */
	/*-----------------------------------*/
	void InsectBot_Motions::SetGlobalLegPos(int leg_number, double x, double y, double z){
		g_LegPos[leg_number].X = x;
		g_LegPos[leg_number].Y = y;
		g_LegPos[leg_number].Z = z;
	}

	/*----------------------------------*/
	/* Set Local Leg End point Position */
	/*----------------------------------*/	
	void InsectBot_Motions::SetLocalLegPos(int leg_number, double x, double y, double z){
		l_LegPos[leg_number].X = x;
		l_LegPos[leg_number].Y = y;
		l_LegPos[leg_number].Z = z;
	}
	
	/*-----------------------------------*/
	/* Global Position to Local Position */
	/*-----------------------------------*/
	void InsectBot_Motions::Global2Local(int leg_number){
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
		buf.X = g_LegPos[leg_number].X + offset_length_x;
		buf.Y = g_LegPos[leg_number].Y + offset_length_y;
		buf.Z = g_LegPos[leg_number].Z;
		offset_angle = offset_angle * nsKinematics3DOF::DEG2RAD;

		/* Rotation */
		Eigen::Vector3d pos(buf.X, buf.Y, buf.Z);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(offset_angle, Eigen::Vector3d(0,0,1));
		pos = rot * pos;
		
		l_LegPos[leg_number].X = pos[0];
		l_LegPos[leg_number].Y = pos[1];
		l_LegPos[leg_number].Z = pos[2];
		
//		cout << "no: " << leg_number <<
//			" / pos: " << pos[0] <<
//			" / " << pos[1] << 
//			" / " << pos[2] << endl;
	}
	
	/*---------------------*/
	/* Local to Leg Angles */
	/*---------------------*/
	void InsectBot_Motions::Local2LegAngles(int leg_number){
		XYZ buf = l_LegPos[leg_number];
		
		kine.SetPosition(buf.X, buf.Y, buf.Z);
		kine.CalcInverseKinematics();
		
		for(int i = 0 ; i < NUM_JOINT_PER_LEG ; i++){
			LegAngle[leg_number].Angle[i] = kine.GetAngle(i);
		}
//		cout << "no: " << leg_number <<
//			" / ang: " << LegAngle[leg_number].Angle[0] <<
//			" / " << LegAngle[leg_number].Angle[1] << 
//			" / " << LegAngle[leg_number].Angle[2] << endl;
	}
	
	

	/*----------*/
	/*Move Shift*/
	/*----------*/
	void InsectBot_Motions::MoveShift(XYZ vel){
		for(int i = 0 ; i < 4 ; i++){
			LegVel[i] = vel;
		}
	}
	
	/*------*/
	/* Turn */
	/*------*/
	void InsectBot_Motions::Turn(double ang_vel){
		double angle = ang_vel * nsTimeKeeper::SAMPLING_TIME;
		for(int i = 0 ; i < 4 ; i++){
			/* Rotation */
			Eigen::Vector3d eigen_pos(g_LegPos[i].X, g_LegPos[i].Y, g_LegPos[i].Z);
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(angle, Eigen::Vector3d(0,0,1));
			eigen_pos = rot * eigen_pos;
//			SetGlobalLegPos(i, eigen_pos[0] + bufX, eigen_pos[1] + bufY, eigen_pos[2] + bufZ);
			g_LegPos[i].X = eigen_pos[0];
			g_LegPos[i].Y = eigen_pos[1];
			g_LegPos[i].Z = eigen_pos[2];
		}
	}
	
	/*------------------*/
	/* Set Leg Velocity */
	/*------------------*/
	void InsectBot_Motions::SetLegVelocity(int leg_number, XYZ vel){
		LegVel[leg_number] = vel;
	}

	/*------------------------*/
	/* Set All Zero velolcity */
	/*------------------------*/
	void InsectBot_Motions::SetLegVelocityAllZero(){
		for(int i = 0 ; i < 4 ; i++){
			LegVel[i].X = 0.0;
			LegVel[i].Y = 0.0;
			LegVel[i].Z = 0.0;
		}
	}
	
	/*----------------*/
	/* Point to Point */
	/*----------------*/
	void InsectBot_Motions::SetEnd2EndMotionParam(int leg_number, double duration_time, XYZ end){
		if(duration_time < 0.1){
			duration_time = 0.1;
		}
		DurationTime[leg_number] = duration_time;
		g_StartPos[leg_number] = g_LegPos[leg_number];
		g_EndPos[leg_number] = end;
		
		LegVel[leg_number].X = ( g_EndPos[leg_number].X - g_StartPos[leg_number].X ) / duration_time;
		LegVel[leg_number].Y = ( g_EndPos[leg_number].Y - g_StartPos[leg_number].Y ) / duration_time;
		LegVel[leg_number].Z = ( g_EndPos[leg_number].Z - g_StartPos[leg_number].Z ) / duration_time;
		//Timer[leg_number] = 0.0;
	}
	
	/*----------------*/
	/* Point to Point */
	/*----------------*/
	void InsectBot_Motions::SetEnd2EndMotionReset(int leg_number){
		Timer[leg_number] = 0.0;
	}
	
	/*----------------*/
	/* Point to Point */
	/*----------------*/
	void InsectBot_Motions::GenerateEnd2EndMotion(int leg_number){
		if(DurationTime[leg_number] < Timer[leg_number]){
			Timer[leg_number] = DurationTime[leg_number];
		}
		
		XYZ diff;
		diff.X = g_EndPos[leg_number].X - g_StartPos[leg_number].X;
		diff.Y = g_EndPos[leg_number].Y - g_StartPos[leg_number].Y;
		diff.Z = g_EndPos[leg_number].Z - g_StartPos[leg_number].Z;
		
		XYZ pos;
		pos.X = diff.X / DurationTime[leg_number] * Timer[leg_number] + g_StartPos[leg_number].X;
		pos.Y = diff.Y / DurationTime[leg_number] * Timer[leg_number] + g_StartPos[leg_number].Y;
		pos.Z = diff.Z / DurationTime[leg_number] * Timer[leg_number] + g_StartPos[leg_number].Z;
		
		SetGlobalLegPos(leg_number, pos.X, pos.Y, pos.Z);
		Global2Local(leg_number);
		Local2LegAngles(leg_number);
		
		Timer[leg_number] += nsTimeKeeper::SAMPLING_TIME;
	}

	/*------------------------------*/
	/* Calculate Square of triangle */
	/*------------------------------*/
	double InsectBot_Motions::CalcSquareOfTriangle(int n1, int n2, int n3){
		double a = sqrt( pow(g_LegPos[n1].X - g_LegPos[n2].X, 2.0)
						+pow(g_LegPos[n1].Y - g_LegPos[n2].Y, 2.0));
		double b = sqrt( pow(g_LegPos[n2].X - g_LegPos[n3].X, 2.0)
						+pow(g_LegPos[n2].Y - g_LegPos[n3].Y, 2.0));
		double c = sqrt( pow(g_LegPos[n3].X - g_LegPos[n1].X, 2.0)
						+pow(g_LegPos[n3].Y - g_LegPos[n1].Y, 2.0));
		double s = (a + b + c) / 2.0;
		double ans = sqrt( s * (s - a) * (s - b) - (s - c) );
		
		return ans;
	}
	
	/*-------------------------------*/
	/* Calculate Cntroid of Triangle */
	/*-------------------------------*/
	XYZ InsectBot_Motions::CalcCentroidOfTriangle(int n1, int n2, int n3){
		XYZ ans;
		
		ans.X = (g_LegPos[n1].X + g_LegPos[n2].X + g_LegPos[n3].X) / 3.0;
		ans.Y = (g_LegPos[n1].Y + g_LegPos[n2].Y + g_LegPos[n3].Y) / 3.0;
		ans.Z = (g_LegPos[n1].Z + g_LegPos[n2].Z + g_LegPos[n3].Z) / 3.0;
		
		return ans;
	}

	/*------------------------------*/
	/* Calculate Centroid of Square */
	/*------------------------------*/
	XYZ InsectBot_Motions::CalcCentroidOfSquare(){
		XYZ ans;
		ans.X = 0;
		ans.Y = 0;
		ans.Z = 0;
		for(int i = 0 ; i < 4 ; i++){
			ans.X += g_LegPos[i].X;
			ans.Y += g_LegPos[i].Y;
			ans.Z += g_LegPos[i].Z;
		}
		ans.X = ans.X / 4.0;
		ans.Y = ans.Y / 4.0;
		ans.Z = ans.Z / 4.0;
		
		return ans;
	}

	/*-------------------------*/
	/* Generate Walking Motion */
	/*-------------------------*/
	void InsectBot_Motions::GenerateMoving(){
		static XYZ pos[4];
		static XYZ end[4];
		static double time = 0.0;
		
		/*------*/
		/* init */
		/*------*/
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			pos[0].X =  0.14;	pos[0].Y =  0.14;	pos[0].Z = -0.1;
			pos[1].X =  0.14;	pos[1].Y = -0.14;	pos[1].Z = -0.1;
			pos[2].X = -0.14;	pos[2].Y = -0.14;	pos[2].Z = -0.1;
			pos[3].X = -0.14;	pos[3].Y =  0.14;	pos[3].Z = -0.1;
			for(int i = 0 ; i < 4 ; i++){
				end[i] = pos[i];
			}
			for(int i = 0 ; i < 4 ; i++){
				SetGlobalLegPos(i, pos[i].X, pos[i].Y, pos[i].Z);
				SetEnd2EndMotionParam(i, 2.0, end[i]);
				SetEnd2EndMotionReset(i);
			}
		}
		
		static int mode = 0;
		static int mleg_number = 0;
		static double timer = 0.0;
		static XYZ centroid[4];
		static bool ms_trig = true;
		int mlegNumber[4];
		mlegNumber[0] = 0;
		mlegNumber[1] = 2;
		mlegNumber[2] = 1;
		mlegNumber[3] = 3;
		
		//XYZ CalcCentroidOfSquare();

		switch(mode){
			case 0:{
				/* move body first of all */
				double duration = 2.0;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 1;
					break;
				}
				XYZ vel;
				vel.X = 0.0;
				vel.Y = 0.0;
				vel.Z = 0.0;
				MoveShift(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 1:{
				/* move body */
				double duration = 0.3;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 2;
					ms_trig = true;
					break;
				}
				
				static XYZ vel;
				if(ms_trig == true){
					ms_trig = false;
					XYZ cent = CalcCentroidOfSquare();
					//cout << "centroid of square: " << cent.X << " / " << cent.Y << endl;
					XYZ target;
					if( mlegNumber[mleg_number] == 0
					 || mlegNumber[mleg_number] == 3){
						//cout << endl << "Left Leg" << endl << endl;
						target.X = ( g_LegPos[1].X + g_LegPos[2].X ) / 2.0;
						target.Y = - 0.04 + cent.Y;
					}else if( mlegNumber[mleg_number] == 1
						   || mlegNumber[mleg_number] == 2){
						//cout << endl << "Right Leg" << endl << endl;
						target.X = ( g_LegPos[0].X + g_LegPos[3].X ) / 2.0;
						target.Y =  0.04 + cent.Y;
					}
					vel.X = -(target.X) / duration;
					vel.Y = -(target.Y) / duration;
					vel.Z = 0.0;
					
					if(-0.001 < vel.X && vel.X < 0.001
					 &&-0.001 < vel.Y && vel.Y < 0.001){
						SetLegVelocityAllZero();
						timer = 0.0;
						mode = 2;
						ms_trig = true;
						break;
					}
				}
				
				MoveShift(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 2:{
				/* move leg */
				//up
				double duration = 0.5;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 3;
					break;
				}
				XYZ vel;
				vel.X = 0.1;
				vel.Y = 0.0;
				vel.Z = 0.2;
				SetLegVelocity(mlegNumber[mleg_number], vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
			
			case 3:{
				/* move leg */
				//down
				double duration = 0.5;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 1;
					mleg_number++;
					if(3 < mleg_number){
						mleg_number = 0;
					}
					break;
				}
				XYZ vel;
				vel.X = 0.03;
				vel.Y = 0.0;
				vel.Z = -0.2;
				SetLegVelocity(mlegNumber[mleg_number], vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			default:
				break;
		}
		//cout << "mode: " << mode << " /mleg: " << mleg_number << endl;
		//cout << "leg vel: " << LegVel[0].X << endl;
		//cout << "leg pos: " << g_LegPos[0].X << endl;
		//cout << "centroid: " << centroid[mleg_number].X << " / " << centroid[mleg_number].Y << endl;
		
		CalcPosFromVel();
		
		time += nsTimeKeeper::SAMPLING_TIME;
	}
	
	/*-------------------------*/
	/* Generate Walking Motion */
	/*-------------------------*/
	void InsectBot_Motions::GenerateMotion_Gamepad(XYZ velocity, XYZ angular_velocity){
		static XYZ pos[4];
		static XYZ end[4];
		static double time = 0.0;
		
		/*------*/
		/* init */
		/*------*/
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			pos[0].X =  0.14;	pos[0].Y =  0.14;	pos[0].Z = -0.1;
			pos[1].X =  0.14;	pos[1].Y = -0.14;	pos[1].Z = -0.1;
			pos[2].X = -0.14;	pos[2].Y = -0.14;	pos[2].Z = -0.1;
			pos[3].X = -0.14;	pos[3].Y =  0.14;	pos[3].Z = -0.1;
			for(int i = 0 ; i < 4 ; i++){
				end[i] = pos[i];
			}
			for(int i = 0 ; i < 4 ; i++){
				SetGlobalLegPos(i, pos[i].X, pos[i].Y, pos[i].Z);
				SetEnd2EndMotionParam(i, 2.0, end[i]);
				SetEnd2EndMotionReset(i);
			}
		}
		
		static int mode = 0;
		static int mleg_number = 0;
		static double timer = 0.0;
		static XYZ centroid[4];
		static bool ms_trig = true;
		int mlegNumber[4];
		mlegNumber[0] = 0;
		mlegNumber[1] = 2;
		mlegNumber[2] = 1;
		mlegNumber[3] = 3;
		
		//XYZ CalcCentroidOfSquare();

		switch(mode){
			case 0:{
				/* move body first of all */
				double duration = 2.0;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 1;
					break;
				}
				XYZ vel;
				vel.X = 0.0;
				vel.Y = 0.0;
				vel.Z = 0.0;
				MoveShift(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 1:{
				/* move body */
				double duration = 0.5;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 2;
					ms_trig = true;
					break;
				}
				
				static XYZ vel;
				if(ms_trig == true){
					ms_trig = false;
					XYZ cent = CalcCentroidOfSquare();
					//cout << "centroid of square: " << cent.X << " / " << cent.Y << endl;
					XYZ target;
					if( mlegNumber[mleg_number] == 0
					 || mlegNumber[mleg_number] == 3){
						//cout << endl << "Left Leg" << endl << endl;
						target.X = ( g_LegPos[1].X + g_LegPos[2].X ) / 2.0;
						target.Y = - 0.04 + cent.Y;
					}else if( mlegNumber[mleg_number] == 1
						   || mlegNumber[mleg_number] == 2){
						//cout << endl << "Right Leg" << endl << endl;
						target.X = ( g_LegPos[0].X + g_LegPos[3].X ) / 2.0;
						target.Y =  0.04 + cent.Y;
					}
					vel.X = -(target.X) / duration;
					vel.Y = -(target.Y) / duration;
					vel.Z = 0.0;
					
					if(-0.001 < vel.X && vel.X < 0.001
					 &&-0.001 < vel.Y && vel.Y < 0.001){
						SetLegVelocityAllZero();
						timer = 0.0;
						mode = 2;
						ms_trig = true;
						break;
					}
				}
				
				MoveShift(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 2:{
				/* move leg */
				//up
				double duration = 0.3;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 3;
					break;
				}
				XYZ vel;
				vel.X = velocity.X;
				vel.Y = velocity.Y;
				vel.Z = 0.2;
				SetLegVelocity(mlegNumber[mleg_number], vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
			
			case 3:{
				/* move leg */
				//down
				double duration = 0.3;
				if(duration < timer){
					SetLegVelocityAllZero();
					timer = 0.0;
					mode = 1;
					mleg_number++;
					if(3 < mleg_number){
						mleg_number = 0;
					}
					break;
				}
				XYZ vel;
				vel.X = velocity.X;
				vel.Y = velocity.Y;
				vel.Z = -0.2;
				SetLegVelocity(mlegNumber[mleg_number], vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			default:
				break;
		}
		//cout << "mode: " << mode << " /mleg: " << mleg_number << endl;
		//cout << "leg vel: " << LegVel[0].X << endl;
		//cout << "leg pos: " << g_LegPos[0].X << endl;
		//cout << "centroid: " << centroid[mleg_number].X << " / " << centroid[mleg_number].Y << endl;
		
		CalcPosFromVel();
		
		time += nsTimeKeeper::SAMPLING_TIME;
	}
	
	void InsectBot_Motions::CalcPosFromVel(){
		for(int i = 0 ; i < 4 ; i++){
			g_LegPos[i].X += LegVel[i].X * nsTimeKeeper::SAMPLING_TIME;
			g_LegPos[i].Y += LegVel[i].Y * nsTimeKeeper::SAMPLING_TIME;
			g_LegPos[i].Z += LegVel[i].Z * nsTimeKeeper::SAMPLING_TIME;
		}
		
		for(int i = 0 ; i < 4 ; i++){
//			XYZ pos = g_LegPos[i];
//			SetGlobalLegPos(i, pos.X, pos.Y, pos.Z);
			Global2Local(i);
			Local2LegAngles(i);
		}
	}

	/*-----------*/
	/* Get Angle */
	/*-----------*/
	double InsectBot_Motions::GetLegAngles(int leg_number, int joint_number){
		return LegAngle[leg_number].Angle[joint_number];
	}
}

