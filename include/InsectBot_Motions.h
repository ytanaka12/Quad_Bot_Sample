/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InsectBot_Motions.h
 * Author: yasufumi
 *
 * Created on 2017/06/29, 19:38
 */

#ifndef INSECTBOT_MOTIONS_H
#define INSECTBOT_MOTIONS_H

#include "Kinematics3DOF.h"
#include <Eigen/Dense>

namespace nsInsectBot_Motions{
	const unsigned int NUM_LEG = 4;
	const unsigned int NUM_JOINT_PER_LEG = 3;
	
	const double OFFSET_LENGTH_X = 0.050;
	const double OFFSET_LENGTH_Y = 0.050;
	
	struct XYZ{
		double X;
		double Y;
		double Z;
	};
	
	struct LegAngles{
		double Angle[NUM_JOINT_PER_LEG];
	};

	class InsectBot_Motions {
	private:
		nsKinematics3DOF::Kinematics3DOF kine;
		
		XYZ g_LegPos[NUM_LEG];	//Global
		XYZ l_LegPos[NUM_LEG];	//Local
		
		//end to end
		XYZ g_StartPos[NUM_LEG];
		XYZ g_EndPos[NUM_LEG];
		double DurationTime[NUM_LEG];
		double Timer[NUM_LEG];
		XYZ LegVel[NUM_LEG];

		LegAngles LegAngle[NUM_LEG];
		
	public:
		InsectBot_Motions();
		InsectBot_Motions(const InsectBot_Motions& orig);
		virtual ~InsectBot_Motions();
		
		void Init();
		
		//---
		void SetGlobalLegPos(int leg_number, double x, double y, double z);
		void SetLocalLegPos(int leg_number, double x, double y, double z);
		
		void Global2Local(int leg_number);
		void Local2LegAngles(int leg_number);
		//---
		
		void MoveShift(XYZ vel);
		void Turn(double ang_vel);

		/* Motions */
		void SetLegVelocity(int leg_number, XYZ vel);
		void SetLegVelocityAllZero();		
		void SetEnd2EndMotionParam(int leg_number, double duration_time, XYZ end);
		void SetEnd2EndMotionReset(int leg_number);
		void GenerateEnd2EndMotion(int leg_number);
		
		double CalcSquareOfTriangle(int n1, int n2, int n3);
		XYZ CalcCentroidOfTriangle(int n1, int n2, int n3);
		XYZ CalcCentroidOfSquare();
		
		void GenerateMoving();
		void GenerateMotion_Gamepad(XYZ velocity, XYZ angular_velocity);
		void CalcPosFromVel();
		
		//---
		double GetLegAngles(int leg_number, int joint_number);
		//---
	};

}

#endif /* INSECTBOT_MOTIONS_H */

