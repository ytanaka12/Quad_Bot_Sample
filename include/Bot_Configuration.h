/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Bot_Configuration.h
 * Author: yasufumi
 *
 * Created on 2017/08/15, 12:42
 */

#ifndef BOT_CONFIGURATION_H
#define BOT_CONFIGURATION_H

namespace nsBot_Configuration{
	const unsigned int NUM_LEG = 4;
	const unsigned int NUM_JOINT_PER_LEG = 3;
	
	const double OFFSET_LENGTH_X = 0.050;
	const double OFFSET_LENGTH_Y = 0.050;
	
class Bot_Configuration {
	
	public:
		struct XYZ{
			double X;
			double Y;
			double Z;
		};

		struct LegInfoStruct{
			double Angle[NUM_JOINT_PER_LEG];
			XYZ LegPos;
			XYZ LegPos_Local;
			XYZ LegVel;
		};

	protected:
		LegInfoStruct LegInfo[NUM_LEG];

	public:
		Bot_Configuration();
		Bot_Configuration(const Bot_Configuration& orig);
		virtual ~Bot_Configuration();

		void Set_LegPos(int leg_number, XYZ pos);
		void Set_LegVel(int leg_number, XYZ vel);
		void Set_LegVel_All(XYZ vel);
		void Set_LegVel_AllZero();
		
		void Set_BodyVel(XYZ vel);
		void Set_BodyAngVel(XYZ ang_vel);

		void Global2Local(int leg_number);
		void Local2LegAngles(int leg_number);
		void UpdatePrameter();
		
		double Get_LegAngle(int leg_number, int joint_number);
		XYZ Get_LegPos(int leg_number);
		XYZ Get_LegVel(int leg_number);
		LegInfoStruct Get_LegInfo(int leg_number);
	};
}

#endif /* BOT_CONFIGURATION_H */

