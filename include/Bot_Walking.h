/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Bot_Walking.h
 * Author: yasufumi
 *
 * Created on 2017/08/15, 22:23
 */

#ifndef BOT_WALKING_H
#define BOT_WALKING_H

#include "Bot_Configuration.h"

namespace nsBot_Walking{
	
	const double INIT_LEG_POS_X = 0.12;
	const double INIT_LEG_POS_Y = 0.12;
	const double INIT_LEG_POS_Z =-0.12;
	
	class Bot_Walking {
		
	private:
		nsBot_Configuration::Bot_Configuration botConf;
		
	public:
		Bot_Walking();
		Bot_Walking(const Bot_Walking& orig);
		virtual ~Bot_Walking();
		
		nsBot_Configuration::Bot_Configuration::XYZ CalcCentroidOfTriangle(int num);
		nsBot_Configuration::Bot_Configuration::XYZ CalcCentroidOfSquare();
		void GenerateMotion();
		void GenerateMotion_LikeSway(double forward_vel, double yaw_rate);
		bool GenerateMotion_StandBy();
		
		nsBot_Configuration::Bot_Configuration Get_BotConf();
	};
}

#endif /* BOT_WALKING_H */

