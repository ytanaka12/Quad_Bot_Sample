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
		
		nsBot_Configuration::Bot_Configuration Get_BotConf();
	};
}

#endif /* BOT_WALKING_H */

