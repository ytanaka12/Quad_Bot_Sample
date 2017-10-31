/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Bot_Walking.cpp
 * Author: yasufumi
 * 
 * Created on 2017/08/15, 22:23
 */

#include "Bot_Walking.h"

#include <iostream>
#include "TimeKeeper.h"

using namespace std;

namespace nsBot_Walking{

	Bot_Walking::Bot_Walking() {
	}

	Bot_Walking::Bot_Walking(const Bot_Walking& orig) {
	}

	Bot_Walking::~Bot_Walking() {
	}
	
	/*-------------------------------*/
	/* Calculate Cntroid of Triangle */
	/*-------------------------------*/
	nsBot_Configuration::Bot_Configuration::XYZ Bot_Walking::CalcCentroidOfTriangle(int num){
		nsBot_Configuration::Bot_Configuration::XYZ ans;
		nsBot_Configuration::Bot_Configuration::XYZ leg_pos[4];
		
		for(int i = 0 ; i < 4 ; i++){
			leg_pos[i] = botConf.Get_LegPos(i);
		}
		
		int n1, n2, n3;
		switch(num){
			case 0:
				n1 = 1;	n2 = 2;	n3 = 3;
				break;
			case 1:
				n1 = 0;	n2 = 2;	n3 = 3;
				break;
			case 2:
				n1 = 0;	n2 = 1;	n3 = 3;
				break;
			case 3:
				n1 = 0;	n2 = 1;	n3 = 2;
				break;
			default:
				break;
		}
		
		ans.X = (leg_pos[n1].X + leg_pos[n2].X + leg_pos[n3].X) / 3.0;
		ans.Y = (leg_pos[n1].Y + leg_pos[n2].Y + leg_pos[n3].Y) / 3.0;
		ans.Z = (leg_pos[n1].Z + leg_pos[n2].Z + leg_pos[n3].Z) / 3.0;
		
		return ans;
	}

	/*------------------------------*/
	/* Calculate Centroid of Square */
	/*------------------------------*/
	nsBot_Configuration::Bot_Configuration::XYZ Bot_Walking::CalcCentroidOfSquare(){
		nsBot_Configuration::Bot_Configuration::XYZ ans;
		nsBot_Configuration::Bot_Configuration::XYZ leg_pos[4];
		for(int i = 0 ; i < 4 ; i++){
			leg_pos[i] = botConf.Get_LegPos(i);
		}
		
		ans.X = 0;
		ans.Y = 0;
		ans.Z = 0;
		for(int i = 0 ; i < 4 ; i++){
			ans.X += leg_pos[i].X;
			ans.Y += leg_pos[i].Y;
			ans.Z += leg_pos[i].Z;
		}
		ans.X = ans.X / 4.0;
		ans.Y = ans.Y / 4.0;
		ans.Z = ans.Z / 4.0;
		
		return ans;
	}
	
	/*-------------------------*/
	/* Generate Walking Motion */
	/*-------------------------*/
	void Bot_Walking::GenerateMotion(){
		static double time = 0.0;
		
		/*------*/
		/* init */
		/*------*/
		static bool init_trig = true;
		if(init_trig == true){
			init_trig = false;
			nsBot_Configuration::Bot_Configuration::XYZ init_LegPos[4];
			for(int i = 0 ; i < 4 ; i++){
				init_LegPos[i].X =  0.14;
				init_LegPos[i].Y =  0.14;
				init_LegPos[i].Z = -0.08;
			}
										init_LegPos[1].Y *= -1.0;
			init_LegPos[2].X *= -1.0;	init_LegPos[2].Y *= -1.0;
			init_LegPos[3].X *= -1.0;
			for(int i = 0 ; i < 4 ; i++){
				botConf.Set_LegPos(i, init_LegPos[i]);
			}
			botConf.Set_LegVel_AllZero();
			//botConf.UpdatePrameter();
		}
		
		static int mode = 0;
		static int mleg_number = 0;
		static double timer = 0.0;
		static nsBot_Configuration::Bot_Configuration::XYZ centroid[4];
		static bool ms_trig = true;
		int mlegNumber[4];
		mlegNumber[0] = 0;
		mlegNumber[1] = 2;
		mlegNumber[2] = 1;
		mlegNumber[3] = 3;

		switch(mode){
			case 0:{
				/* move body first of all */
				double duration = 2.0;
				if(duration < timer){
					botConf.Set_LegVel_AllZero();
					timer = 0.0;
					mode = 1;
					break;
				}
				nsBot_Configuration::Bot_Configuration::XYZ vel;
				vel.X = 0.0;
				vel.Y = 0.0;
				vel.Z = 0.0;
				botConf.Set_LegVel_All(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 1:{
				/* move body */
				//double duration = 1.0;
				double duration = 1.0;
				if(duration < timer){
					botConf.Set_LegVel_AllZero();
					timer = 0.0;
					mode = 2;
					ms_trig = true;
					break;
				}
				
				static nsBot_Configuration::Bot_Configuration::XYZ vel;
				if(ms_trig == true){
					ms_trig = false;
					//nsBot_Configuration::Bot_Configuration::XYZ cent = CalcCentroidOfSquare();
					//cout << "centroid of square: " << cent.X << " / " << cent.Y << endl;
					nsBot_Configuration::Bot_Configuration::XYZ cent = CalcCentroidOfTriangle(mlegNumber[mleg_number]);
					nsBot_Configuration::Bot_Configuration::XYZ target;
					nsBot_Configuration::Bot_Configuration::XYZ leg_pos[4];
					for(int i = 0 ; i < 4 ; i++){
						leg_pos[i] = botConf.Get_LegPos(i);
					}
					if( mlegNumber[mleg_number] == 0
					 || mlegNumber[mleg_number] == 3){
						//cout << endl << "Left Leg" << endl << endl;
						//target.X = ( leg_pos[1].X + leg_pos[2].X ) / 2.0 + 0.03;
						target.X = cent.X;
						target.Y = - 0.0 + cent.Y;
					}else if( mlegNumber[mleg_number] == 1
						   || mlegNumber[mleg_number] == 2){
						//cout << endl << "Right Leg" << endl << endl;
						//target.X = ( leg_pos[0].X + leg_pos[3].X ) / 2.0 + 0.0;
						target.X = cent.X;
						target.Y =  0.0 + cent.Y;
					}
					vel.X = -(target.X) / duration;
					vel.Y = -(target.Y) / duration;
					vel.Z = 0.0;
					
					if(-0.001 < vel.X && vel.X < 0.001
					 &&-0.001 < vel.Y && vel.Y < 0.001){
						botConf.Set_LegVel_AllZero();
						timer = 0.0;
						mode = 2;
						ms_trig = true;
						break;
					}
				}
				
				botConf.Set_LegVel_All(vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
				
			case 2:{
				/* move leg */
				//up
				double duration = 0.3;
				//double duration = 1.0;
				if(duration < timer){
					botConf.Set_LegVel_AllZero();
					timer = 0.0;
					mode = 3;
					break;
				}
				nsBot_Configuration::Bot_Configuration::XYZ vel;
				vel.X = 0.1;
				vel.Y = 0.0;
				vel.Z = 0.2;
				botConf.Set_LegVel(mlegNumber[mleg_number], vel);
				timer += nsTimeKeeper::SAMPLING_TIME;
				break;
			}
			
			case 3:{
				/* move leg */
				//down
				double duration = 0.3;
				//double duration = 1.0;
				if(duration < timer){
					botConf.Set_LegVel_AllZero();
					timer = 0.0;
					mode = 1;
					mleg_number++;
					if(3 < mleg_number){
						mleg_number = 0;
					}
					break;
				}
				nsBot_Configuration::Bot_Configuration::XYZ vel;
				vel.X = 0.03;
				vel.Y = 0.0;
				vel.Z = -0.2;
				botConf.Set_LegVel(mlegNumber[mleg_number], vel);
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
		
		botConf.Set_LegVel_AllZero();
		botConf.UpdatePrameter();
		
//		nsBot_Configuration::Bot_Configuration::XYZ bufpos;
//		bufpos = botConf.Get_LegPos(0);
//		cout << "leg pos: " << bufpos.X << " / " << bufpos.Y << " / " << bufpos.Z << endl;
		
		time += nsTimeKeeper::SAMPLING_TIME;
	}
	
	nsBot_Configuration::Bot_Configuration Bot_Walking::Get_BotConf(){
		nsBot_Configuration::Bot_Configuration ans(botConf);
		return ans;
	}
}


