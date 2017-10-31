/* test_servoBlaster02 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <linux/input.h>
#include <wiringPi.h>
#include <math.h>
#include <Eigen/Dense>

#include "ServoBlaster.h"
#include "TimeKeeper.h"
#include "Kinematics3DOF.h"
#include "ServoAdjuster.h"
#include "InsectBot_Motions.h"
#include "JoystickDriver.h"
#include "SensorInfo.h"
#include "Bot_Configuration.h"
#include "Bot_Walking.h"
#include "AttitudeControl.h"

using namespace std;

const unsigned int BTN_PORT = 12;	//BCM(GPIO)

int main(void){
	/*-------------------------------*/
	/* Read Servo Adjuster Paramters */
	/*-------------------------------*/
	nsServoAdjuster::ServoAdjuster sa;
	sa.ReadParameterFile();
	vector<unsigned int> p1pins = sa.GetP1Pins();

	/*--------------------*/
	/* setup ServoBlaster */
	/*--------------------*/
	//sudo servod --p1pins=3,5,7,11,13,15,19,21,23,29,31,33 --min=100us --max=3000us
	nsServoBlaster::ServoBlaster sb;
	sb.Set_P1Pins(p1pins);
	sb.Set_MinPalseWidth_us(100);
	sb.Set_MaxPalseWidth_us(3000);
	sb.OpenDeviceFile();
	
	system("pwd");
	
	/*------------*/
	/* setup GPIO */
	/*------------*/
	if(wiringPiSetupGpio() == -1){
		cout << "Can not setup GPIO" << endl;
		return -1;
	}else{
		pinMode(BTN_PORT, INPUT);
	}
	
	/*----------*/
	/* Joystick */
	/*----------*/
	nsJoystickDriver::JoystickDriver js;
	
	/*---------------*/
	/* Sensor Info   */
	/*---------------*/
	nsSensorInfo::SensorInfo sensInfo;
	
	/*-------------------*/
	/* Bot Configuration */
	/*-------------------*/
	nsBot_Configuration::Bot_Configuration botConf;
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
	botConf.UpdatePrameter();
	
	/*-------------------*/
	/* InsectBot Motions */
	/*-------------------*/
	nsBot_Walking::Bot_Walking botWalking;
	//nsInsectBot_Motions::InsectBot_Motions ibotm;

	/*------------*/
	/* time count */
	/*------------*/
	nsTimeKeeper::TimeKeeper tkeeper;

	/*-----------*/
	/* main loop */
	/*-----------*/
	while(true){
		/*---------------------*/
		/* Read Digital Signal */
		/*---------------------*/
		int btnSignal = digitalRead(BTN_PORT);
		//cout <<"btn Signal = " << btnSignal << endl;
		
		/* Read GamepadInfo */
		js.Update();

		/*-------*/
		/* break */
		/*-------*/
		if(btnSignal == 0 || js.GetButton_Start() == 1){
			break;
		}
		
		/*------------------*/
		/* InsectBot motion */
		/*------------------*/
//		nsInsectBot_Motions::XYZ velocity;
//		nsInsectBot_Motions::XYZ angular_velocity;
//		velocity.X = js.GetLStick_Y() * 0.15;
//		velocity.Y = js.GetLStick_X() * 0.15;
//		velocity.Z = js.GetRStick_Y() * 0.15;
//		ibotm.GenerateMotion_Gamepad(velocity, angular_velocity);
		botWalking.GenerateMotion();
		botConf = botWalking.Get_BotConf();
		
		/*-------------*/
		/* Sensor Info */
		/*-------------*/
		sensInfo.Update();
		nsBot_Configuration::Bot_Configuration::XYZ acc;
		acc.X = sensInfo.Get_Acc_X();
		acc.Y = sensInfo.Get_Acc_Y();
		acc.Z = sensInfo.Get_Acc_Z();
		nsBot_Configuration::Bot_Configuration::XYZ ang_vel;
		ang_vel.X = sensInfo.Get_AngVel_X();
		ang_vel.Y = sensInfo.Get_AngVel_Y();
		ang_vel.Z = sensInfo.Get_AngVel_Z();
		
		/*------------------*/
		/* Attitude Control */
		/*------------------*/
		nsAttitudeControl::AttitudeControl attiCont;
		attiCont.Set_BotConf(botConf);
		attiCont.Set_Acc(acc);
		attiCont.Set_AngVel(ang_vel);
		attiCont.Calculation();
		nsBot_Configuration::Bot_Configuration botConf_Mod;
		botConf_Mod = attiCont.Get_Modified_BotConf();
//		cout << "         ang: " << botConf.Get_LegAngle(0, 3) << endl;
//		cout << "modified ang: " << botConf_Mod.Get_LegAngle(0, 3) << endl;
//		cout << "        diff: " << botConf_Mod.Get_LegAngle(0, 3) - botConf.Get_LegAngle(0, 3) << endl;
		
		double ang[4][3];
		for(int i = 0 ; i < 4 ; i++){
			for(int j = 0 ; j < 3 ; j++){
				//ang[i][j] = ibotm.GetLegAngles(i, j);
				//ang[i][j] = botConf.Get_LegAngle(i, j);
				ang[i][j] = botConf_Mod.Get_LegAngle(i, j);
			}
		}
		
		/* Leg 1 */
		sb.SendPulseWidth_us(0, sa.Angle2PW_us(0, ang[0][0]));
		sb.SendPulseWidth_us(1, sa.Angle2PW_us(1, ang[0][1]));
		sb.SendPulseWidth_us(2, sa.Angle2PW_us(2, ang[0][2]));
		
		/* Leg 2 */
		sb.SendPulseWidth_us(3, sa.Angle2PW_us(3, ang[1][0]));
		sb.SendPulseWidth_us(4, sa.Angle2PW_us(4, ang[1][1]));
		sb.SendPulseWidth_us(5, sa.Angle2PW_us(5, ang[1][2]));
		
		/* Leg 3 */
		sb.SendPulseWidth_us(6, sa.Angle2PW_us(6, ang[2][0]));
		sb.SendPulseWidth_us(7, sa.Angle2PW_us(7, ang[2][1]));
		sb.SendPulseWidth_us(8, sa.Angle2PW_us(8, ang[2][2]));
		
		/* Leg 4 */
		sb.SendPulseWidth_us(9, sa.Angle2PW_us(9, ang[3][0]));
		sb.SendPulseWidth_us(10, sa.Angle2PW_us(10, ang[3][1]));
		sb.SendPulseWidth_us(11, sa.Angle2PW_us(11, ang[3][2]));

		/* must wait */
		tkeeper.TimeKeep();
		//cout << "Elapsed Time: " << tkeeper.GetElapsedTime() << endl;
		//cout << "proc time: " << tkeeper.GetProcessingTime() << endl;

	}

	/*-----------*/
	/* Terminate */
	/*-----------*/
	cout << "Terminated" << endl;

	return 0;
}
