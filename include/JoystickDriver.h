/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   JoystickDriver.h
 * Author: yasufumi
 *
 * Created on 2017/07/03, 17:22
 */

#ifndef JOYSTICKDRIVER_H
#define JOYSTICKDRIVER_H

#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

namespace nsJoystickDriver{
	
	const char JOY_DEV[30] = "/dev/input/js0";
	const int MAX_VALUE_ANALOG = 32767;

	class JoystickDriver {
	private:
		bool IsGamepad;
		int joy_fd, num_of_axis, num_of_buttons;
		char name_of_joystick[80];
		std::vector<char> joy_button;
		std::vector<int> joy_axis;
		
		double LStick_X;
		double LStick_Y;
		double RStick_X;
		double RStick_Y;
		double LTrigger;
		double RTrigger;
		
		int Button_A;
		int Button_B;
		int Button_X;
		int Button_Y;
		
		int Button_L;
		int Button_R;
		
		int Button_Start;
		int Button_Back;
		
		int D_PAD_Up;
		int D_PAD_Down;
		int D_PAD_Left;
		int D_PAD_RIGHT;
		
		void Init();
		void Terminate();

	public:
		JoystickDriver();
		JoystickDriver(const JoystickDriver& orig);
		virtual ~JoystickDriver();
		
		void Update();
		
		double GetLStick_X();
		double GetLStick_Y();
		double GetRStick_X();
		double GetRStick_Y();
		double GetLTrigger();
		double GetRTrigger();
		
		int GetButton_A();
		int GetButton_B();
		int GetButton_X();
		int GetButton_Y();
		
		int GetButton_L();
		int GetButton_R();
		
		int GetButton_Start();
		int GetButton_Back();
		
		int GetD_PAD_Up();
		int GetD_PAD_Down();
		int GetD_PAD_Left();
		int GetD_PAD_RIGHT();

	};

}

#endif /* JOYSTICKDRIVER_H */

