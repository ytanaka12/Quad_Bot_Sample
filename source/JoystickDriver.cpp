/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   JoystickDriver.cpp
 * Author: yasufumi
 * 
 * Created on 2017/07/03, 17:22
 */

#include "JoystickDriver.h"

#include <iostream>

using namespace std;

namespace nsJoystickDriver{

	JoystickDriver::JoystickDriver() {
		Init();
		
		LStick_X = 0.0;
		LStick_Y = 0.0;
		RStick_X = 0.0;
		RStick_Y = 0.0;
		LTrigger = 0.0;
		RTrigger = 0.0;
		
		Button_A = 0;
		Button_B = 0;
		Button_X = 0;
		Button_Y = 0;
		
		Button_L = 0;
		Button_R = 0;
		
		Button_Start = 0;
		Button_Back = 0;
		
		D_PAD_Up = 0;
		D_PAD_Down = 0;
		D_PAD_Left = 0;
		D_PAD_RIGHT = 0;
	}

	JoystickDriver::JoystickDriver(const JoystickDriver& orig) {
	}

	JoystickDriver::~JoystickDriver() {
		Terminate();
	}
	
	/*------*/
	/* Init */
	/*------*/
	void JoystickDriver::Init(){
		joy_fd = -1;
		num_of_axis = 0;
		num_of_buttons = 0;
		
		if((joy_fd=open(JOY_DEV,O_RDONLY)) < 0){
			cerr<<"Failed to open Device file of Gamepad "<<JOY_DEV<<endl;
			IsGamepad = false;
		}else{
			IsGamepad = true;
		}

		ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
		ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
		ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

		joy_button.resize(num_of_buttons,0);
		joy_axis.resize(num_of_axis,0);

		cout<<"Joystick: "<<name_of_joystick<<endl
		<<"  axis: "<<num_of_axis<<endl
		<<"  buttons: "<<num_of_buttons<<endl;

		fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
	}

	/*-----------*/
	/* Terminate */
	/*-----------*/	
	void JoystickDriver::Terminate(){
		close(joy_fd);
	}
	
	/*--------*/
	/* Update */
	/*--------*/	
	void JoystickDriver::Update(){
		js_event js;
		
		if(IsGamepad != true){
			return;
		}

		read(joy_fd, &js, sizeof(js_event));

		switch (js.type & ~JS_EVENT_INIT){
			case JS_EVENT_AXIS:
				if((int)js.number>=joy_axis.size())	{cerr<<"err:"<<(int)js.number<<endl;}
				joy_axis[(int)js.number]= js.value;
				break;
			case JS_EVENT_BUTTON:
				if((int)js.number>=joy_button.size())	{cerr<<"err:"<<(int)js.number<<endl;}
				joy_button[(int)js.number]= js.value;
				break;
		}
		
		LStick_X = (double)joy_axis[0] / (double)MAX_VALUE_ANALOG;
		LStick_Y = -(double)joy_axis[1] / (double)MAX_VALUE_ANALOG;
		LTrigger = (double)joy_axis[2] / (double)MAX_VALUE_ANALOG + 1.0;

		RStick_X = (double)joy_axis[3] / (double)MAX_VALUE_ANALOG;
		RStick_Y = -(double)joy_axis[4] / (double)MAX_VALUE_ANALOG;
		RTrigger = (double)joy_axis[5] / (double)MAX_VALUE_ANALOG + 1.0;
		
		Button_A = joy_button[0];
		Button_B = joy_button[1];
		Button_X = joy_button[2];
		Button_Y = joy_button[3];
		
		Button_L = joy_button[4];
		Button_R = joy_button[5];
		
		Button_Back = joy_button[6];
		Button_Start = joy_button[7];
	
//		D_PAD_Up;
//		D_PAD_Down;
//		D_PAD_Left;
//		D_PAD_RIGHT;
	}
		
	double JoystickDriver::GetLStick_X(){
		return LStick_X;
	}
	
	double JoystickDriver::GetLStick_Y(){
		return LStick_Y;
	}
	
	double JoystickDriver::GetRStick_X(){
		return RStick_X;
	}
	
	double JoystickDriver::GetRStick_Y(){
		return RStick_Y;
	}
	
	double JoystickDriver::GetLTrigger(){
		return LTrigger;
	}
	
	double JoystickDriver::GetRTrigger(){
		return RTrigger;
	}

	int JoystickDriver::GetButton_A(){
		return Button_A;
	}
	
	int JoystickDriver::GetButton_B(){
		return Button_B;
	}
	
	int JoystickDriver::GetButton_X(){
		return Button_X;
	}
	
	int JoystickDriver::GetButton_Y(){
		return Button_Y;
	}

	int JoystickDriver::GetButton_L(){
		return Button_L;
	}
	
	int JoystickDriver::GetButton_R(){
		return Button_R;
	}

	int JoystickDriver::GetButton_Start(){
		return Button_Start;
	}
	
	int JoystickDriver::GetButton_Back(){
		return Button_Back;
	}

	int JoystickDriver::GetD_PAD_Up(){
		return D_PAD_Up;
	}
	
	int JoystickDriver::GetD_PAD_Down(){
		return D_PAD_Down;
	}
	
	int JoystickDriver::GetD_PAD_Left(){
		return D_PAD_Left;
	}
	
	int JoystickDriver::GetD_PAD_RIGHT(){
		return D_PAD_RIGHT;
	}
}