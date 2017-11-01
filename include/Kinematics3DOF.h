/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Kinematics3DOF.h
 * Author: yasufumi
 *
 * Created on 2017/06/26, 19:52
 */

#ifndef KINEMATICS3DOF_H
#define KINEMATICS3DOF_H

namespace nsKinematics3DOF{
	const double DEG2RAD = 3.1415 / 180.0;
	const double RAD2DEG = 180.0 / 3.1415;

	/* Link Length */
	const double L_LINK_1 = 0.074;	//[m]
	const double L_LINK_2 = 0.090;	//[m]
	const double L_LINK_3 = 0.120;	//[m]

	/* Parameter of Homogeneous Transfor Matrix */
	const double P_TH[3] = {0.0 , 0.0 , 90.0 * DEG2RAD};
	const double P_D[3] = {0.0 , 0.0 , 0.0};
	const double P_A[3] = {L_LINK_1, L_LINK_2, L_LINK_3};
	const double P_AL[3] = {-90.0 * DEG2RAD , 0.0 , 0.0};

	/* Class */
	class Kinematics3DOF {
	private:
		double Ang[3];
		
		double PosX;
		double PosY;
		double PosZ;
		
	public:
		Kinematics3DOF();
		Kinematics3DOF(const Kinematics3DOF& orig);
		virtual ~Kinematics3DOF();

		void SetAngle(int number, double angle);
		void SetPosition(double x, double y, double z);

		void CalcKinematics();
		void CalcInverseKinematics();

		void GetPosition(double pos[]);
		double GetPosition_X();
		double GetPosition_Y();
		double GetPosition_Z();
		double GetAngle(int number);

	};

}

#endif /* KINEMATICS3DOF_H */

