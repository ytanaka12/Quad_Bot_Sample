/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Kinematics3DOF.cpp
 * Author: yasufumi
 * 
 * Created on 2017/06/26, 19:52
 */

#include "Kinematics3DOF.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;

namespace nsKinematics3DOF{

	Kinematics3DOF::Kinematics3DOF() {
		Ang[0] = 0.0;
		Ang[1] = 0.0;
		Ang[2] = 0.0;
		
		PosX = 0.0;
		PosY = 0.0;
		PosZ = 0.0;
		
//		MatrixXd m(2,2);
//		m(0,0) = 3;
//		m(1,0) = 2.5;
//		m(0,1) = -1;
//		m(1,1) = m(1,0) + m(0,1);
//		std::cout << m << std::endl;
	}

	Kinematics3DOF::Kinematics3DOF(const Kinematics3DOF& orig) {
	}

	Kinematics3DOF::~Kinematics3DOF() {
	}
	
	/*------------*/
	/* Set Angles */
	/*------------*/
	void Kinematics3DOF::SetAngle(int number, double angle){
		Ang[number] = angle;
	}
	
	/*--------------*/
	/* Set Position */
	/*--------------*/
	void Kinematics3DOF::SetPosition(double x, double y, double z){
		PosX = x;
		PosY = y;
		PosZ = z;
	}
	
	Eigen::Matrix4d HomogeneousTransMatrix(int number, double angle){
		Eigen::Matrix4d mat;
		
		mat(0, 0) = cos(angle + P_TH[number]);
		mat(0, 1) = - sin(angle + P_TH[number]) * cos(P_AL[number]);
		mat(0, 2) = sin(angle + P_TH[number]) * sin(P_AL[number]);
		mat(0, 3) = P_A[number] * cos(angle + P_TH[number]);
		
		mat(1, 0) = sin(angle + P_TH[number]);
		mat(1, 1) = cos(angle + P_TH[number]) * cos(P_AL[number]);
		mat(1, 2) = - cos(angle + P_TH[number]) * sin(P_AL[number]);
		mat(1, 3) = P_A[number] * sin(angle + P_TH[number]);
		
		mat(2, 0) = 0.0;
		mat(2, 1) = sin(P_AL[number]);
		mat(2, 2) = cos(P_AL[number]);
		mat(2, 3) = P_D[number];
		
		mat(3, 0) = 0.0;
		mat(3, 1) = 0.0;
		mat(3, 2) = 0.0;
		mat(3, 3) = 1.0;
		
		return mat;
	}
	
	/*----------------------*/
	/* Calculate Kinematics */
	/*----------------------*/
	void Kinematics3DOF::CalcKinematics(){
		Eigen::Vector3d pos(0,0,0);
		
		Eigen::Matrix4d htMat_1 = HomogeneousTransMatrix(0, Ang[0]);
		Eigen::Matrix4d htMat_2 = HomogeneousTransMatrix(1, Ang[1]);
		Eigen::Matrix4d htMat_3 = HomogeneousTransMatrix(2, Ang[2]);
		
		Eigen::Matrix4d AnsMat = htMat_1 * htMat_2 * htMat_3;
		
		PosX = AnsMat(0, 3);
		PosY = AnsMat(1, 3);
		PosZ = AnsMat(2, 3);
		
//		cout << "---" << endl;
//		cout << "htMat:" << endl;
//		cout << htMat_1 * htMat_2 << endl;
//		cout << "---" << endl;
	}
	
	/*------------------------------*/
	/* Calculate Inverse Kinematics */
	/*------------------------------*/
	void Kinematics3DOF::CalcInverseKinematics(){
		Ang[0] = atan2(PosY, PosX);

		/* position of Joint2 */
		Eigen::Vector3d pos1(L_LINK_1, 0, 0);
		Eigen::Matrix3d rot1;
		rot1 = Eigen::AngleAxisd(Ang[0], Eigen::Vector3d(0,0,1));
		pos1 = rot1 * pos1;
		
		double a = sqrt( pow(PosX - pos1[0], 2.0) 
						+ pow(PosY - pos1[1], 2.0) 
						+ pow(PosZ - pos1[2], 2.0) );
		double b = L_LINK_2;
		double c = L_LINK_3;
		Ang[2] = acos( (b * b + c * c - a * a) / (2.0 * b * c) );
		Ang[2] = - ( Ang[2] - 90.0 * DEG2RAD );
		
		/* preparation of calculation angle2 */
		Eigen::Vector3d pos(PosX, PosY, PosZ);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(- Ang[0], Eigen::Vector3d(0,0,1));
		pos = rot * pos;
		
		double gamma = acos( (a * a + b * b - c * c) / (2.0 * a * b) );
		double beta = atan2(pos[2], pos[0] - L_LINK_1);
		
		Ang[1] = - ( gamma + beta );
	}
	
	/*--------------*/
	/* Get Position */
	/*--------------*/
	void Kinematics3DOF::GetPosition(double pos[]){
		pos[0] = PosX;
		pos[1] = PosY;
		pos[2] = PosZ;
	}
	
	/*----------------*/
	/* Get Position X */
	/*----------------*/
	double Kinematics3DOF::GetPosition_X(){
		return PosX;
	}
	
	/*----------------*/
	/* Get Position Y */
	/*----------------*/
	double Kinematics3DOF::GetPosition_Y(){
		return PosY;
	}
	
	/*----------------*/
	/* Get Position Z */
	/*----------------*/
	double Kinematics3DOF::GetPosition_Z(){
		return PosZ;
	}
	
	/*-----------*/
	/* Get Angle */
	/*-----------*/
	double Kinematics3DOF::GetAngle(int number){
		return Ang[number];
	}

}













