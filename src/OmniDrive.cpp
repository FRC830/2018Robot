/*
 * OmniDrive.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: RatPack
 */

#include "OmniDrive.h"

OmniDrive::OmniDrive(VictorSP *fr, VictorSP *fl, VictorSP *mfl, VictorSP *mbl, VictorSP *mfr, VictorSP *mbr, VictorSP *back, frc::AnalogGyro *class_gyro) {
	fr_motor = fr;
	fl_motor = fl;
	mfl_motor = mfl;
	mbl_motor = mbl;
	mfr_motor = mfr;
	mbr_motor = mbr;
	back_motor = back;
	gyro = class_gyro;

};

float OmniDrive::avg(float left_value, float right_value) {
	return (left_value + right_value)/2;
};

void OmniDrive::drive(float y_speed, float x_speed, float turn) {
	float front_speed = 0;
	float back_speed = 0;
	float left_speed = 0;
	float right_speed = 0;
	turn = k*turn;
	float angle = 0;
	if (fabs(y_speed) <= minVal) {
		if (fabs(turn) <= minVal) {
			angle = gyro->GetAngle()/-60;
			SmartDashboard::PutNumber("angle", angle*-60);
		}
		y_speed = 0;
	}
	if (fabs(x_speed) <= minVal) {
		x_speed = 0;
	}
	if (fabs(turn) <= minVal) {
		turn = 0;
	}
	//motor controllers moving same direction
	if ((y_speed || x_speed) && turn) {
		left_speed = avg(y_speed, turn);
		right_speed = avg(y_speed, -turn);
		front_speed = avg(x_speed, turn);
		back_speed = avg(x_speed, -turn);
	}
	else if (y_speed || x_speed) {
		left_speed = y_speed;
		if (angle !=0) {
			left_speed = angle;
		}
		right_speed = y_speed;
		front_speed = x_speed;
		back_speed = x_speed;
	}
	else if (turn){
		left_speed = turn;
		right_speed = -turn;
		front_speed = turn;
		back_speed = -turn;
	}

	fr_motor->Set(front_speed);
	fl_motor->Set(front_speed);
	mfl_motor->Set(left_speed);
	mbl_motor->Set(left_speed);
	mfr_motor->Set(right_speed);
	mbr_motor->Set(right_speed);
	back_motor->Set(back_speed);

	SmartDashboard::PutNumber("front speed", front_speed);
	SmartDashboard::PutNumber("back speed", back_speed);
	SmartDashboard::PutNumber("left speed", left_speed);
	SmartDashboard::PutNumber("right speed", right_speed);

};


