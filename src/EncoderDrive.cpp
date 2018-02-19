/*
 * EncoderDrive.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: RatPack
 */

#include "EncoderDrive.h"
#include <iostream>
using namespace std;

void EncoderDrive::initalizePIDController(PIDController *PID, Encoder *input, VictorSP *output) {
	input->SetDistancePerPulse((6*3.1416)/1024.0);
	input->SetPIDSourceType(PIDSourceType::kDisplacement);
	PID->SetAbsoluteTolerance(0.5);
	PID->SetInputRange(-100000,1000000000);
	PID->SetOutputRange(-1.0,1.0);
	PID->SetPID(0.1f,0,0);
	//PID->SetSetpoint(0);
	//PID->Enable();

}
EncoderDrive::EncoderDrive(Encoder  *front_left_enc, Encoder  *back_left_enc, Encoder  *front_right_enc, Encoder  *back_right_enc,
		VictorSP *fl, VictorSP *bl, VictorSP *fr, VictorSP *br): MecanumDrive(*fl, *br, *fr, *br),
		 front_right_enc(front_right_enc), back_right_enc(back_right_enc), fl(fl),bl(bl),fr(fr), br(br) {
	flPID = new PIDController(0.1, 0,0, front_left_enc, fl);
	blPID = new PIDController(0.1, 0,0, back_left_enc, bl);
	frPID = new PIDController(0.1, 0,0, front_right_enc, fr);
	brPID = new PIDController(0.1, 0,0, back_right_enc, br);
	initalizePIDController(flPID, front_left_enc, fl);
	initalizePIDController(blPID, back_left_enc, bl);
	initalizePIDController(frPID, front_right_enc, fr);
	initalizePIDController(brPID, back_right_enc, br);

	flPID->SetName("fl PID");
	SmartDashboard::PutData(flPID);
	flPID->SetName("bl PID");
	SmartDashboard::PutData(blPID);
	flPID->SetName("fr PID");
	SmartDashboard::PutData(frPID);
	flPID->SetName("br PID");
	SmartDashboard::PutData(brPID);

	// TODO Auto-generated constructor stub

}

void EncoderDrive::DistanceDrive(float distance) {
	float ticks = distanceToTicks(distance);
	setSetpoint(ticks, flPID);
	setSetpoint(ticks, blPID);
	setSetpoint(ticks, frPID);
	setSetpoint(ticks, brPID);

}
double EncoderDrive::distanceToTicks(double distance) {
	return distance / front_left_enc->GetDistancePerPulse();
}

void EncoderDrive::setSetpoint(double distance, PIDController *encoderPID) {
	encoderPID->SetSetpoint(distanceToTicks(distance));
}



EncoderDrive::~EncoderDrive() {
	// TODO Auto-generated destructor stub
}

