/*
 * EncoderDrive.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: RatPack
 */

#include "EncoderDrive.h"
#include <iostream>
using namespace std;

void EncoderDrive::initalizePIDController(PIDController *PID, SuperEncoder *input, VictorSP *output) {
	input->SetDistancePerPulse(1.0/1024.0);
	PID->SetAbsoluteTolerance(1);
	PID->SetInputRange(-8,8);
	PID->SetOutputRange(-1.0,1.0);
	PID->SetSetpoint(0);
	PID->Enable();

}
EncoderDrive::EncoderDrive(SuperEncoder  *front_left_enc, SuperEncoder  *back_left_enc, SuperEncoder  *front_right_enc, SuperEncoder  *back_right_enc,
		VictorSP *fl, VictorSP *bl, VictorSP *fr, VictorSP *br): MecanumDrive(*fl, *bl, *fr, *br), front_left_enc(front_left_enc), back_left_enc(back_left_enc),
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

double EncoderDrive::signalToRPS(double signal) {
	return (signal * 6.5) * 0.7;
}

void EncoderDrive::setSetpoint(double signal, PIDController *encoderPID) {
	encoderPID->SetSetpoint(signalToRPS(signal));
}

void EncoderDrive::DriveCarties(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {


	  if (!reported) {
	    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
	               HALUsageReporting::kRobotDrive_MecanumCartesian);
	    reported = true;
	  }
	ySpeed = Limit(ySpeed);
	ySpeed = ApplyDeadband(ySpeed, m_deadband);

	xSpeed = Limit(xSpeed);
	xSpeed = ApplyDeadband(xSpeed, m_deadband);

	// Compensate for gyro angle.
	Vector2d input{ySpeed, xSpeed};
	input.Rotate(-gyroAngle);

	double wheelSpeeds[4];
	wheelSpeeds[kFrontLeft] = input.x + input.y + zRotation;
	wheelSpeeds[kFrontRight] = input.x - input.y + zRotation;
	wheelSpeeds[kRearLeft] = -input.x + input.y + zRotation;
	wheelSpeeds[kRearRight] = -input.x - input.y + zRotation;


	SmartDashboard::PutNumber("fl motor", flPID->Get());
	SmartDashboard::PutNumber("raw fl motor", fl->Get());
	SmartDashboard::PutNumber("bl motor", blPID->Get());
	SmartDashboard::PutNumber("fr motor", frPID->Get());
	SmartDashboard::PutNumber("br motor", brPID->Get());

	SmartDashboard::PutNumber("fl motor read", wheelSpeeds[kFrontLeft]);
	SmartDashboard::PutNumber("bl motor read",  wheelSpeeds[kRearLeft]);
	SmartDashboard::PutNumber("fr motor read",  wheelSpeeds[kFrontRight]);
	SmartDashboard::PutNumber("br motor read",  wheelSpeeds[kRearRight]);


	flPID->SetSetpoint(signalToRPS(wheelSpeeds[kFrontLeft]));
	blPID->SetSetpoint(signalToRPS(wheelSpeeds[kRearLeft]));
	frPID->SetSetpoint(signalToRPS(wheelSpeeds[kFrontRight]));
	brPID->SetSetpoint(signalToRPS(wheelSpeeds[kRearRight]));

//	setSetpoint(wheelSpeeds[kFrontLeft], flPID);
//	setSetpoint(wheelSpeeds[kFrontRight], frPID);
//	setSetpoint(wheelSpeeds[kRearLeft], blPID);
//	setSetpoint(wheelSpeeds[kRearRight], brPID);

//	fl->Set(wheelSpeeds[kFrontLeft]);
//	bl->Set(wheelSpeeds[kRearLeft]);
//	fr->Set(wheelSpeeds[kFrontRight]);
//	br->Set(wheelSpeeds[kRearRight]);




	SmartDashboard::PutNumber("fl setpoint", flPID->GetSetpoint());

	Normalize(wheelSpeeds);

	m_safetyHelper.Feed();
}




EncoderDrive::~EncoderDrive() {
	// TODO Auto-generated destructor stub
}

