/*
 * EncoderDrive.h
 *
 *  Created on: Feb 16, 2018
 *      Author: RatPack
 */

#ifndef ENCODERDRIVE_H_
#define ENCODERDRIVE_H_

#include "WPILib.h"
#include "Lib830.h"


//class SuperEncoder : public Encoder {
//public:
//
//	SuperEncoder(int encoderA, int encoderB, bool reverseDirection = false, EncodingType encodingType = k4X)
//	:Encoder(encoderA, encoderB, reverseDirection, encodingType) {}
////	double GetRate() {
////		timer.Start();
////		float cur_time = timer.Get();
////		float cur_ticks = this->GetRaw();
////		float rate = ((cur_ticks - prev_ticks)/ (cur_time - prev_time)) / 1024.0;
////		prev_time = cur_time;
////		prev_ticks = cur_ticks;
////		SmartDashboard::PutNumber("get rate", rate);
////		return rate;
////	}
//	double PIDGet(){
//		return GetRate() * 2.5;
//	}
//
//private:
//	Timer timer;
//	float prev_ticks = 0;
//	float prev_time = 0;
//
//};

class EncoderDrive : public MecanumDrive {
public:
	EncoderDrive(
			Encoder  *front_left_enc,
			Encoder  *back_left_enc,
			Encoder  *front_right_enc,
			Encoder  *back_right_enc,
			VictorSP *fl,
			VictorSP *bl,
			VictorSP *fr,
			VictorSP *br
	);

	void DistanceDrive(float distance);

	virtual ~EncoderDrive();

private:

	bool reported = false;
	Encoder *front_left_enc;
	Encoder *back_left_enc;
	Encoder *front_right_enc;
	Encoder *back_right_enc;

	VictorSP *fl;
	VictorSP *bl;
	VictorSP *fr;
	VictorSP *br;

	PIDController *flPID;
	PIDController *blPID;
	PIDController *frPID;
	PIDController *brPID;

	//double signalToRPS(double signal);
	double distanceToTicks(double distance);
	void setSetpoint(double distance, PIDController *encoderPID);

	void initalizePIDController(PIDController *PID, Encoder *input, VictorSP *output);




};

#endif /* ENCODERDRIVE_H_ */
