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

class DriveController : public PIDSource, public PIDOutput {
public :
	std::atomic<double> ticks;
	std::atomic<double> speed;
	double PIDGet(){
		return ticks;
	}
	void PIDWrite(double speed){
		this->speed = speed;
	}
};

class EncoderDrive : public MecanumDrive {
public:
	EncoderDrive(
			Encoder *front_left_enc,
			Encoder *back_left_enc,
			Encoder *front_right_enc,
			Encoder *back_right_enc,
			VictorSP *fl,
			VictorSP *bl,
			VictorSP *fr,
			VictorSP *br
	);

	void DriveCartiesian(double x, double y, double z, double field_orient = 0.0);

	virtual ~EncoderDrive();

private:


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

	double speedToRPS(float speed);




};

#endif /* ENCODERDRIVE_H_ */
