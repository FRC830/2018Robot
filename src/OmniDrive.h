/*
 * OmniDrive.h
 *
 *  Created on: Jan 9, 2018
 *      Author: RatPack
 */

#ifndef SRC_OMNIDRIVE_H_
#define SRC_OMNIDRIVE_H_

#include "WPILib.h"

class OmniDrive {
public:
	OmniDrive(VictorSP *fr, VictorSP *fl, VictorSP *mfl, VictorSP *mbl, VictorSP *mfr, VictorSP *mbr, VictorSP *back);
	float avg(float left_value, float right_value);
	void drive(float y_speed, float x_speed, float turn);

private:
	VictorSP *fr_motor;
	VictorSP* fl_motor;
	VictorSP *mfl_motor;
	VictorSP *mbl_motor;
	VictorSP *mfr_motor;
	VictorSP *mbr_motor;
	VictorSP *back_motor;

};



#endif /* SRC_OMNIDRIVE_H_ */
