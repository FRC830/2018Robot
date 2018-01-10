/*
 * OmniDrive.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: RatPack
 */

#include "OmniDrive.h"

OmniDrive::OmniDrive(VictorSP *fr, VictorSP *fl, VictorSP *mfl, VictorSP *mbl, VictorSP *mfr, VictorSP *mbr, VictorSP *back) {
	fr_motor = fr;
	fl_motor = fl;
	mfl_motor = mfl;
	mbl_motor = mbl;
	mfr_motor = mfr;
	mbr_motor = mbr;
	back_motor = back;
};

float OmniDrive::avg(float left_value, float right_value) {
	return (left_value + right_value)/2;
};

void OmniDrive::drive(float y_speed, float x_speed, float turn) {

};


