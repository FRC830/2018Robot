/*
 * EncoderDrive.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: RatPack
 */

#include "EncoderDrive.h"

EncoderDrive::EncoderDrive(Encoder *front_left_enc,Encoder *back_left_enc,Encoder *front_right_enc,
		Encoder *back_right_enc,
		VictorSP *fl,
		VictorSP *bl,
		VictorSP *fr,
		VictorSP *br):front_left_enc(front_left_enc), back_left_enc(back_left_enc),
		 front_right_enc(front_right_enc), back_right_enc(back_right_enc), fl(fl),bl(bl),fr(fr), br(br) {


	// TODO Auto-generated constructor stub

}

EncoderDrive::~EncoderDrive() {
	// TODO Auto-generated destructor stub
}

