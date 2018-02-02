/*
 * Arm.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: RatPack
 */

#include "Arm.h"

Arm::Arm(VictorSP *armMotor, AnalogPotentiometer *pot):armMotor(armMotor), pot(pot) {
	pid = new PIDController(0.1, 0, 0, pot, armMotor);
	pid->SetInputRange(-135,135); //subject to change
	pid->SetOutputRange(-0.3, 0.3);
	pid->SetAbsoluteTolerance(5);
	pid->Enable();
	up = false;
	down = false;
	p = 0.1f;
	i = 0;
	d = 0;
	pid->SetPID(p,i,d);
}

void Arm::armPosition(Toggle &up, Toggle &down, int &position) {
	if (up) {
		if (position < 4) {
			position++;
		}
		up = false;
	}
	else if (down) {
		if (position > 0) {
			position--;
		}
		down = false;
	}
}

void Arm::armMove(double position) {
	pid->SetSetpoint(position);
}

void Arm::update(bool button_up, bool button_down) {
	up.toggle(button_up);
	down.toggle(button_down);

	armPosition(up, down, pos);
	armMove(setPoints[pos]);

}

void Arm::disablePID() {
	pid->Disable();
}
Arm::~Arm() {

}










