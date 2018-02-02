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
	setpoint = DOWN;
}

void Arm::toDown() {
	pos = DOWN;
}
void Arm::toSwitch() {
	pos = SWITCH;
}
void Arm::toScale() {
	pos = SCALE_MID;
}

void Arm::teleopArmPosition(Toggle &button_up, Toggle &button_down) {
	up = button_up;
	down = button_down;
	if (up) {
		if (pos < 5) {
			pos++;
		}
		button_up = false;
	}
	else if (down) {
		if (pos > 0) {
			pos--;
		}
		button_down = false;
	}
}

void Arm::manualPosition(float manual_pos) {
	manual_pos *= 135; //to change
	setPoints[6] = manual_pos;
	pos = MANUAL;
}

void Arm::disablePID() {
	pid->Disable();
}

void Arm::armMoveUpdate() {
	double position = setPoints[pos];
	pid->SetSetpoint(position);
}


Arm::~Arm() {

}










