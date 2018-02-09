/*
 * Arm.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: RatPack
 */

#include "Arm.h"

Arm::Arm(VictorSP *armMotor, AnalogPotentiometer *pot):armMotor(armMotor), pot(pot) {
	p = 0.1f;
	i = 0;
	d = 0;
	pos = DOWN;
	state = PID;
	speed = 0;

	pid = new PIDController(p, i, d, pot, armMotor);
	pid->SetInputRange(-135,135); //subject to change
	pid->SetOutputRange(-0.3, 0.3);
	pid->SetAbsoluteTolerance(5);
	pid->SetPID(p,i,d);
	pid->SetSetpoint(setPoints[pos]);
	pid->Enable();

	up = false;
	down = false;
	toManual = false;
	toAutomatic = false;

}

int Arm::getClosestPos(std::vector<double> class_vector, float input) {
	std::vector<float> difference;
	for (int i = 0; i < 6; i++) {
		float diff = fabs(class_vector[i] - input);
		difference.push_back(diff);
	}
	int index = std::distance(difference.begin(), std::min_element(difference.begin(), difference.end()));
	return index;
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

void Arm::automaticPosition(Toggle &button_up, Toggle &button_down) {
	up = button_up;
	down = button_down;
	state = PID;
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
	if (toAutomatic) {
		pos = getClosestPos(setPoints, setPoints[6]);
		toAutomatic = false;
	}
}

void Arm::manualPosition(float button_up, float button_down) {
	float manual_pos = 0;
	float change = button_up - button_down;
	state = PID;
	if (!toManual) {
		manual_pos = setPoints[pos];
		toManual = true;
	}

	if (manual_pos > 0 && manual_pos < 135) {
		manual_pos = manual_pos + (change * 1); //subject to change
		setPoints[6] = manual_pos;
	}

	pos = MANUAL;
}

void Arm::rawPosition(float speed){
	this->speed=speed;
	state = RAW;
}


void Arm::armMoveUpdate() {
	pid->SetEnabled(state == PID);
	if (state == RAW){
		armMotor->Set(speed);
	}
	else {
		double position = setPoints[pos];
		pid->SetSetpoint(position);

		if (pos == 6) {
			toManual = false;
			toAutomatic = true;
		}
		SmartDashboard::PutNumber("position", pos);
		SmartDashboard::PutNumber("set point", position);
		SmartDashboard::PutNumber("potentiometer", pot->Get());
		SmartDashboard::PutBoolean("to manual", toManual);
		SmartDashboard::PutNumber("state", state); // 0 or 1
	}
}

float Arm::getPosition() {
	return pot->Get();
}

float Arm::getSetPoint() {
	return setPoints[pos];
}


Arm::~Arm() {

}
