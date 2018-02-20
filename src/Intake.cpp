/*
 * Intake.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: RatPack
 */

#include <Intake.h>

Intake::Intake(VictorSP *LeftMotor, VictorSP *RightMotor):leftMotor(LeftMotor), rightMotor(RightMotor) {
	timer.Reset();
	intakeMode = NOTHING;
}

Intake::~Intake() {}

void Intake::toIntake() {
	timer.Reset();
	timer.Start();
	intakeMode = INTAKE;
}

void Intake::toOutput() {
	timer.Reset();
	timer.Start();
	intakeMode = OUTPUT;
}

void Intake::stop() {
	intakeMode = NOTHING;
}

void Intake::update() {
	float intakeSpeed = 1.0;
	float outputSpeed = -1.0;
	float time = timer.Get();
	if (time > 0.25) {
		intakeMode = NOTHING;
	}
	switch (intakeMode) {
	case INTAKE:
		leftMotor->Set(intakeSpeed);
		rightMotor->Set(intakeSpeed);
		break;
	case OUTPUT:
		leftMotor->Set(outputSpeed);
		rightMotor->Set(outputSpeed);
		break;
	default:
		leftMotor->Set(0);
		rightMotor->Set(0);
	};
}
