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
	timer.Start();
	intakeMode = INTAKE;
}

void Intake::toOutput() {
	timer.Start();
	intakeMode = OUTPUT;
}

void Intake::update() {
	float intakeSpeed = 0.5;
	float outputSpeed = -0.5;
	float time = timer.Get();
	if (time > 2.0) {
		intakeMode = NOTHING;
	}
	switch (intakeMode) {
	case INTAKE:
		leftMotor->Set(intakeSpeed);
		rightMotor->Set(-intakeSpeed);
		break;
	case OUTPUT:
		leftMotor->Set(outputSpeed);
		rightMotor->Set(-outputSpeed);
		break;
	default:
		leftMotor->Set(0);
		rightMotor->Set(0);
		timer.Reset();
	};
}
