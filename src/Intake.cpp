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

void Intake::toSlowOutput() {
	timer.Reset();
	timer.Start();
	intakeMode = SLOW_OUTPUT;
}

void Intake::toAdjust() {
	intakeMode = ADJUST;
}

void Intake::stop() {
	intakeMode = NOTHING;
}

//void Intake::toAdjust()

void Intake::update() {
	float intakeSpeed = 1.0;
	float outputSpeed = -0.75;
	float time = timer.Get();
	if (time > 0.25) {
		intakeMode = NOTHING;
	}
	switch (intakeMode) {
	case ADJUST:
		leftMotor->Set(intakeSpeed);
		rightMotor->Set(intakeSpeed/2);
		break;
	case INTAKE:
		leftMotor->Set(intakeSpeed);
		rightMotor->Set(intakeSpeed);
		break;
	case OUTPUT:
		leftMotor->Set(outputSpeed);
		rightMotor->Set(outputSpeed);
		break;
	case SLOW_OUTPUT:
		leftMotor->Set(outputSpeed/4);
		rightMotor->Set(outputSpeed/4);
		break;
	default:
		leftMotor->Set(0);
		rightMotor->Set(0);
	};

	SmartDashboard::PutNumber("intake output speed", leftMotor->Get());
	SmartDashboard::PutNumber("intake mode", intakeMode);
}
