/*
 * ElevatorArm.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: RatPack
 */

#include "ElevatorArm.h"

ElevatorArm::ElevatorArm(VictorSP *armMotor, Encoder *armEncoder /*digital input bottom switch??*/): armMotor(armMotor), armEncoder(armEncoder) {
	// TODO Auto-generated constructor stub
	armPID = new PIDController(p,i,d, armEncoder, armMotor);
	armPID->SetInputRange(-100,1000); //subject to change
	armPID->SetOutputRange(-0.5, 0.7); //to change
	armPID->SetAbsoluteTolerance(20);
	armEncoder->SetPIDSourceType(PIDSourceType::kDisplacement);
	armPID->Enable();
	prev_speed = 0;
	state = NOTHING;

}

void ElevatorArm::reset() {
	armEncoder->Reset();
}
void ElevatorArm::toDown() {
	state = DOWN;
}

void ElevatorArm::toIntake() {
	state = INTAKE; //may not be necessary
}

void ElevatorArm::toSwitch() {
	state = SWITCH;
}

void ElevatorArm::toScale() {
	state = SCALE;
}

void ElevatorArm::toManual(float input) {
	 float cur_speed = Lib830::accel(prev_speed, input, 15);
	 prev_speed = cur_speed;
}

void ElevatorArm::armMove(Toggle &button_up, Toggle &button_down) {

}

ElevatorArm::~ElevatorArm() {
	// TODO Auto-generated destructor stub
}

