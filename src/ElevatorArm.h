/*
 * ElevatorArm.h
 *
 *  Created on: Mar 6, 2018
 *      Author: RatPack
 */

#ifndef ELEVATORARM_H_
#define ELEVATORARM_H_

#include "WPILib.h"
#include "Lib830.h"

class ElevatorArm {
public:
	ElevatorArm(VictorSP *armMotor, Encoder *armEncoder);
	void toDown();
	void toIntake();
	void toSwitch();
	void toScale();
	void toManual(float speed);
	void reset();

	void armMove(Toggle &button_up, Toggle &button_down); //might not use, depends on encoder
	void update();

	virtual ~ElevatorArm();
private:
	VictorSP *armMotor;
	Encoder *armEncoder;
	PIDController *armPID;
	float p = 0.1f;
	float i = 0;
	float d = 0;

	float prev_speed;
	std::vector<float> setPoints = {0, 100, 200, 400, 1000, 1200};
	enum State {NOTHING, DOWN, INTAKE, SWITCH, SCALE, SCALE_HIGH};
	State state;

};

#endif /* ELEVATORARM_H_ */
