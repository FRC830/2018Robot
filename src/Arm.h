/*
 * Arm.h
 *
 *  Created on: Feb 1, 2018
 *      Author: RatPack
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include "Lib830.h"
#include "WPILib.h"


class Arm{
public:
	Arm(VictorSP *armMotor, AnalogPotentiometer *pot);

	void toDown();
	void toSwitch();
	void toScale(); //for middle
	void automaticPosition(Toggle &button_up, Toggle &button_down);
	void manualPosition(float button_up, float button_down); //not done
	void rawPosition(float speed);

	float getRawPosition();
	float getSetPoint();
	void armMoveUpdate();
	virtual ~Arm();

private:
	VictorSP *armMotor;
	AnalogPotentiometer *pot;
	PIDController *pid;
	bool up;
	bool down;
	float speed;
	float prev_speed = 0;
	static const int TICKS_TO_ACCEL = 15;

	bool toManual;
	bool toAutomatic;
	int pos;
	float p,i,d;

	std::vector<double> setPoints = {55, 65,120,180,200,210, 0};
	enum setPoint {DOWN, INTAKE, SWITCH, SCALE_LOW, SCALE_MID, SCALE_HIGH, MANUAL };
	enum State {RAW, PID};
	State state;

	int getClosestPos(std::vector<double> class_vector, float input);
	//random numbers atm
};



#endif /* SRC_ARM_H_ */
