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
	void update(bool button_up, bool button_down);
	void disablePID();
	void manualShoot(float manual_pos); //not done
	virtual ~Arm();

private:
	VictorSP *armMotor;
	AnalogPotentiometer *pot;
	PIDController *pid;
	Toggle up;
	Toggle down;
	int pos = 0;
	float p,i,d;

	void armPosition(Toggle &up, Toggle &down, int &position);
	void armMove(double position);

	std::vector<double> setPoints = {0,20,30,40,125};
};


#endif /* SRC_ARM_H_ */
