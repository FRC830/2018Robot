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
	void teleopArmPosition(Toggle &button_up, Toggle &button_down);
	void manualPosition(float manual_pos); //not done


	void disablePID();
	void armMoveUpdate();
	virtual ~Arm();

private:
	VictorSP *armMotor;
	AnalogPotentiometer *pot;
	PIDController *pid;
	bool up;
	bool down;
	int pos = 0;
	float p,i,d;

	std::vector<double> setPoints = {0,10,20,30,40,50,0};
	enum setPoint {DOWN, INTAKE, SWITCH, SCALE_LOW, SCALE_MID, SCALE_HIGH, MANUAL };
	setPoint setpoint;
	//random numbers atm
};


#endif /* SRC_ARM_H_ */
