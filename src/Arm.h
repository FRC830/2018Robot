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
	void manualPosition(float button_up, float button_down); //not done


	void disablePID();
	float getPosition();
	float getSetPoint();
	void armMoveUpdate();
	virtual ~Arm();

private:
	VictorSP *armMotor;
	AnalogPotentiometer *pot;
	PIDController *pid;
	bool up;
	bool down;

	bool toManual;
	bool toAutomatic;
	int pos;
	float p,i,d;

	std::vector<double> setPoints = {0,10,20,30,40,50,0};
	enum setPoint {DOWN, INTAKE, SWITCH, SCALE_LOW, SCALE_MID, SCALE_HIGH, MANUAL };
	enum State {MANUAL_STATE, AUTOMATIC};
	State state;

	int getClosestPos(std::vector<double> class_vector, float input);
	//random numbers atm
};


#endif /* SRC_ARM_H_ */
