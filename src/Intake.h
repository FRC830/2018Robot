/*
 * Intake.h
 *
 *  Created on: Feb 4, 2018
 *      Author: RatPack
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "Lib830.h"

class Intake {
public:
	Intake(VictorSP *LeftMotor, VictorSP *RightMotor);
	virtual ~Intake();
	void toIntake();
	void toOutput();
	void update();
	void stop();
	Timer timer;
private:
	enum IntakeMode {INTAKE, OUTPUT, NOTHING};
	IntakeMode intakeMode;
	VictorSP *leftMotor;
	VictorSP *rightMotor;
};


#endif /* SRC_INTAKE_H_ */
