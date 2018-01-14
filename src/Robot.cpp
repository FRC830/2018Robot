#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Lib830.h>
#include <WPIlib.h>
#include <OmniDrive.h>

using namespace std;

class Robot: public frc::IterativeRobot {
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

public:

	static const int FRONT_LEFT_PWM;
	static const int BACK_LEFT_PWM;
	static const int FRONT_RIGHT_PWM;
	static const int BACK_RIGHT_PWM;
	/*static const int FL_PWM = 0;
	static const int FR_PWM = 1;
	static const int MFL_PWM = 2;
	static const int MBL_PWM = 3;
	static const int MFR_PWM = 4;
	static const int MBR_PWM = 5;
	static const int BACK_PWM = 6; */

	static const int ANLOG_GYRO = 0;

	static const int TICKS_TO_ACCEL = 10;

	//OmniDrive *drive;

	RobotDrive *drive;
	Lib830::GamepadF310 * pilot;\
	frc::AnalogGyro *gyro;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		gyro = 	new frc::AnalogGyro(ANLOG_GYRO);

		/*drive = new OmniDrive(
				new VictorSP(FR_PWM),
				new VictorSP(FL_PWM),
				new VictorSP(MFL_PWM),
				new VictorSP(MBL_PWM),
				new VictorSP(MFR_PWM),
				new VictorSP(MBR_PWM),
				new VictorSP(BACK_PWM),
				gyro
			); */
		drive = new RobotDrive (
				new VictorSP(FRONT_LEFT_PWM),
				new VictorSP(BACK_LEFT_PWM),
				new VictorSP(FRONT_RIGHT_PWM),
				new VictorSP(BACK_RIGHT_PWM)
		);

		pilot = new Lib830::GamepadF310(0);
		gyro->Calibrate();
		gyro->Reset();

	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		string message = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		SmartDashboard::PutString("message", message);

		char mes = message[0];
		cout << "first character: " << mes << endl;

	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	float prev_y_speed = 0;
	float prev_x_speed = 0;
	float prev_turn = 0;

	void TeleopPeriodic() override {
		float y_speed = Lib830::accel(prev_y_speed, pilot->LeftY(), TICKS_TO_ACCEL);
		float x_speed = Lib830::accel(prev_x_speed, pilot->LeftX(), TICKS_TO_ACCEL);
		float turn =  Lib830::accel(prev_turn, pilot->RightX(), TICKS_TO_ACCEL);

		drive->MecanumDrive_Cartesian(x_speed, y_speed, turn, gyro->GetAngle());

		prev_y_speed = y_speed;
		prev_x_speed = x_speed;
		prev_turn = turn;


	}

	void TestPeriodic() {
		//lw->Run();
	}
	void DisabledPeriodic() {
		drive->MecanumDrive_Cartesian(0,0,0);
	}

};

START_ROBOT_CLASS(Robot)
