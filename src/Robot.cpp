#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Lib830.h>
#include <WPIlib.h>
#include "GripPipeline.h"
#include <thread>
#include <cmath>

using namespace Lib830;
using namespace std;


class Robot: public frc::IterativeRobot {
private:
	/*frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected; */

	enum AutoMode {NOTHING, CENTER, LEFT, RIGHT};

public:
	static const int FRONT_LEFT_PWM = 0;
	static const int BACK_LEFT_PWM = 1;
	static const int FRONT_RIGHT_PWM = 2;
	static const int BACK_RIGHT_PWM = 3;

	static const int RED_LED_DIO = 0;
	static const int GREEN_LED_DIO = 1;
	static const int BLUE_LED_DIO = 2;
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

	MecanumDrive *drive;
	Lib830::GamepadF310 * pilot;
	frc::AnalogGyro *gyro;

	VictorSP fl;
	VictorSP bl;
	VictorSP fr;
	VictorSP br;
	Timer timer;
	static const int DIO_RED = 0;
	static const int DIO_GREEN = 1;
	static const int DIO_BLUE = 2;

	Lib830::DigitalLED *led;

	static const int TEST_PWM = 5;

	VictorSP * test;

	SendableChooser<AutoMode*> *chooser;

	AnalogPotentiometer *pot;
	PIDController *pid;




	Robot() :IterativeRobot(), fl(FRONT_LEFT_PWM), bl(BACK_LEFT_PWM), fr(FRONT_RIGHT_PWM), br(BACK_RIGHT_PWM) {}

	static Toggle vision;
	static void CameraPeriodic() {
		CameraServer *server;
		grip::GripPipeline * pipeline;

		pipeline = new grip::GripPipeline();
		cs::UsbCamera camera;
		cv::Mat image;
		cv::Mat temp_image;
		bool g_frame = false;

		cs::CvSink sink;
		cs::CvSource outputStream;

		server = CameraServer::GetInstance();

		camera = server->StartAutomaticCapture();
		camera.SetResolution(320,240);
		sink = server->GetVideo();
		outputStream = server->PutVideo("Processed", 320, 240);

		bool setExposure = true;

		while(1) {

			bool working = sink.GrabFrame(temp_image);
			SmartDashboard::PutBoolean("working", working);
			if (working) {
				g_frame = true;
				image = temp_image;
			}
			if (!g_frame) {
				continue;
			}
			if (vision) {
				if (setExposure) {
					camera.SetExposureManual(30);
					setExposure = false;
				}
				pipeline->Process(image);
			}
			else {
				if (!setExposure) {
					camera.SetExposureAuto();
					setExposure = true;
				}

			}


			//outputStream.PutFrame(*pipeline->GetHslThresholdOutput());

			outputStream.PutFrame(image);
		}

	}



	float StrafeVisionCorrect(){
		float midx = SmartDashboard::GetNumber("mid point x", 160);
		return (midx-160)/260;
	}

	void RobotInit() {
		/*chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser); */

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


		drive = new MecanumDrive (
				fl,
				bl,
				fr,
				br
		);

		pilot = new Lib830::GamepadF310(0);
		gyro->Calibrate();
		gyro->Reset();

		/*SmartDashboard::PutNumber("hue min", 60);
		SmartDashboard::PutNumber("hue max", 108);
		SmartDashboard::PutNumber("sat min", 131);
		SmartDashboard::PutNumber("sat max", 255);
		SmartDashboard::PutNumber("lum min", 140);
		SmartDashboard::PutNumber("lum max", 220);*/

		led = new DigitalLED(new DigitalOutput(RED_LED_DIO), new DigitalOutput(GREEN_LED_DIO), new DigitalOutput(BLUE_LED_DIO));
		std::thread visionThread(CameraPeriodic);
		visionThread.detach();

		//led = new DigitalLED( new DigitalOutput(DIO_RED), new DigitalOutput(DIO_GREEN), new DigitalOutput(DIO_BLUE));
		pilot = new GamepadF310(0);

		test = new VictorSP(TEST_PWM);

		chooser = new SendableChooser<AutoMode*>;

		chooser->AddObject("Left", new AutoMode(LEFT));
		chooser->AddObject("Right", new AutoMode(RIGHT));
		chooser->AddObject("Center", new AutoMode(CENTER));
		chooser->AddDefault("Nothing", new AutoMode(NOTHING));

		SmartDashboard::PutData(chooser);

		timer.Reset();
		timer.Start();

		pot = new AnalogPotentiometer(1, 270, -135);
		pid = new PIDController(6.0f, 2, 0, pot, test);
		pid->SetInputRange(-135,135);
		pid->SetOutputRange(-1.0, 1.0);
		pid->SetAbsoluteTolerance(5);
		pid->Enable();

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
	bool acquired = false;

	void AutonomousInit() override {

		/*autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}*/
		timer.Reset();
		timer.Start();

		gyro->Reset();
		acquired = false;
		vision = true;

	}

	float getXSpeed (bool &acquired_once, float target_speed, float default_speed) {
		if (target_speed) {
			if(!acquired_once) {
				acquired_once = true;
			}
			return target_speed;
		}
		else {
			if (acquired_once) {
				return 0;
			}
			else {
				return default_speed;
			}
		}
	}

	float prev_y_speed = 0;
	float prev_x_speed = 0;

	void AutonomousPeriodic() {
		string message = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		SmartDashboard::PutString("message", message);
		char mes = message[0];
		cout << "first character: " << mes << endl;

		AutoMode mode = NOTHING;
		if (chooser->GetSelected()) {
			mode = *chooser->GetSelected();
		}

		float x_speed = 0;
		float y_speed = 0;
		float angle = gyro->GetAngle();
		float rot = angle/-60;


		float time = timer.Get();

		if (time < 2) {
			y_speed = 0.5;
		}

		else if (time > 2 && time < 8) {
			y_speed = 0.3;
			if (acquired) {
				y_speed = 0.5;
			}
			switch(mode) {
			case CENTER:
				if (mes == 'L') {
					x_speed = getXSpeed(acquired, StrafeVisionCorrect(), -0.3);
				}
				else if (mes == 'R') {
					x_speed = getXSpeed(acquired, StrafeVisionCorrect(), 0.3);
				}
				break;
			case RIGHT:
				if (mes == 'L') {
					x_speed = 0;
				}
				else if (mes == 'R') {
					x_speed = getXSpeed(acquired, StrafeVisionCorrect(), 0);
				}
				break;
			case LEFT:
				if (mes == 'L') {
					x_speed = getXSpeed(acquired, StrafeVisionCorrect(), 0.2);
				}
				else if (mes == 'R') {
					x_speed = 0;
				}
				break;
			default:
				x_speed = 0;
				y_speed = 0;
				rot = 0;
				break;
			}
		}

		float f_x_speed = accel(prev_x_speed, x_speed, TICKS_TO_ACCEL);
		float f_y_speed = accel(prev_y_speed, y_speed, TICKS_TO_ACCEL);

		drive->DriveCartesian(f_x_speed, f_y_speed, rot, angle);

		SmartDashboard::PutNumber("x speed", f_x_speed);
		SmartDashboard::PutNumber("y speed", f_y_speed);
		SmartDashboard::PutNumber("rot", rot);
		SmartDashboard::PutNumber("angle", angle);
		SmartDashboard::PutBoolean("acquired", acquired);

		prev_x_speed = f_x_speed;
		prev_y_speed = f_y_speed;

	}

	void TeleopInit() {
		vision = false;
	}


	float prev_turn = 0;

	Toggle field_orient;

	float value(float input) {
		if (fabs(input) < 0.13) {
			return 0;
		}
		else {
			return input;
		}
	}

	void TeleopPeriodic() override {
		float y_speed = Lib830::accel(prev_y_speed, value(pilot->LeftY()), TICKS_TO_ACCEL);
		float x_speed = Lib830::accel(prev_x_speed, value(pilot->LeftX()), TICKS_TO_ACCEL);
		float turn =  Lib830::accel(prev_turn, value(pilot->RightX()), TICKS_TO_ACCEL);
		float gyro_read = 0;

		if (field_orient.toggle(pilot->ButtonState(GamepadF310::BUTTON_X))){
			gyro_read = gyro->GetAngle();
		}


		drive->DriveCartesian(x_speed, y_speed, turn, gyro_read);

		prev_y_speed = y_speed;
		prev_x_speed = x_speed;
		prev_turn = turn;

		SmartDashboard::PutNumber("gyro read", gyro_read);
		SmartDashboard::PutBoolean("field orident", field_orient);
		SmartDashboard::PutNumber("Left y", value(pilot->LeftY()));
		SmartDashboard::PutNumber("actual left y", pilot->LeftY());


		if (pilot->ButtonState(GamepadF310::BUTTON_A)){
			SmartDashboard::PutNumber("Strafe speed", StrafeVisionCorrect());
		}

		//led->Set(pilot->LeftTrigger(), pilot->RightTrigger(), pilot->LeftY());


		//test->Set(pilot->RightY());

		up.toggle(pilot->ButtonState(GamepadF310::BUTTON_A));
		down.toggle(pilot->ButtonState(GamepadF310::BUTTON_Y));
		vector<float> setPoints = {0, 31.5, -90.1, 23.6, 20.0};


		armMove(up, down, pos);

		pid->SetSetpoint(setPoints[pos]);

		SmartDashboard::PutNumber("pos", pos);
		SmartDashboard::PutNumber("potentiometer", pot->Get());
		SmartDashboard::PutNumber("set point", setPoints[pos]);
		SmartDashboard::PutNumber("test speed", test->Get());

	}

	void TestPeriodic() {
		//lw->Run();
	}
	void DisabledPeriodic() {
		drive->DriveCartesian(0,0,0);
		//led->Disable();
		//led->Alternate({1,0.7,0}, {0,0,1});


	}
//#define PI 3.141592

	int pos = 0;
	Toggle up;
	Toggle down;

	void armMove(Toggle &up, Toggle &down, int &position) {
		if (up) {
			if (position < 4) {
				position++;
			}
			up = false;
		}
		else if (down) {
			if (position > 0) {
				position--;
			}
			down = false;
		}
	}

	void RobotPeriodic() {
		vision.toggle(pilot->ButtonState(GamepadF310::BUTTON_B));
		//led->Set(0,1,0);
		//DigitalLED::Color cyan = {0, 0.4, 1};

		led->RainbowFade(10);
		//cout << "vision correct: " << vision << endl;

		//up.toggle(pilot->ButtonState(GamepadF310::BUTTON_A));
		//down.toggle(pilot->ButtonState(GamepadF310::BUTTON_X));


	}

};

Toggle Robot::vision(true);

START_ROBOT_CLASS(Robot)

