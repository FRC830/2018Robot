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
#include "Arm.h"
#include "Intake.h"

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
	static const int FRONT_LEFT_PWM = 1; //practice
	static const int BACK_LEFT_PWM = 0; //practice
	static const int FRONT_RIGHT_PWM = 3; //practice
	static const int BACK_RIGHT_PWM = 2; //practice

	static const int LEFT_INTAKE_PWM = 5; //subject to choonge
	static const int RIGHT_INTAKE_PWM = 6;

	static const int ARM_PWM = 9;


	static const int RED_LED_DIO = 0;
	static const int GREEN_LED_DIO = 8;
	static const int BLUE_LED_DIO = 15;

	static const int ANLOG_GYRO = 0;
	static const int POTENTIOMETER_ANALOG = 1;

	static const int TICKS_TO_ACCEL = 15;

	static const int RED_RELAY = 0;
	static const int GREEN_RELAY = 1;
	static const int BLUE_RELAY = 2;


	//OmniDrive *drive;

	MecanumDrive *drive;
	Lib830::GamepadF310 * pilot;
	Lib830::GamepadF310 * copilot;

	frc::AnalogGyro *gyro;
	Relay redLED;
	Relay greenLED;
	Relay blueLED;

	VictorSP fl;
	VictorSP bl;
	VictorSP fr;
	VictorSP br;
	Timer timer;


	Lib830::DigitalLED *led;

	VictorSP * test;

	SendableChooser<AutoMode*> *chooser;

	//AnalogPotentiometer *pot;
	//PIDController *pid;

	Arm *arm;
	Intake *intake;

	DigitalLED *relayLED;



	Robot() :IterativeRobot(), fl(FRONT_LEFT_PWM), bl(BACK_LEFT_PWM), fr(FRONT_RIGHT_PWM), br(BACK_RIGHT_PWM), redLED(RED_RELAY, Relay::kForwardOnly), greenLED(GREEN_RELAY, Relay::kForwardOnly), blueLED(BLUE_RELAY, Relay::kForwardOnly) {}

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

		drive = new MecanumDrive (
				fl,
				bl,
				fr,
				br
		);

		pilot = new Lib830::GamepadF310(0);
		copilot = new Lib830::GamepadF310(1);

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

		//test = new VictorSP(TEST_PWM);

		chooser = new SendableChooser<AutoMode*>;

		chooser->AddObject("Left", new AutoMode(LEFT));
		chooser->AddObject("Right", new AutoMode(RIGHT));
		chooser->AddObject("Center", new AutoMode(CENTER));
		chooser->AddDefault("Nothing", new AutoMode(NOTHING));

		SmartDashboard::PutData(chooser);

		timer.Reset();
		timer.Start();

		/*pot = new AnalogPotentiometer(1, 270, -135);


		SmartDashboard::PutNumber("p",0.1);
		SmartDashboard::PutNumber("i",0);
		SmartDashboard::PutNumber("d",0);

		pid = new PIDController(0.1, 0, 0, pot, test);
		pid->SetInputRange(-135,135);
		pid->SetOutputRange(-0.3, 0.3);
		pid->SetAbsoluteTolerance(5);
		pid->Enable();8*/

		arm = new Arm(
			new VictorSP(ARM_PWM),
			new AnalogPotentiometer(POTENTIOMETER_ANALOG, 270, 0)
		);
		intake = new Intake(
			new VictorSP(LEFT_INTAKE_PWM),
			new VictorSP(RIGHT_INTAKE_PWM)
		);


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

		timer.Reset();
		timer.Start();

		gyro->Reset();
		acquired = false;
		vision = true;
		arm->toDown();

	}

	float getXSpeed (float target_speed, float default_speed) {
		if (target_speed) {
			if(!acquired) {
				acquired = true;
			}
			return target_speed;
		}
		else {
			if (acquired) {
				return 0;
			}
			else {
				return default_speed;
			}
		}
	}

	float prev_y_speed = 0;

	float prev_x_speed = 0;
	bool output_cube = false;

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
			arm->toSwitch();

			if (acquired) {
				y_speed = 0.5; //set speed to be slower
			}
			switch(mode) {
			case CENTER:
				output_cube = true;
				if (mes == 'L') {
					x_speed = getXSpeed(StrafeVisionCorrect(), -0.3);
				}
				else if (mes == 'R') {
					x_speed = getXSpeed(StrafeVisionCorrect(), 0.3);
				}
				break;
			case RIGHT:
				if (mes == 'L') {
					x_speed = 0;
				}
				else if (mes == 'R') {
					x_speed = getXSpeed(StrafeVisionCorrect(), 0);
					output_cube = true;
				}
				break;
			case LEFT:
				if (mes == 'L') {
					x_speed = getXSpeed(StrafeVisionCorrect(), 0.2);
					output_cube = true;
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
		else if (time > 8 && time < 12) {
			x_speed = 0;
			y_speed = 0;
			rot = 0;
			if(output_cube){
				intake->toOutput();
			}
		}

		float f_x_speed = accel(prev_x_speed, x_speed, TICKS_TO_ACCEL);
		float f_y_speed = accel(prev_y_speed, y_speed, TICKS_TO_ACCEL);

		drive->DriveCartesian(f_x_speed, f_y_speed, rot, angle);
		arm->armMoveUpdate();
		intake->update();

		SmartDashboard::PutNumber("x speed", f_x_speed);
		SmartDashboard::PutNumber("y speed", f_y_speed);
		SmartDashboard::PutNumber("rot", rot);
		SmartDashboard::PutNumber("angle", angle);
		SmartDashboard::PutBoolean("acquired", acquired);

		prev_x_speed = f_x_speed;
		prev_y_speed = f_y_speed;
		//comment

	}

	float change = 0;
	void TeleopInit() {
		vision = false;
		double p = SmartDashboard::GetNumber("p", 0.1);
		double i = SmartDashboard::GetNumber("i", 0);
		double d =SmartDashboard::GetNumber("d", 0);
		//pid->SetPID(p,i,d);
		gyro->Reset();
		change = 0;
		led->Disable();

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

	Toggle PID;
	float last_val = 0;
	void TeleopPeriodic() override {

		float angle = gyro->GetAngle() - change;
		//DigitalLED::Color color1 {DigitalLED::Lime};
		//DigitalLED::Color color2 {DigitalLED::Lime};
		float y_speed = Lib830::accel(prev_y_speed, value(pilot->LeftY()), TICKS_TO_ACCEL);
		float x_speed = Lib830::accel(prev_x_speed, value(pilot->LeftX()), TICKS_TO_ACCEL);
		//float turn =  Lib830::accel(prev_turn, value(pilot->RightX()), 5);
		float turn = value(pilot->RightX()/2);

		SmartDashboard::PutNumber("value turn", value(pilot->RightX()));
		float gyro_read = 0;

		if (field_orient.toggle(pilot->ButtonState(GamepadF310::BUTTON_X))){
			//color1 = DigitalLED::Magenta;
			led->Set(0,0,0);
			gyro_read = gyro->GetAngle();
		}
		else {
			led->Set(1,0,1);
		}

		if (!turn) {
			turn = angle/-50;
		}
		else  {
			change = gyro->GetAngle();
		}

		SmartDashboard::PutNumber("no reset gyro", gyro->GetAngle() - change);


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


		down.toggle(copilot->LeftTrigger());
		up.toggle(copilot->RightTrigger());
		if (PID.toggle(copilot->ButtonState(GamepadF310::BUTTON_START))){
			arm->rawPosition(copilot->RightTrigger()-copilot->LeftTrigger());
			//color2 = DigitalLED::Cyan;
		}
		else {
			if (copilot->ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER) || copilot->ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) ) {
				arm->manualPosition(copilot->RightTrigger(),copilot->LeftTrigger());
				//color2 = DigitalLED::Yellow;
			}
			else {
				arm->automaticPosition(up, down);
			}
		}
		arm->armMoveUpdate();

		if(copilot->ButtonState(GamepadF310::BUTTON_Y)){
			intake->toIntake();
		}
		else if(copilot->ButtonState(GamepadF310::BUTTON_X)){
			intake->toOutput();
		}

		intake->update();

		//led->Set(DigitalLED::AliceBlue);

	}

	void TestPeriodic() {
		//lw->Run();
	}
	void DisabledPeriodic() {
		drive->DriveCartesian(0,0,0);
		//led->RainbowFade(10);
		//led->Set(1,1,1);

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

		//cout << "vision correct: " << vision << endl;

		//up.toggle(pilot->ButtonState(GamepadF310::BUTTON_A));
		//down.toggle(pilot->ButtonState(GamepadF310::BUTTON_X));

		SmartDashboard::PutNumber("pot position",arm->getRawPosition());
		SmartDashboard::PutNumber("front left", fl.Get());
		SmartDashboard::PutNumber("back left", bl.Get());
		SmartDashboard::PutNumber("front right", fr.Get());
		SmartDashboard::PutNumber("back right", br.Get());
		SmartDashboard::PutNumber("gyro", gyro->GetAngle());

//		redLED.Set(Relay::kOn);
//		greenLED.Set(Relay::kOn);
//		blueLED.Set(Relay::kOn);
//		SmartDashboard::PutNumber("relay get", redLED.Get());





	}

};

Toggle Robot::vision(true);

START_ROBOT_CLASS(Robot)

