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
#include "EncoderDrive.h"

using namespace Lib830;
using namespace std;

class TurnController : public PIDSource, public PIDOutput {
public :
	std::atomic<double> gyro_angle;
	std::atomic<double> turn;
	double PIDGet(){
		return gyro_angle;
	}
	void PIDWrite(double turn){
		this->turn = turn;
	}
};


class Robot: public frc::IterativeRobot {
private:
	/*frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected; */

	enum AutoMode {NOTHING, CENTER, LEFT, RIGHT, STRAIGHT};

public:
	static const int FRONT_LEFT_PWM = 4; //real
	static const int BACK_LEFT_PWM = 9; //real
	static const int FRONT_RIGHT_PWM = 0; //real
	static const int BACK_RIGHT_PWM = 1; //real

//	static const int FRONT_LEFT_PWM = 1; //practice
//	static const int BACK_LEFT_PWM = 0; //practice
//	static const int FRONT_RIGHT_PWM = 3; //practice
//	static const int BACK_RIGHT_PWM = 2; //practice


	static const int LEFT_INTAKE_PWM = 5; //subject to choonge
	static const int RIGHT_INTAKE_PWM = 6;

	static const int ARM_PWM = 8;


	static const int RED_LED_DIO = 9;
	static const int GREEN_LED_DIO = 24;
	static const int BLUE_LED_DIO = 25;

	static const int ANLOG_GYRO = 0;
	static const int POTENTIOMETER_ANALOG = 1;

	static const int TICKS_TO_ACCEL = 15;

	static const int RED_RELAY = 0;
	static const int GREEN_RELAY = 1;
	static const int BLUE_RELAY = 2;

	static const int FL_ENCODER_DIO_ONE = 0;
	static const int FL_ENCODER_DIO_TWO = 1;
	static const int BL_ENCODER_DIO_ONE = 2;
	static const int BL_ENCODER_DIO_TWO = 3;
	static const int FR_ENCODER_DIO_ONE = 4;
	static const int FR_ENCODER_DIO_TWO = 5;
	static const int BR_ENCODER_DIO_ONE = 6;
	static const int BR_ENCODER_DIO_TWO = 7;


	MecanumDrive *drive;
	Lib830::GamepadF310 * pilot;
	Lib830::GamepadF310 * copilot;

	frc::AnalogGyro *gyro;
//	Relay redLED;
//	Relay greenLED;
//	Relay blueLED;

	VictorSP fl;
	VictorSP bl;
	VictorSP fr;
	VictorSP br;
	Timer timer;

	TurnController turnController;
	PIDController turnPID;

	Lib830::DigitalLED *led;

	VictorSP * test;

	SendableChooser<AutoMode*> *chooser;

	//AnalogPotentiometer *pot;
	//PIDController *pid;

	Arm *arm;
	Intake *intake;

	DigitalLED *relayLED;

	Toggle gyroCorrect;

	Encoder *flencoder;
	Encoder *blencoder;
	Encoder *frencoder;
	Encoder *brencoder;




	Robot() :IterativeRobot(),
			fl(FRONT_LEFT_PWM),
			bl(BACK_LEFT_PWM),
			fr(FRONT_RIGHT_PWM),
			br(BACK_RIGHT_PWM),
			turnPID(1/80.0, 0.0, 0.05, turnController,turnController,0.02),
			gyroCorrect(true)

//			redLED(RED_RELAY, Relay::kForwardOnly),
//			greenLED(GREEN_RELAY, Relay::kForwardOnly),
//			blueLED(BLUE_RELAY, Relay::kForwardOnly)
			{led = nullptr;}

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
		return (midx-160)/160;
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
		chooser->AddObject("Straight", new AutoMode(STRAIGHT));

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


		turnPID.SetName("turn PID");
		SmartDashboard::PutData(&turnPID);

//		encoderDrive = new EncoderDrive (
//			new Encoder(FL_ENCODER_DIO_ONE, FL_ENCODER_DIO_TWO),
//			new Encoder(BL_ENCODER_DIO_ONE, BL_ENCODER_DIO_TWO, true),
//			new Encoder(FR_ENCODER_DIO_ONE, FR_ENCODER_DIO_TWO, true),
//			new Encoder(BR_ENCODER_DIO_ONE, BR_ENCODER_DIO_TWO, true),
//			&fl, &bl, &fr, &br
//		);

		flencoder = new Encoder(FL_ENCODER_DIO_ONE, FL_ENCODER_DIO_TWO, true);
		blencoder = new Encoder(BL_ENCODER_DIO_ONE, BL_ENCODER_DIO_TWO, true);
		frencoder = new Encoder(FR_ENCODER_DIO_ONE, FR_ENCODER_DIO_TWO);
		brencoder = new Encoder(BR_ENCODER_DIO_ONE, BR_ENCODER_DIO_TWO);
		SmartDashboard::PutNumber("pulse per rev", 1024);

	}

	double GetEncoderDistance(Encoder *encoder) {
		return encoder->GetDistancePerPulse() * encoder->GetRaw();
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
		flencoder->Reset();
		blencoder->Reset();
		frencoder->Reset();
		brencoder->Reset();


		float pulse_per_rev = SmartDashboard::GetNumber("pulse per rev", 1024.0);

		flencoder->SetDistancePerPulse((6*3.1416)/pulse_per_rev);
		blencoder->SetDistancePerPulse((6*3.1416)/pulse_per_rev);
		frencoder->SetDistancePerPulse((6*3.1416)/pulse_per_rev);
		brencoder->SetDistancePerPulse((6*3.1416)/pulse_per_rev);



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

	float CORRECT_SIDE_SCALE_DIST = 305;
	float distance = 0;
	float start_time = 0;

	void AutonomousPeriodic() {
		string message = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		SmartDashboard::PutString("message", message);
		char mes = message[0];
		char mes_2 = message[1];
		cout << "first character: " << mes << endl;
		cout << "second character: " << mes_2 << endl;



		AutoMode mode = NOTHING;
		if (chooser->GetSelected()) {
			mode = *chooser->GetSelected();
		}

		float x_speed = 0;
		float y_speed = 0;
		float angle = gyro->GetAngle();
		float rot = angle/-30;


		float time = timer.Get();

		distance = (GetEncoderDistance(flencoder) + GetEncoderDistance(blencoder) + GetEncoderDistance(frencoder) + GetEncoderDistance(brencoder)) /4.0;;

		if (time < 1) {
			y_speed = 0.5;
		}

		else if (time > 1 && time < 8) {
			y_speed = 0.3;
			arm->toSwitch();

			if (acquired) {
				y_speed = 0.3; //set speed to be slower
			}
			switch(mode) {
			case CENTER:
				output_cube = true;
				if (mes == 'L') {
					x_speed = getXSpeed(StrafeVisionCorrect(), -0.7);
				}
				else if (mes == 'R') {
					x_speed = getXSpeed(StrafeVisionCorrect(), 0.7);
				}
				break;
			case RIGHT:
				if (mes == 'L') {
					x_speed = getXSpeed(StrafeVisionCorrect(), -0.2);
					if (acquired && StrafeVisionCorrect() < 5) {//
						start_time = time;
						x_speed = 0.5;
						acquired = false;

					}
					else if (!acquired && start_time) {
						x_speed = 0.5;
						SmartDashboard::PutNumber("start time", time - start_time);
						if (time - start_time > 0.5) {
							x_speed = 0;

							if (mes_2 == 'R') {
								if (distance < CORRECT_SIDE_SCALE_DIST) {
									y_speed = (CORRECT_SIDE_SCALE_DIST - distance)/50;
									if (time- start_time > 1.25) {
										x_speed = -0.8;
									}
								}
								else {
									y_speed = 0;
								}
							}
							else if (mes_2 == 'L') {
								/*do stuff*/
							}
						}
					}
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
			case STRAIGHT:
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

		SmartDashboard::PutNumber("get distance", distance);
		SmartDashboard::PutNumber("raw", flencoder->GetRaw());


		float f_x_speed = accel(prev_x_speed, x_speed, TICKS_TO_ACCEL);
		float f_y_speed = accel(prev_y_speed, y_speed, TICKS_TO_ACCEL);

		drive->DriveCartesian(f_x_speed/1.5, f_y_speed/1.5, rot, angle);
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

	float gyroTarget = 0;

	void doA180(Toggle &pressed) {
		if (pressed) {
			cout << "PRESSED SUCCESS!" <<endl;
			gyroTarget += 181;
			pressed = false;
		}
		SmartDashboard::PutNumber("setpoint", gyroTarget);
		led->RainbowFade(1);
	}
	Toggle turn_180;

	float change = 0;
	int times_ran = 0;
	float raw_total = 0;

	void TeleopInit() {
		turnPID.SetInputRange(0,360);
		turnPID.SetOutputRange(-0.3,0.3);
		turnPID.SetAbsoluteTolerance(1);
		turnPID.SetSetpoint(0);
		turnPID.SetContinuous();
		turnPID.Enable();


		vision = false;
		turn_180 = false;
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
//		SmartDashboard::PutNumber("fl encoder", flencoder.GetRate());
//		SmartDashboard::PutNumber("bl encoder", blencoder.GetRate());
//		SmartDashboard::PutNumber("fr encoder", frencoder.GetRate());
//		SmartDashboard::PutNumber("br encoder", brencoder.GetRate());
		float angle = gyro->GetAngle() - change;
		turnController.gyro_angle = angle;
		//DigitalLED::Color color1 {DigitalLED::Lime};
		//DigitalLED::Color color2 {DigitalLED::Lime};
		float y_speed = Lib830::accel(prev_y_speed, value(pilot->LeftY()), TICKS_TO_ACCEL);
		float x_speed = Lib830::accel(prev_x_speed, value(pilot->LeftX()), TICKS_TO_ACCEL );
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

		SmartDashboard::PutBoolean("gyro correct", gyroCorrect);

		if (gyroCorrect.toggle(pilot->ButtonState(GamepadF310::BUTTON_LEFT_STICK))) {
			turn = turnController.turn;
			gyroTarget += 2*value(pilot->RightX());
			if (gyroTarget < 360){
				gyroTarget += 360;
			}
			turnPID.SetSetpoint(fmod(gyroTarget,360));
		}
		else  {
			change = gyro->GetAngle();
		}
		turn_180.toggle(pilot->DPadLeft());
		doA180(turn_180);

		SmartDashboard::PutBoolean("180 turn",turn_180);
		SmartDashboard::PutNumber("adjusted angle", angle);

		//float final_turn =  Lib830::accel(prev_turn, turn, 5);

		//encoderDrive->DriveCarties(x_speed, y_speed, final_turn, 0);

		drive->DriveCartesian(x_speed, y_speed, turn, -gyro_read);
		//testDrive->DriveCartes(x_speed, y_speed, turn, -gyro_read);



		prev_y_speed = y_speed;
		prev_x_speed = x_speed;
		//prev_turn = final_turn;

		SmartDashboard::PutNumber("gyro read", gyro_read);
		SmartDashboard::PutBoolean("field orient", field_orient);
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
		//drive->DriveCartesian(0,0,0);
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

	void RobotPeriodic() override {
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
		static Timer t2;
		t2.Start();
		if (int(t2.Get()) % 2)
			led->Set(1,1,1);
		else
			led->Set(0,0,0);

		SmartDashboard::PutNumber("raw", flencoder->GetRaw());
		SmartDashboard::PutNumber("get distance", distance);

	}



};

Toggle Robot::vision(true);

START_ROBOT_CLASS(Robot)

