/*
 * pilot controls:
 * LeftY - forward/backwards
 * LeftX - strafe
 * RightX - turn
 * Button X - turns on field oriented drive
 * Left Stick - toggles teleop gyro correct
 * DPad Up - turns 180 degrees
 * Button B - changes camera exposure to show vision targets
 *
 * copilot controls:
 * Left trigger - arm up (&intake at the same time)
 * DPad left will stop the intake from spinning as the arm is raised up
 * Right trigger - arm down
 * Button Y -  intake
 * Button A - output
 * Button X - slow output
 * Button A - intake adjust (turns one side the other way)
 * DPad Up - winch up
 * DPad Down - winch down
 */



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
#include "ADXRS450_Gyro.h"

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


class Winch {
public:
	Winch(DigitalInput *limitSwitch, VictorSP *winch): limitSwitch(limitSwitch), winch(winch) {
	}
	void Set(float speed) {
		if (speed < 0) {
			speed = speed/2;
		}
		this->speed = speed;
	}
	void toUp() {
		speed = 1;
	}
	void toDown() {
		speed = -0.5;
	}
	void stop() {
		speed = 0;
	}
	bool isUp() {
		return limitSwitch->Get();
	};
	void update() {
		SmartDashboard::PutNumber("winch speed", winch->Get());
		/*if (isUp()) {
			speed = 0;
		}*/
		winch->Set(speed/1.5);
		speed = 0;
	}
private:
	DigitalInput *limitSwitch;
	VictorSP *winch;
	float speed = 0.0;

};

class Robot: public frc::IterativeRobot {
private:
	/*frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected; */

	enum AutoMode {NOTHING, CENTER = 2, LEFT = -1, RIGHT = 1, STRAIGHT = 3};
public:
	static const int FRONT_LEFT_PWM = 0; //real
	static const int BACK_LEFT_PWM = 1; //real
	static const int FRONT_RIGHT_PWM = 6; //real
	static const int BACK_RIGHT_PWM = 7; //real

//	static const int FRONT_LEFT_PWM = 6; //practice
//	static const int BACK_LEFT_PWM = 7; //practice
//	static const int FRONT_RIGHT_PWM = 0; //practice originally 1
//	static const int BACK_RIGHT_PWM = 3; //practice


	static const int LEFT_INTAKE_PWM = 4; //practice bot and real
	static const int RIGHT_INTAKE_PWM = 3; //real bot
//	static const int RIGHT_INTAKE_PWM = 1; //practice bot

	static const int ARM_PWM = 9; //real
//	static const int ARM_PWM = 8; //practice


	static const int WINCH_PWM = 2;
	static const int LIMIT_SWITCH_DIO = 9;

	static const int RED_LED_DIO = 21;
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

	frc::AnalogGyro *gyro; //practice bot with analog
//	frc::ADXRS450_Gyro *gyro; //real bot

	VictorSP fl {FRONT_LEFT_PWM};
	VictorSP bl {BACK_LEFT_PWM};
	VictorSP fr {FRONT_RIGHT_PWM};
	VictorSP br {BACK_RIGHT_PWM};

	Timer timer;

	TurnController turnController;
	PIDController turnPID {1/80.0, 0.0, 0.05, turnController,turnController,0.02};

	Lib830::DigitalLED *led;

	VictorSP * test;

	SendableChooser<AutoMode*> *chooser;

	//AnalogPotentiometer *pot;
	//PIDController *pid;

	Arm *arm;
	Intake *intake;

	DigitalLED *relayLED;

	Toggle gyroCorrect {true};
	Toggle armManual {true};

	Winch winch {new DigitalInput(LIMIT_SWITCH_DIO), new VictorSP(WINCH_PWM)};
	//VictorSP winch {WINCH_PWM};

	Encoder *flencoder;
	Encoder *blencoder;
	Encoder *frencoder;
	Encoder *brencoder;


	static Toggle vision;
	static Toggle driver_vision;
	static void CameraPeriodic() {
		CameraServer *server;
		grip::GripPipeline * pipeline;

		pipeline = new grip::GripPipeline();
		cs::UsbCamera vision_camera ("cam1", 1);
		cs::UsbCamera driver_camera ("cam0", 0);
		cv::Mat image;
		cv::Mat temp_image;
		bool g_frame = false;

		cs::CvSink sink;
		cs::CvSource outputStream;

		server = CameraServer::GetInstance();
		driver_camera = server->StartAutomaticCapture(0);
		vision_camera = server->StartAutomaticCapture(1);
		vision_camera.SetResolution(320,240);
		//driver_camera.SetResolution(640,480);


		//driver_camera.SetFPS(24);


		//string name = "serve_" + vision_camera.GetName();
		sink = server->GetVideo(vision_camera);
		outputStream = server->PutVideo("Processed", 320, 240);
		cout << "camera periodic" <<endl;


		bool setExposure = true;
		vision_camera.SetExposureManual(30);
		//driver_camera.SetExposureAuto();

		while(1) {
			if (driver_vision) {
				sink = server->GetVideo(driver_camera);
				vision = false;
			}
			else {
				sink = server->GetVideo(vision_camera);
			}

			bool working = sink.GrabFrame(temp_image);
			SmartDashboard::PutBoolean("working", working);

			if (working) {
				g_frame = true;
				image = temp_image;
			}
			if (!g_frame) {
				continue;
			}
			if (!driver_vision) {
//				if (setExposure) {
//					vision_camera.SetExposureManual(30);
//					setExposure = false;
//				}
//				pipeline->Process(image);
//			}
//			else {
//				if (!setExposure) {
//					vision_camera.SetExposureAuto();
//					setExposure = true;
//				}
				pipeline->Process(image);
			}
			if (!driver_vision) {
				outputStream.PutFrame(*pipeline->GetHslThresholdOutput());
			}
			else {
				outputStream.PutFrame(image);
			}

		}

	}

	void RobotInit() {
		/*chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser); */

		gyro = 	new frc::AnalogGyro(ANLOG_GYRO);
		//gyro = new frc::ADXRS450_Gyro();

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


		SmartDashboard::PutData("auton mode",chooser);

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
		SmartDashboard::PutBoolean("bad gyro", false);

		SmartDashboard::PutNumber("hue min", 70);
		SmartDashboard::PutNumber("hue max", 100);
		SmartDashboard::PutNumber("sat min", 0);
		SmartDashboard::PutNumber("sat max", 255);
		SmartDashboard::PutNumber("lum min", 200);
		SmartDashboard::PutNumber("lum max", 255);

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
	bool bad_gyro = false;
	float gyro_turn = 0;

	void encoderReset() {
		float pulse_per_rev = SmartDashboard::GetNumber("pulse per rev", 1024.0);
		float dis_per_pulse = (6*3.1416)/pulse_per_rev;
		distance = 0;

		flencoder->Reset();
		blencoder->Reset();
		frencoder->Reset();
		brencoder->Reset();

		flencoder->SetDistancePerPulse(dis_per_pulse);
		blencoder->SetDistancePerPulse(dis_per_pulse);
		frencoder->SetDistancePerPulse(dis_per_pulse);
		brencoder->SetDistancePerPulse(dis_per_pulse);
	}


	bool output_cube = false;
	bool toscale = false;
	bool toswitch = false;
	void AutonomousInit() override {

		driver_vision = false;
		timer.Reset();
		timer.Start();

		gyro->Reset();
		acquired = false;
		output_cube = false;
		toscale = false;
		toswitch = false;

		vision = true;
		//arm->toDown();
		encoderReset();

		bad_gyro = SmartDashboard::GetBoolean("bad gyro", false);
		gyro_turn = 0;
	}

	float StrafeVisionCorrect(){
		float midx = SmartDashboard::GetNumber("mid point x", 160);
		return ((midx-160)/160) /2;
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

	float SCALE_DIST = 305;
	float distance = 0;
	void AutonomousPeriodic() {
		string message = frc::DriverStation::GetInstance().GetGameSpecificMessage();
				SmartDashboard::PutString("message", message);
				char mes = message[0];
				char mes_2 = message[1];
//				cout << "first character: " << mes << endl;
//				cout << "second character: " << mes_2 << endl;



				AutoMode mode = NOTHING;
				if (chooser->GetSelected()) {
					mode = *chooser->GetSelected();
				}

				float x_speed = 0;
				float y_speed = 0;
				float angle = gyro->GetAngle() + gyro_turn;
				float rot = angle/-30;


				float time = timer.Get();

				distance = (GetEncoderDistance(flencoder) + GetEncoderDistance(blencoder) + GetEncoderDistance(frencoder) + GetEncoderDistance(brencoder)) /4.0;;
				if (mode == NOTHING){
					x_speed = 0;
					y_speed = 0;
					rot = 0;
				} else{
					if (time < 1) {
						if (mode == CENTER) {
							y_speed = 0.3;
						}
						else {
							y_speed = 0.5;
						}

							arm->toSwitchNoPot();
							intake->toIntake();
					}

					else if (time > 1 && time < 8) {
						y_speed = 0.5;
						intake->toIntake();
						//arm->toSwitch();
						arm->toSwitchNoPot();
						if (time > 1.5) {
							winch.Set(-1.0);
						}


						if (acquired) {
							y_speed = 0.3; //set speed to be slower
						}
						switch(mode) {
						case CENTER:
							y_speed = 0.3;
							if (mes == 'L') {
								x_speed = getXSpeed(StrafeVisionCorrect(), -0.8);
							}
							else if (mes == 'R') {
								x_speed = getXSpeed(StrafeVisionCorrect(), 0.8);
							}
							output_cube = true;
							break;
						case RIGHT:
							y_speed = 0.5;
							if (mes == 'L') {
								if (mes_2 == 'R') {
									toscale = true;
								}
							else if (mes == 'R') {
								toswitch = true;
								if (time > 4.75) { //sketch\[][]

									y_speed = 0;
								}
							}
							break;
						case LEFT:
							x_speed = 0;
							y_speed = 0.5;
							if (mes == 'L') {
								toswitch = true;
								if (time > 4.75) {
									y_speed = 0;
								}
							}
							else if (mes == 'R') {
								if (mes_2 == 'L') {
									toscale = true;
								}
							}
							break;
						case STRAIGHT:
							if (time > 3.5) {
								y_speed = 0;
							}

							break;
						default:
							x_speed = 0;
							y_speed = 0;
							rot = 0;
							break;
						}
					}
					}
					else if (time > 8) {
						x_speed = 0;
						y_speed = 0;
						if(output_cube){
							intake->toOutput();
						}
						if (toscale || toswitch) {
							float after_turn_speed = 0;
							if (toscale) {
								after_turn_speed = 0.2;
								winch.Set(1);
							}
							else {
								after_turn_speed = 0.7;
							}
							if (time < 10) {
								if (mode == LEFT) {
									//angle = gyro->GetAngle() - 90;
									gyro_turn = -90;
									rot = angle/-90;
								}
								else if (mode == RIGHT) {
									//angle = gyro->GetAngle() + 90;
									gyro_turn = 90;
									rot = angle/-90;
								}
							}
							else if (time > 10 && time < 11.5) {
								y_speed = after_turn_speed;
							}
							else {
								y_speed = 0;
								rot = 0;
								//intake->toOutput();
								output_cube = true;
							}
						}
					}
				}

				SmartDashboard::PutNumber("get distance", distance);
				SmartDashboard::PutNumber("raw", flencoder->GetRaw());

				if (bad_gyro) {
					rot = 0;
				}

				float f_x_speed = accel(prev_x_speed, x_speed, TICKS_TO_ACCEL);
				float f_y_speed = accel(prev_y_speed, y_speed, TICKS_TO_ACCEL);
				drive->DriveCartesian(f_x_speed/1.5, f_y_speed/1.5, rot, 0);
				arm->armMoveUpdate();
				intake->update();
				winch.update();

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
			//cout << "PRESSED SUCCESS!" <<endl;
			gyroTarget += 151;
			pressed = false;
		}
		SmartDashboard::PutNumber("setpoint", gyroTarget);
		led->RainbowFade(1);
	}
	Toggle turn_180;

	float change = 0;

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
		gyroTarget = 0;
		led->Disable();

		bad_gyro = SmartDashboard::GetBoolean("bad gyro", true);
		if (bad_gyro) {
			gyroCorrect = false;
		}

		driver_vision = true;
		encoderReset();


	}


	float prev_turn = 0;

	Toggle field_orient;

	float deadZone(float input) {
		if (fabs(input) < 0.13) {
			return 0;
		}
		else {
			return input;
		}
	}

	Toggle intakeMove;


	void TeleopPeriodic() override {
		distance = (GetEncoderDistance(flencoder) + GetEncoderDistance(blencoder) + GetEncoderDistance(frencoder) + GetEncoderDistance(brencoder)) /4.0;
		SmartDashboard::PutNumber("get distance", distance);

 		float angle = gyro->GetAngle() - change;
		turnController.gyro_angle = angle;
		DigitalLED::Color color1 {DigitalLED::Lime};
		DigitalLED::Color color2 {DigitalLED::Lime};
		float y_speed = Lib830::accel(prev_y_speed, deadZone(pilot->LeftY()), TICKS_TO_ACCEL);
		float x_speed = Lib830::accel(prev_x_speed, deadZone(pilot->LeftX()), TICKS_TO_ACCEL );
		float turn = deadZone(pilot->RightX()/2);

		SmartDashboard::PutNumber("value turn", deadZone(pilot->RightX()));
		float gyro_read = 0;

		if (field_orient.toggle(pilot->ButtonState(GamepadF310::BUTTON_X))){
			color1 = DigitalLED::Magenta;
			gyro_read = gyro->GetAngle();
		}


		SmartDashboard::PutBoolean("gyro correct", gyroCorrect);

		if (gyroCorrect.toggle(pilot->ButtonState(GamepadF310::BUTTON_LEFT_STICK))) {
			color1 = DigitalLED::White;
			turn = turnController.turn;
			gyroTarget += 2*deadZone(pilot->RightX());
			if (gyroTarget < 360){
				gyroTarget += 360;
			}
			turnPID.SetSetpoint(fmod(gyroTarget,360));
		}
		else  {
			change = gyro->GetAngle();
			gyroTarget = 0;
		}
		turn_180.toggle(pilot->DPadLeft());
		doA180(turn_180);

		SmartDashboard::PutBoolean("180 turn",turn_180);
		SmartDashboard::PutNumber("adjusted angle", angle);

		drive->DriveCartesian(x_speed, y_speed, turn, -gyro_read);


		prev_y_speed = y_speed;
		prev_x_speed = x_speed;

		SmartDashboard::PutNumber("gyro read", gyro_read);
		SmartDashboard::PutBoolean("field orient", field_orient);
		SmartDashboard::PutNumber("Left y", deadZone(pilot->LeftY()));
		SmartDashboard::PutNumber("actual left y", pilot->LeftY());



		winch.Set(deadZone(copilot->LeftY()));

		down.toggle(copilot->LeftTrigger());
		up.toggle(copilot->RightTrigger());
		if (armManual.toggle(copilot->ButtonState(GamepadF310::BUTTON_START))){
			arm->rawPosition(copilot->RightTrigger()-(copilot->LeftTrigger()/2));
			if (copilot->RightTrigger() && !copilot->DPadLeft()) {
				intake->toIntake();
			}

			color2 = DigitalLED::Cyan;
		}
		else {
			if (copilot->ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER) || copilot->ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) ) {
				arm->manualPosition(copilot->RightTrigger(),copilot->LeftTrigger());
				color2 = DigitalLED::Yellow;
			}
			else {
				arm->automaticPosition(up, down);
			}
		}

		if(copilot->ButtonState(GamepadF310::BUTTON_Y)){
			intake->toIntake();
			if (copilot->ButtonState(GamepadF310::BUTTON_B)) {
				intake->toAdjust();
			}
		}
		else if(copilot->ButtonState(GamepadF310::BUTTON_X)){
			intake->toSlowOutput();
		}
		else if(copilot->ButtonState(GamepadF310::BUTTON_A)){
			intake->toOutput();
		}
		arm->armMoveUpdate();
		intake->update();
		winch.update();

	}

	Toggle up;
	Toggle down;

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

	void RobotPeriodic() override {
		vision.toggle(pilot->ButtonState(GamepadF310::BUTTON_B));
		driver_vision.toggle(pilot->ButtonState(GamepadF310::BUTTON_BACK));
		SmartDashboard::PutBoolean("vision", vision);
		SmartDashboard::PutBoolean("driver vision", driver_vision);

		SmartDashboard::PutNumber("pot position",arm->getRawPosition());
		SmartDashboard::PutNumber("front left", fl.Get());
		SmartDashboard::PutNumber("back left", bl.Get());
		SmartDashboard::PutNumber("front right", fr.Get());
		SmartDashboard::PutNumber("back right", br.Get());
		SmartDashboard::PutNumber("gyro", gyro->GetAngle());
		//SmartDashboard::PutBoolean("limit switch",winch.isUp());

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

	}



};

Toggle Robot::vision(true);
Toggle Robot::driver_vision(true);

START_ROBOT_CLASS(Robot)

