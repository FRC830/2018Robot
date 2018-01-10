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


class Robot: public frc::IterativeRobot {
public:
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

			pipeline->Process(image);



			//outputStream.PutFrame(*pipeline->GetHslThresholdOutput());

			outputStream.PutFrame(image);
		}

	}


	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		std::thread visionThread(CameraPeriodic);
		visionThread.detach();

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

	void TeleopPeriodic() {

	}

	void TestPeriodic() {
		//lw->Run();

	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};


START_ROBOT_CLASS(Robot)

