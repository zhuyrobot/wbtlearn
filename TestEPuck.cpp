#include<opencv2/opencv.hpp>
#include<spdlog/spdlog.h>
#include <webots/GPS.hpp>
#include <webots/LED.hpp>
#include <webots/Pen.hpp>
#include <webots/Gyro.hpp>
#include <webots/Node.hpp>
#include <webots/Skin.hpp>
#include <webots/Brake.hpp>
#include <webots/Field.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Mouse.hpp>
#include <webots/Radar.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Compass.hpp>
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Speaker.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Receiver.hpp>
#include <webots/Connector.hpp>
#include <webots/Supervisor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include <webots/DifferentialWheels.hpp>
using namespace std;
using namespace cv;

//Comment last 6 lines in E-puck.proto if showing e-puck_wifi.dll initialisation failed
void TestEPuck(int argc = 0, char** argv = 0)//From webots/projects/robots/gctronic/e-puck/controllers/e-puck/e-puck.c
{
	//0.GetConstant
	using namespace webots;
	const int range = 512;
	const double wheelRadius = 0.02;
	const double axleLength = 0.052;
	const double braitenberg[8][2] = { {0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06}, {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942} };

	//1.GetDevices
	Robot wbrobot;
	Camera* wbcam = wbrobot.getCamera("camera");
	Accelerometer* wbacc = wbrobot.getAccelerometer("accelerometer");
	DistanceSensor* wbdists[8]; for (int k = 0; k < 8; ++k) wbdists[k] = wbrobot.getDistanceSensor("ps" + std::to_string(k));
	PositionSensor* wbposs[2]; for (int k = 0; k < 2; ++k) wbposs[k] = wbrobot.getPositionSensor(k == 0 ? "left wheel sensor" : "right wheel sensor");
	Motor* wbmotors[2]; for (int k = 0; k < 2; ++k) wbmotors[k] = wbrobot.getMotor(k == 0 ? "left wheel motor" : "right wheel motor");

	//2.EnableDevices
	int timeStep = wbrobot.getModel() == "GCtronic e-puck2" ? 64 : 256;
	int camTimeStep = wbrobot.getModel() == "GCtronic e-puck2" ? 64 : 1024;
	wbcam->enable(camTimeStep);
	wbacc->enable(timeStep);
	for (int k = 0; k < 8; ++k) wbdists[k]->enable(timeStep);
	for (int k = 0; k < 2; ++k) wbposs[k]->enable(timeStep);
	for (int k = 0; k < 2; ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0.0); }

	//3.StartTask
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		Mat_<Vec4b> wbcamData(wbcam->getHeight(), wbcam->getWidth());
		memcpy(wbcamData.data, wbcam->getImage(), wbcamData.total() * wbcamData.step[1]);
		const double* wbaccData = wbacc->getValues();
		double wbdistsData[8]; for (int k = 0; k < 8; ++k) wbdistsData[k] = wbdists[k]->getValue();
		double wbpossData[2]; for (int k = 0; k < 2; ++k) wbpossData[k] = wbposs[k]->getValue();
		double distances[2]; for (int k = 0; k < 2; ++k) distances[k] = wbpossData[k] * wheelRadius;
		double orientation = (distances[1] - distances[0]) * axleLength;
		double velocities[2] = { 0, 0 };
		for (int i = 0; i < 2; ++i)
			for (int j = 0; j < 8; ++j)
				velocities[i] += braitenberg[j][i] * (1.0 - wbdistsData[j] / range);

		//3.2 Control
		for (int k = 0; k < 2; ++k) wbmotors[k]->setVelocity(velocities[k]);

		//3.3 ShowResults
		cv::imshow(__FUNCTION__, wbcamData);
		cv::waitKey(timeStep / 2);
		spdlog::info("Accelerometer: {:.3f}, {:.3f}, {:.3f}", wbaccData[0], wbaccData[1], wbaccData[2]);
		spdlog::info("DistanceSensors: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}", wbdistsData[0], wbdistsData[1], wbdistsData[2], wbdistsData[3], wbdistsData[4], wbdistsData[5], wbdistsData[6], wbdistsData[7]);
		spdlog::info("PositionSensors: {:.3f}, {:.3f}", wbpossData[0], wbpossData[1]);
		spdlog::info("DeducedDistances: {:.3f}, {:.3f}", distances[0], distances[1]);
		spdlog::info("DeducedOrientation: {:.3f}", orientation);
		spdlog::info("ControlledVelocities: {:.3f}, {:.3f}\n\n", velocities[0], velocities[1]);
	}
}

int main(int argc, char** argv) { TestEPuck(argc, argv); return 0; }