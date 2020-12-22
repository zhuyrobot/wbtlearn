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

void TestFourWheels(int argc = 0, char** argv = 0)//From webots/projects/samples/tutorials/controllers/four_wheels_collision_avoidance/four_wheels_collision_avoidance.c
{
	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const double maxSpeed = 6.28;

	//1.GetDevices
	Robot wbrobot;
	DistanceSensor* wbdists[2]; for (int k = 0; k < 2; ++k) wbdists[k] = wbrobot.getDistanceSensor(k == 0 ? "distanceSensor2" : "distanceSensor1");
	Motor* wbmotors[4];  for (int k = 0; k < 4; ++k) wbmotors[k] = wbrobot.getMotor("motor" + std::to_string(k + 1));

	//2.EnableDevices
	for (int i = 0; i < 2; i++) wbdists[i]->enable(timeStep);
	for (int i = 0; i < 4; i++) { wbmotors[i]->setPosition(INFINITY); wbmotors[i]->setVelocity(0.0); }

	//3.StartTask
	int nAvoidObstacle = 0;
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		double wbdistsData[2]; for (int k = 0; k < 2; ++k) wbdistsData[k] = wbdists[k]->getValue();
		double velocities[2] = { 1.0, 1.0 };
		if (nAvoidObstacle > 0)
		{
			--nAvoidObstacle;
			velocities[0] = 1.0;
			velocities[1] = -1.0;
		}
		else
			for (int k = 0; k < 2; ++k)
				if (wbdistsData[k] < 950) nAvoidObstacle = 100;

		//3.2 Control
		for (int k = 0; k < 4; ++k) wbmotors[k]->setVelocity(k % 2 == 0 ? velocities[0] : velocities[1]);

		//3.3 ShowResults
		spdlog::info("DistanceSensors: {:.3f}, {:.3f}", wbdistsData[0], wbdistsData[1]);
		spdlog::info("ControlledVelocities: {:.3f}, {:.3f}\n\n", velocities[0], velocities[1]);
	}
}

int main(int argc, char** argv) { TestFourWheels(argc, argv); return 0; }