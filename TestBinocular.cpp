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

void TestBinocular(int argc = 0, char** argv = 0)//From webots/projects/samples/howto/controllers/binocular/binocular.c
{
	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const int camTimeStep = timeStep * 2;
	const int velThreshold = 1;

	//1.GetDevices
	Robot wbrobot;
	Camera* wbcams[2]; for (int k = 0; k < 2; ++k) wbcams[k] = wbrobot.getCamera("camera" + std::to_string(k));
	DistanceSensor* wbdists[2]; for (int k = 0; k < 2; ++k) wbdists[k] = wbrobot.getDistanceSensor("ds" + std::to_string(k));
	Motor* wbmotors[2]; for (int k = 0; k < 2; ++k) wbmotors[k] = wbrobot.getMotor(k == 0 ? "left wheel motor" : "right wheel motor");

	//2.EnableDevices
	for (int k = 0; k < 2; ++k) wbcams[k]->enable(camTimeStep);
	for (int k = 0; k < 2; ++k) wbdists[k]->enable(timeStep);
	for (int k = 0; k < 2; ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0.0); }

	//3.StartTask
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		Mat_<Vec4b> wbcamsData[2];
		for (int k = 0; k < 2; ++k)
		{
			wbcamsData[k].create(wbcams[k]->getHeight(), wbcams[k]->getWidth());
			memcpy(wbcamsData[k].data, wbcams[k]->getImage(), wbcamsData[k].total() * wbcamsData[k].step[1]);
		}
		double wbdistsData[2]; for (int k = 0; k < 2; ++k) wbdistsData[k] = wbdists[k]->getValue();
		double velocities[2] = { 0, 0 };
		if (wbdistsData[1] > 500)
		{
			//If both distance sensors are detecting something, this means that we are facing a wall. In this case we need to move backwards.
			if (wbdistsData[0] > 200) { velocities[0] = -velThreshold; velocities[1] = -velThreshold / 2; }
			//We turn proportionnaly to the sensors value because the closer we are from the wall, the more we need to turn.
			else { velocities[0] = -wbdistsData[1] / 100; velocities[1] = (wbdistsData[0] / 100) + 0.5; }
		}
		else if (wbdistsData[0] > 500) { velocities[0] = (wbdistsData[1] / 100) + 0.5; velocities[1] = -wbdistsData[0] / 100; }
		else { velocities[0] = velThreshold; velocities[1] = velThreshold; } //If nothing was detected we can move forward at maximal speed.

		//3.2 Control
		for (int k = 0; k < 2; ++k) wbmotors[k]->setVelocity(velocities[k]);

		//3.3 ShowResults
		Mat binoIma; hconcat(wbcamsData, 2, binoIma);//vconcat(wbcamsData, 2, binoIma);
		cv::imshow(__FUNCTION__, binoIma);
		cv::waitKey(timeStep / 2);
		spdlog::info("DistanceSensors: {:.3f}, {:.3f}", wbdistsData[0], wbdistsData[1]);
		spdlog::info("ControlledVelocities: {:.3f}, {:.3f}\n\n", velocities[0], velocities[1]);
	}
}

int main(int argc, char** argv) { TestBinocular(argc, argv); return 0; }