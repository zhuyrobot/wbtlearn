#include<opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
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

#ifndef ns1970
#define ms1970 chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define us1970 chrono::time_point_cast<chrono::microseconds>(chrono::system_clock::now()).time_since_epoch().count()
#define ns1970 chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count()
#endif

void Test2Motor2Cam(int argc = 0, char** argv = 0)//From webots/projects/samples/howto/controllers/binocular/binocular.c
{
	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const int camTimeStep = timeStep * 2;
	if (argc < 5) { spdlog::error("Usage: appName leftCamName rightCamName leftMotorName rightMotorName"); return; }
	string camsName[2] = { argv[1], argv[2] };
	string motorsName[2] = { argv[3], argv[4] };
	const int64 timestart = ms1970;
	utils::fs::createDirectories(fmt::format("./{}", timestart));

	//1.GetDevices
	Robot wbrobot;
	Camera* wbcams[2]; for (int k = 0; k < 2; ++k) wbcams[k] = wbrobot.getCamera(camsName[k]);
	Motor* wbmotors[2]; for (int k = 0; k < 2; ++k) wbmotors[k] = wbrobot.getMotor(motorsName[k]);

	//2.EnableDevices
	for (int k = 0; k < 2; ++k) wbcams[k]->enable(camTimeStep);
	for (int k = 0; k < 2; ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0); }

	//3.StartTask
	int ctlCode = 0;
	cv::namedWindow(__FUNCTION__, 0);
	int velFactor = 100; cv::createTrackbar("factor:", __FUNCTION__, &velFactor, 1000, 0, 0);
	int reservation = 0; cv::createTrackbar("RealVel=factor*0.01 ControlKeys=w/s/a/d/space SaveKey=1", __FUNCTION__, &reservation, 1, 0, 0);
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		Mat_<Vec4b> wbcamsData[2];
		for (int k = 0; k < 2; ++k)
		{
			wbcamsData[k].create(wbcams[k]->getHeight(), wbcams[k]->getWidth());
			memcpy(wbcamsData[k].data, wbcams[k]->getImage(), wbcamsData[k].total() * wbcamsData[k].step[1]);
		}
		double velocity = velFactor * 0.01;

		//3.2 Control
		if (ctlCode == int('w')) { wbmotors[0]->setVelocity(velocity);  wbmotors[1]->setVelocity(velocity); }
		if (ctlCode == int('s')) { wbmotors[0]->setVelocity(-velocity);  wbmotors[1]->setVelocity(-velocity); }
		if (ctlCode == int('a')) { wbmotors[0]->setVelocity(-velocity);  wbmotors[1]->setVelocity(velocity); }
		if (ctlCode == int('d')) { wbmotors[0]->setVelocity(velocity);  wbmotors[1]->setVelocity(-velocity); }
		if (ctlCode == int(' ')) { wbmotors[0]->setVelocity(0);  wbmotors[1]->setVelocity(0); }

		//3.3 ShowResults
		Mat binoIma; hconcat(wbcamsData, 2, binoIma);//vconcat(wbcamsData, 2, binoIma);
		cv::imshow(__FUNCTION__, binoIma);
		ctlCode = cv::waitKey(timeStep / 2);
		if (ctlCode == int('1'))
		{
			int64 name = ns1970;
			utils::fs::createDirectories(fmt::format("./{}/left", timestart)); cv::imwrite(fmt::format("./{}/left/left{}.png", timestart, name), wbcamsData[0]);
			utils::fs::createDirectories(fmt::format("./{}/right", timestart)); cv::imwrite(fmt::format("./{}/right/right{}.png", timestart, name), wbcamsData[1]);
			utils::fs::createDirectories(fmt::format("./{}/bino", timestart)); cv::imwrite(fmt::format("./{}/bino/bino{}.png", timestart, name), binoIma);
		}
		if (ctlCode == int('q')) break;
	}cv::destroyWindow(__FUNCTION__);
}

int main(int argc, char** argv) { Test2Motor2Cam(argc, argv); return 0; }