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

void Test2Motor1Cam(int argc = 0, char** argv = 0)
{
	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const int camTimeStep = timeStep * 2;
	if (argc < 4) { spdlog::error("Usage: appName camName leftMotorName rightMotorName"); return; }
	string cameraName(argv[1]);
	string motorsName[2] = { argv[2], argv[3] };
	const int64 timestart = ms1970;
	utils::fs::createDirectories(fmt::format("./{}", timestart));

	//1.GetDevices
	Robot wbrobot;
	Camera* wbcam; wbcam = wbrobot.getCamera(cameraName);
	Motor* wbmotors[2]; for (int k = 0; k < 2; ++k) wbmotors[k] = wbrobot.getMotor(motorsName[k]);

	//2.EnableDevices
	wbcam->enable(camTimeStep);
	for (int k = 0; k < 2; ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0); }

	//3.StartTask
	int ctlCode = 0;
	cv::namedWindow(__FUNCTION__, 0);
	int velFactor = 100; cv::createTrackbar("factor:", __FUNCTION__, &velFactor, 1000, 0, 0);
	int reservation = 0; cv::createTrackbar("RealVel=factor*0.01 ControlKeys=w/s/a/d/space SaveKey=1", __FUNCTION__, &reservation, 1, 0, 0);
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		Mat_<Vec4b> wbcamData(wbcam->getHeight(), wbcam->getWidth());
		memcpy(wbcamData.data, wbcam->getImage(), wbcamData.total() * wbcamData.step[1]);
		double velocity = velFactor * 0.01;

		//3.2 Control
		if (ctlCode == int('w')) { wbmotors[0]->setVelocity(velocity);  wbmotors[1]->setVelocity(velocity); }
		if (ctlCode == int('s')) { wbmotors[0]->setVelocity(-velocity);  wbmotors[1]->setVelocity(-velocity); }
		if (ctlCode == int('a')) { wbmotors[0]->setVelocity(-velocity);  wbmotors[1]->setVelocity(velocity); }
		if (ctlCode == int('d')) { wbmotors[0]->setVelocity(velocity);  wbmotors[1]->setVelocity(-velocity); }
		if (ctlCode == int(' ')) { wbmotors[0]->setVelocity(0);  wbmotors[1]->setVelocity(0); }

		//3.3 ShowResults
		cv::imshow(__FUNCTION__, wbcamData);
		ctlCode = cv::waitKey(timeStep / 2);
		if (ctlCode == int('1')) cv::imwrite(fmt::format("./{}/mono{}.png", timestart, ns1970), wbcamData);
		if (ctlCode == int('q')) break;
	}cv::destroyWindow(__FUNCTION__);
}

int main(int argc, char** argv) { Test2Motor1Cam(argc, argv); return 0; }