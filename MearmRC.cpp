#include<opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
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

class MearmRC : public QWidget
{
public:
	static void TestGUI(int argc = 0, char** argv = 0)
	{
		QApplication app(argc, argv);
		MearmRC me;
		me.show();
		while (1)
		{
			QApplication::processEvents();
			this_thread::sleep_for(100ms);
		}
	}
	static void TestURE(int argc = 0, char** argv = 0)
	{
		//0.GetConstant
		using namespace webots;
		const int timeStep = 64;
		vector<string> motorsName = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

		//1.GetDevices
		Robot wbrobot;
		vector<Motor*> wbmotors(motorsName.size()); for (int k = 0; k < wbmotors.size(); ++k) wbmotors[k] = wbrobot.getMotor(motorsName[k]);

		//2.EnableDevices
		for (int k = 0; k < wbmotors.size(); ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0); }

		//3.StartTask
		QApplication qtApp(argc, argv);
		MearmRC mearmRC; mearmRC.show();
		while (wbrobot.step(timeStep) != -1)
		{
			//3.1 Percept

			//3.2 Control
			for (int k = 0; k < wbmotors.size(); ++k) wbmotors[k]->setVelocity(mearmRC.motors[k]);

			//3.3 ShowResults
			QApplication::processEvents();
		}
	}

public:
	double motors[6] = { 0 };
	int64 actions[2] = { 0 };
	int64 nspace = 0;
	void keyPressEvent(QKeyEvent* e) { if (e->key() == ' ') ++nspace; groupBoxDisplayPanel->setTitle(QString("DisplayPanel: nspace=%1").arg(nspace)); }
	MearmRC(QWidget* parent = 0) : QWidget(parent)
	{
		//0.Basic settting
		this->setWindowTitle("Super Cube");
		this->setMinimumSize(QSize(640, 240));
		this->setFont(QFont("", 15, QFont::Thin));
		this->setFocusPolicy(Qt::StrongFocus);

		//1.Group1 setting
		vboxLayoutMain->addWidget(groupBoxControlPanel);
		vboxLayoutMain->addWidget(groupBoxDisplayPanel);
		{
			vboxLayoutMain->setStretch(1, 1);
			QList<QWidget*> children = groupBoxDisplayPanel->findChildren<QWidget*>();
			for (int k = 0; k < children.size(); ++k) children[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
		}

		//2.Group2 setting
		for (int k = 0; k < comboBoxMotorModes.size(); ++k) gridLayoutControlPanel->addWidget(comboBoxMotorModes[k], k, 0);
		for (int k = 0; k < doubleSpinBoxMotorRanges.size(); ++k) gridLayoutControlPanel->addWidget(doubleSpinBoxMotorRanges[k], 0, k + 1);
		for (int k = 0; k < pushButtonMotorPlus.size(); ++k) gridLayoutControlPanel->addWidget(pushButtonMotorPlus[k], 1, k + 1);
		for (int k = 0; k < pushButtonMotorMinus.size(); ++k) gridLayoutControlPanel->addWidget(pushButtonMotorMinus[k], 2, k + 1);
		{
			for (int k = 0; k < comboBoxMotorModes.size(); ++k) comboBoxMotorModes[k]->addItems(QStringList() << "Mode0" << "Mode1");
			for (int k = 0; k < doubleSpinBoxMotorRanges.size(); ++k) { doubleSpinBoxMotorRanges[k]->setRange(0, 100); doubleSpinBoxMotorRanges[k]->setValue(0.5); }
			for (int k = 0; k < comboBoxMotorModes.size(); ++k) connect(comboBoxMotorModes[k], QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)->void {QMessageBox::information(this, "", "Not enabled yet"); });
			for (int k = 0; k < pushButtonMotorPlus.size(); ++k) connect(pushButtonMotorPlus[k], &QPushButton::pressed, [this, k]()->void
				{
					motors[k] = doubleSpinBoxMotorRanges[k]->value();
					groupBoxControlPanel->setTitle(QString("ControlPanel: motor%1=%2").arg(k).arg(motors[k]));
				});
			for (int k = 0; k < pushButtonMotorMinus.size(); ++k) connect(pushButtonMotorMinus[k], &QPushButton::pressed, [this, k]()->void
				{
					motors[k] = -doubleSpinBoxMotorRanges[k]->value();
					groupBoxControlPanel->setTitle(QString("ControlPanel: motor%1=%2").arg(k).arg(motors[k]));
				});
			for (int k = 0; k < pushButtonMotorPlus.size(); ++k) connect(pushButtonMotorPlus[k], &QPushButton::released, [this, k]()->void
				{
					motors[k] = 0;
					groupBoxControlPanel->setTitle(QString("ControlPanel: motor%1=%2").arg(k).arg(motors[k]));
				});
			for (int k = 0; k < pushButtonMotorMinus.size(); ++k) connect(pushButtonMotorMinus[k], &QPushButton::released, [this, k]()->void
				{
					motors[k] = 0;
					groupBoxControlPanel->setTitle(QString("ControlPanel: motor%1=%2").arg(k).arg(motors[k]));
				});
		}

		//3.Group3 setting
		for (int k = 0; k < pushButtonActions.size(); ++k) gridLayoutDisplayPanel->addWidget(pushButtonActions[k], k, 0);
		gridLayoutDisplayPanel->addWidget(labelVis, 0, 1, int(pushButtonActions.size()), 1);
		{
			gridLayoutDisplayPanel->setColumnStretch(0, 0);
			gridLayoutDisplayPanel->setColumnStretch(1, 1);
			for (int k = 0; k < pushButtonActions.size(); ++k) connect(pushButtonActions[k], &QPushButton::pressed, [this, k]()->void
				{
					++actions[k];
					groupBoxDisplayPanel->setTitle(QString("DisplayPanel: action%1=%2").arg(k).arg(actions[k]));
				});
		}
	}

public:
	QVBoxLayout* vboxLayoutMain = new QVBoxLayout(this);
	QGroupBox* groupBoxControlPanel = new QGroupBox("ControlPanel: ", this);
	QGroupBox* groupBoxDisplayPanel = new QGroupBox("DisplayPanel: ", this);

	QGridLayout* gridLayoutControlPanel = new QGridLayout(groupBoxControlPanel);
	vector<QComboBox*> comboBoxMotorModes = { new QComboBox(groupBoxControlPanel), new QComboBox(groupBoxControlPanel), new QComboBox(groupBoxControlPanel) };
	vector<QDoubleSpinBox*> doubleSpinBoxMotorRanges =
	{
		new QDoubleSpinBox(groupBoxControlPanel), new QDoubleSpinBox(groupBoxControlPanel), new QDoubleSpinBox(groupBoxControlPanel),
		new QDoubleSpinBox(groupBoxControlPanel), new QDoubleSpinBox(groupBoxControlPanel), new QDoubleSpinBox(groupBoxControlPanel)
	};
	vector<QPushButton*> pushButtonMotorPlus =
	{
		new QPushButton("Motor0+", groupBoxControlPanel), new QPushButton("Motor1+", groupBoxControlPanel), new QPushButton("Motor2+", groupBoxControlPanel),
		new QPushButton("Motor3+", groupBoxControlPanel), new QPushButton("Motor4+", groupBoxControlPanel), new QPushButton("Motor5+", groupBoxControlPanel)
	};
	vector<QPushButton*> pushButtonMotorMinus =
	{
		new QPushButton("Motor0-", groupBoxControlPanel), new QPushButton("Motor1-", groupBoxControlPanel), new QPushButton("Motor2-", groupBoxControlPanel),
		new QPushButton("Motor3-", groupBoxControlPanel), new QPushButton("Motor4-", groupBoxControlPanel), new QPushButton("Motor5-", groupBoxControlPanel)
	};

	QGridLayout* gridLayoutDisplayPanel = new QGridLayout(groupBoxDisplayPanel);
	vector<QPushButton*> pushButtonActions = { new QPushButton("Action0", groupBoxDisplayPanel), new QPushButton("Action1", groupBoxDisplayPanel) };
	QLabel* labelVis = new QLabel("This label can be string or image", groupBoxDisplayPanel);
};

//1.URE environment
//1.1 New world and save as your expected name
//1.2 Add WebotsProjects->objects->backgrounds->TextureBackground
//1.3 Add WebotsProjects->objects->backgrounds->TextureBackgroundLight
//1.4 Add WebotsProjects->objects->robotstadium->RobotstadiumSoccerField
//1.5 Add WebotsProjects->robots->universal_robots->UR*E and set: controller=<extern>, translationY=0.6

//2.Cam environment
//2.1 Add four BaseNodes->Camera to UR*E->toolSlot and set their names: camera0, camera1, camera2, camera3
//2.2 Set camera0: rotation=0 1 -1 3.14   width=640   height=480   fieldOfView=1.57
//2.2 Set camera1: rotation=1 -1 1 -2.1   translation=-0.05 0 0   width=640   height=480   fieldOfView=1.57
//2.3 Set camera2: rotation=1 0 0 -1.57   translation=0 -0.2 0   width=640   height=480   fieldOfView=1.57
//2.4 Set camera3: rotation=1 1 -1 -2.1   translation=0.05 0 0   width=640   height=480   fieldOfView=1.57
//2.5 Add WebotsProjects->objects->chairs->WoodenChair and set: translation=3 0 -2
//2.6 Add WebotsProjects->objects->garden->DogHouse and set: translation=-1.5 0 2
int main(int argc, char** argv)
{
	//MearmRC::TestGUI(argc, argv); return 0;
	//MearmRC::TestURE(argc, argv); return 0;

	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const int camTimeStep = timeStep * 2;
	vector<string> camsName = { "camera0", "camera1", "camera2", "camera3" };
	if (argc < 2) { camsName.clear(); spdlog::warn("Camera devices disabled"); }
	vector<string> motorsName = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
	const int64 timestart = ms1970;
	utils::fs::createDirectories(fmt::format("./{}", timestart));

	//1.GetDevices
	Robot wbrobot;
	vector<Camera*> wbcams(camsName.size()); for (int k = 0; k < wbcams.size(); ++k) wbcams[k] = wbrobot.getCamera(camsName[k]);
	vector<Motor*> wbmotors(motorsName.size()); for (int k = 0; k < wbmotors.size(); ++k) wbmotors[k] = wbrobot.getMotor(motorsName[k]);

	//2.EnableDevices
	for (int k = 0; k < wbcams.size(); ++k) wbcams[k]->enable(camTimeStep);
	for (int k = 0; k < wbmotors.size(); ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0); }

	//3.StartTask
	int ctlCode = 0;
	QApplication qtApp(argc, argv);
	MearmRC mearmRC; mearmRC.show();
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept
		vector<Mat_<Vec4b>> wbcamsData(wbcams.size());
		for (int k = 0; k < wbcamsData.size(); ++k)
		{
			wbcamsData[k].create(wbcams[k]->getHeight(), wbcams[k]->getWidth());
			memcpy(wbcamsData[k].data, wbcams[k]->getImage(), wbcamsData[k].total() * wbcamsData[k].step[1]);
		}

		//3.2 Control
		for (int k = 0; k < wbmotors.size(); ++k) wbmotors[k]->setVelocity(mearmRC.motors[k]);

		//3.3 ShowResults
		QApplication::processEvents();
		if (wbcamsData.empty()) continue;
		Mat allIma; hconcat(wbcamsData, allIma);
		if (mearmRC.nspace % 2)
		{
			int64 name = ns1970;
			for (int k = 0; k < wbcamsData.size(); ++k) { utils::fs::createDirectories(fmt::format("./{}/cam{}", timestart, k)); cv::imwrite(fmt::format("./{}/cam{}/{}.png", timestart, k, name), wbcamsData[k]); }
			utils::fs::createDirectories(fmt::format("./{}/all", timestart)); cv::imwrite(fmt::format("./{}/all/{}.png", timestart, name), allIma);
			++mearmRC.nspace;
		}
		cvtColor(allIma, allIma, COLOR_BGRA2RGBA);
		mearmRC.labelVis->setScaledContents(true);
		mearmRC.labelVis->setPixmap(QPixmap::fromImage(QImage(allIma.data, allIma.cols, allIma.rows, QImage::Format_RGBA8888)));
	}
}