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

class WbtRC : public QWidget
{
public:
	static void TestMe(int argc = 0, char** argv = 0)
	{
		QApplication app(argc, argv);
		WbtRC me;
		me.show();
		while (1)
		{
			QApplication::processEvents();
			this_thread::sleep_for(100ms);
		}
	}

public:
	double fb = 0;
	double lr = 0;
	double ud = 0;
	double pitch = 0;
	double yaw = 0;
	double roll = 0;

public:
	WbtRC(QWidget* parent = 0) : QWidget(parent)
	{

		//0.Basic settting
		this->setWindowTitle("Super Cube");
		this->setMinimumSize(QSize(1200, 450));
		this->setFont(QFont("", 15, QFont::Thin));

		//1.Group1 setting
		hboxLayoutMain->addWidget(groupBoxTrans);
		hboxLayoutMain->addWidget(groupBoxInteraction);
		hboxLayoutMain->addWidget(groupBoxRotation);
		{
			hboxLayoutMain->setStretch(1, 1);
			QList<QPushButton*> children1 = groupBoxTrans->findChildren<QPushButton*>();
			QList<QPushButton*> children2 = groupBoxRotation->findChildren<QPushButton*>();
			for (int k = 0; k < children1.size() - 2; ++k) children1[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			for (int k = 0; k < children2.size() - 2; ++k) children2[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			comboBoxTransMode->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			comboBoxRotationMode->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
		}

		//2.Group2 setting
		gridLayoutTrans->addWidget(doubleSpinBoxTransFB, 0, 0);
		gridLayoutTrans->addWidget(doubleSpinBoxTransLR, 0, 1);
		gridLayoutTrans->addWidget(doubleSpinBoxTransUD, 0, 2);
		gridLayoutTrans->addWidget(pushButtonForward, 1, 1);
		gridLayoutTrans->addWidget(pushButtonLeftward, 2, 0);
		gridLayoutTrans->addWidget(comboBoxTransMode, 2, 1);
		gridLayoutTrans->addWidget(pushButtonRightward, 2, 2);
		gridLayoutTrans->addWidget(pushButtonBackward, 3, 1);
		gridLayoutTrans->addWidget(pushButtonDownward, 4, 0);
		gridLayoutTrans->addWidget(pushButtonUpward, 4, 2);
		{
			doubleSpinBoxTransFB->setRange(0, 100); doubleSpinBoxTransFB->setValue(0.5);
			doubleSpinBoxTransLR->setRange(0, 100); doubleSpinBoxTransLR->setValue(0.5);
			doubleSpinBoxTransUD->setRange(0, 100); doubleSpinBoxTransUD->setValue(0.5);
			connect(pushButtonForward, &QPushButton::pressed, [this]()->void { fb = doubleSpinBoxTransFB->value(); groupBoxInteraction->setTitle(QString("Send: fb=%1").arg(fb)); });
			connect(pushButtonBackward, &QPushButton::pressed, [this]()->void { fb = -doubleSpinBoxTransFB->value(); groupBoxInteraction->setTitle(QString("Send: fb=%1").arg(fb)); });
			connect(pushButtonLeftward, &QPushButton::pressed, [this]()->void { lr = -doubleSpinBoxTransLR->value(); groupBoxInteraction->setTitle(QString("Send: lr=%1").arg(lr)); });
			connect(pushButtonRightward, &QPushButton::pressed, [this]()->void { lr = doubleSpinBoxTransLR->value(); groupBoxInteraction->setTitle(QString("Send: lr=%1").arg(lr)); });
			connect(pushButtonUpward, &QPushButton::pressed, [this]()->void { ud = doubleSpinBoxTransUD->value(); groupBoxInteraction->setTitle(QString("Send: ud=%1").arg(ud)); });
			connect(pushButtonDownward, &QPushButton::pressed, [this]()->void { ud = -doubleSpinBoxTransUD->value(); groupBoxInteraction->setTitle(QString("Send: ud=%1").arg(ud)); });
			connect(pushButtonForward, &QPushButton::released, [this]()->void { fb = 0; groupBoxInteraction->setTitle(QString("Send: fb=%1").arg(fb)); });
			connect(pushButtonBackward, &QPushButton::released, [this]()->void { fb = 0; groupBoxInteraction->setTitle(QString("Send: fb=%1").arg(fb)); });
			connect(pushButtonLeftward, &QPushButton::released, [this]()->void { lr = 0; groupBoxInteraction->setTitle(QString("Send: lr=%1").arg(lr)); });
			connect(pushButtonRightward, &QPushButton::released, [this]()->void { lr = 0; groupBoxInteraction->setTitle(QString("Send: lr=%1").arg(lr)); });
			connect(pushButtonUpward, &QPushButton::released, [this]()->void { ud = 0; groupBoxInteraction->setTitle(QString("Send: ud=%1").arg(ud)); });
			connect(pushButtonDownward, &QPushButton::released, [this]()->void { ud = 0; groupBoxInteraction->setTitle(QString("Send: ud=%1").arg(ud)); });
			connect(comboBoxTransMode, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)->void { });
		}

		//3.Group3 setting
		gridLayoutRotation->addWidget(doubleSpinBoxRotationPitch, 0, 0);
		gridLayoutRotation->addWidget(doubleSpinBoxRotationYaw, 0, 1);
		gridLayoutRotation->addWidget(doubleSpinBoxRotationRoll, 0, 2);
		gridLayoutRotation->addWidget(pushButtonMinusPitch, 1, 1);
		gridLayoutRotation->addWidget(pushButtonMinusYaw, 2, 0);
		gridLayoutRotation->addWidget(comboBoxRotationMode, 2, 1);
		gridLayoutRotation->addWidget(pushButtonPlusYaw, 2, 2);
		gridLayoutRotation->addWidget(pushButtonPlusPitch, 3, 1);
		gridLayoutRotation->addWidget(pushButtonMinusRoll, 4, 0);
		gridLayoutRotation->addWidget(pushButtonPlusRoll, 4, 2);
		{
			doubleSpinBoxRotationPitch->setRange(0, 100); doubleSpinBoxRotationPitch->setValue(0.5);
			doubleSpinBoxRotationYaw->setRange(0, 100); doubleSpinBoxRotationYaw->setValue(0.5);
			doubleSpinBoxRotationRoll->setRange(0, 100); doubleSpinBoxRotationRoll->setValue(0.5);
			connect(pushButtonMinusPitch, &QPushButton::pressed, [this]()->void { pitch = -doubleSpinBoxRotationPitch->value(); groupBoxInteraction->setTitle(QString("Send: pitch=%1").arg(pitch)); });
			connect(pushButtonPlusPitch, &QPushButton::pressed, [this]()->void { pitch = doubleSpinBoxRotationPitch->value(); groupBoxInteraction->setTitle(QString("Send: pitch=%1").arg(pitch)); });
			connect(pushButtonMinusYaw, &QPushButton::pressed, [this]()->void { yaw = -doubleSpinBoxRotationYaw->value(); groupBoxInteraction->setTitle(QString("Send: yaw=%1").arg(yaw)); });
			connect(pushButtonPlusYaw, &QPushButton::pressed, [this]()->void { yaw = doubleSpinBoxRotationYaw->value(); groupBoxInteraction->setTitle(QString("Send: yaw=%1").arg(yaw)); });
			connect(pushButtonMinusRoll, &QPushButton::pressed, [this]()->void { roll = -doubleSpinBoxRotationRoll->value(); groupBoxInteraction->setTitle(QString("Send: roll=%1").arg(roll)); });
			connect(pushButtonPlusRoll, &QPushButton::pressed, [this]()->void { roll = doubleSpinBoxRotationRoll->value(); groupBoxInteraction->setTitle(QString("Send: roll=%1").arg(roll)); });
			connect(pushButtonMinusPitch, &QPushButton::released, [this]()->void { pitch = 0; groupBoxInteraction->setTitle(QString("Send: pitch=%1").arg(pitch)); });
			connect(pushButtonPlusPitch, &QPushButton::released, [this]()->void { pitch = 0; groupBoxInteraction->setTitle(QString("Send: pitch=%1").arg(pitch)); });
			connect(pushButtonMinusYaw, &QPushButton::released, [this]()->void { yaw = 0; groupBoxInteraction->setTitle(QString("Send: yaw=%1").arg(yaw)); });
			connect(pushButtonPlusYaw, &QPushButton::released, [this]()->void { yaw = 0; groupBoxInteraction->setTitle(QString("Send: yaw=%1").arg(yaw)); });
			connect(pushButtonMinusRoll, &QPushButton::released, [this]()->void { roll = 0; groupBoxInteraction->setTitle(QString("Send: roll=%1").arg(roll)); });
			connect(pushButtonPlusRoll, &QPushButton::released, [this]()->void { roll = 0; groupBoxInteraction->setTitle(QString("Send: roll=%1").arg(roll)); });
			connect(comboBoxRotationMode, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)->void {});
		}

		//4.Group4 setting
		gridLayoutInteraction->addWidget(labelFPV, 0, 0);
	}

public:
	QHBoxLayout* hboxLayoutMain = new QHBoxLayout(this);
	QGroupBox* groupBoxTrans = new QGroupBox("Trans", this);
	QGroupBox* groupBoxRotation = new QGroupBox("Rotation", this);
	QGroupBox* groupBoxInteraction = new QGroupBox("Send: ", this);

	QGridLayout* gridLayoutTrans = new QGridLayout(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransFB = new QDoubleSpinBox(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransLR = new QDoubleSpinBox(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransUD = new QDoubleSpinBox(groupBoxTrans);
	QComboBox* comboBoxTransMode = new QComboBox(groupBoxTrans);
	QPushButton* pushButtonForward = new QPushButton("Forward", groupBoxTrans);
	QPushButton* pushButtonBackward = new QPushButton("Backward", groupBoxTrans);
	QPushButton* pushButtonLeftward = new QPushButton("Leftward", groupBoxTrans);
	QPushButton* pushButtonRightward = new QPushButton("Rightward", groupBoxTrans);
	QPushButton* pushButtonUpward = new QPushButton("Upward", groupBoxTrans);
	QPushButton* pushButtonDownward = new QPushButton("Downward", groupBoxTrans);

	QGridLayout* gridLayoutRotation = new QGridLayout(groupBoxRotation);
	QDoubleSpinBox* doubleSpinBoxRotationPitch = new QDoubleSpinBox(groupBoxRotation);
	QDoubleSpinBox* doubleSpinBoxRotationYaw = new QDoubleSpinBox(groupBoxRotation);
	QDoubleSpinBox* doubleSpinBoxRotationRoll = new QDoubleSpinBox(groupBoxRotation);
	QComboBox* comboBoxRotationMode = new QComboBox(groupBoxRotation);
	QPushButton* pushButtonMinusPitch = new QPushButton("MinusPitch", groupBoxRotation);
	QPushButton* pushButtonPlusPitch = new QPushButton("PlusPitch", groupBoxRotation);
	QPushButton* pushButtonMinusYaw = new QPushButton("MinusYaw", groupBoxRotation);
	QPushButton* pushButtonPlusYaw = new QPushButton("PlusYaw", groupBoxRotation);
	QPushButton* pushButtonMinusRoll = new QPushButton("MinusRoll", groupBoxRotation);
	QPushButton* pushButtonPlusRoll = new QPushButton("PlusRoll", groupBoxRotation);

	QGridLayout* gridLayoutInteraction = new QGridLayout(groupBoxInteraction);
	QLabel* labelFPV = new QLabel("(1)This label is for caller\n\n(2)This label can be string or image", groupBoxInteraction);
};

//You can add UR3E/UR5E/UR10E to any world to test this remote control.
int main(int argc, char** argv)
{
	//WbtRC::TestMe(); return 0;

	//0.GetConstant
	using namespace webots;
	const int timeStep = 64;
	const int64 timestart = ms1970;
	//utils::fs::createDirectories(fmt::format("./{}", timestart));
	string motorsName[6] = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

	//1.GetDevices
	Robot wbrobot;
	Motor* wbmotors[6]; for (int k = 0; k < 6; ++k) wbmotors[k] = wbrobot.getMotor(motorsName[k]);

	//2.EnableDevices
	for (int k = 0; k < 6; ++k) { wbmotors[k]->setPosition(INFINITY); wbmotors[k]->setVelocity(0); }

	//3.StartTask
	int ctlCode = 0;
	QApplication qtApp(argc, argv);
	WbtRC wbtRC; wbtRC.show();
	while (wbrobot.step(timeStep) != -1)
	{
		//3.1 Percept

		//3.2 Control
		wbmotors[0]->setVelocity(wbtRC.lr);
		wbmotors[1]->setVelocity(wbtRC.fb);
		wbmotors[2]->setVelocity(wbtRC.ud);
		wbmotors[3]->setVelocity(wbtRC.roll);
		wbmotors[4]->setVelocity(wbtRC.pitch);
		wbmotors[5]->setVelocity(wbtRC.yaw);

		//3.3 ShowResults
		QApplication::processEvents();
	}
}