#include <opencv2/opencv.hpp>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
using namespace std;
using namespace cv;

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
		this->setMinimumSize(QSize(1280, 480));
		this->setFont(QFont("", 15, QFont::Thin));

		//1.Group1 setting
		hboxLayoutMain->addWidget(groupBoxTrans);
		hboxLayoutMain->addWidget(groupBoxStatus);
		hboxLayoutMain->addWidget(groupBoxRotation);
		{
			hboxLayoutMain->setStretch(1, 1);
			QList<QPushButton*> children1 = groupBoxTrans->findChildren<QPushButton*>();
			QList<QWidget*> children2 = groupBoxStatus->findChildren<QWidget*>();
			QList<QPushButton*> children3 = groupBoxRotation->findChildren<QPushButton*>();
			for (int k = 0; k < children1.size() - 2; ++k) children1[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			for (int k = 0; k < children2.size(); ++k) children2[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			for (int k = 0; k < children3.size() - 2; ++k) children3[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
		}

		//2.Group2 setting
		gridLayoutTrans->addWidget(doubleSpinBoxTransFB, 0, 0);
		gridLayoutTrans->addWidget(doubleSpinBoxTransLR, 0, 1);
		gridLayoutTrans->addWidget(doubleSpinBoxTransUD, 0, 2);
		gridLayoutTrans->addWidget(pushButtonForward, 1, 1);
		gridLayoutTrans->addWidget(pushButtonLeftward, 2, 0);
		gridLayoutTrans->addWidget(pushButtonRightward, 2, 2);
		gridLayoutTrans->addWidget(pushButtonBackward, 3, 1);
		gridLayoutTrans->addWidget(pushButtonDownward, 4, 0);
		gridLayoutTrans->addWidget(pushButtonUpward, 4, 2);
		{
			doubleSpinBoxTransFB->setRange(0, 100); doubleSpinBoxTransFB->setValue(0.1);
			doubleSpinBoxTransLR->setRange(0, 100); doubleSpinBoxTransLR->setValue(0.1);
			doubleSpinBoxTransUD->setRange(0, 100); doubleSpinBoxTransUD->setValue(0.1);
			connect(pushButtonForward, &QPushButton::pressed, [this]()->void { fb = doubleSpinBoxTransFB->value(); labelStatus->setText(QString("fb=%1").arg(fb)); });
			connect(pushButtonBackward, &QPushButton::pressed, [this]()->void { fb = -doubleSpinBoxTransFB->value(); labelStatus->setText(QString("fb=%1").arg(fb)); });
			connect(pushButtonLeftward, &QPushButton::pressed, [this]()->void { lr = -doubleSpinBoxTransLR->value(); labelStatus->setText(QString("lr=%1").arg(lr)); });
			connect(pushButtonRightward, &QPushButton::pressed, [this]()->void { lr = doubleSpinBoxTransLR->value(); labelStatus->setText(QString("lr=%1").arg(lr)); });
			connect(pushButtonUpward, &QPushButton::pressed, [this]()->void { ud = doubleSpinBoxTransUD->value(); labelStatus->setText(QString("ud=%1").arg(ud)); });
			connect(pushButtonDownward, &QPushButton::pressed, [this]()->void { ud = -doubleSpinBoxTransUD->value(); labelStatus->setText(QString("ud=%1").arg(ud)); });
			connect(pushButtonForward, &QPushButton::released, [this]()->void { fb = 0; labelStatus->setText(QString("fb=%1").arg(fb)); });
			connect(pushButtonBackward, &QPushButton::released, [this]()->void { fb = 0; labelStatus->setText(QString("fb=%1").arg(fb)); });
			connect(pushButtonLeftward, &QPushButton::released, [this]()->void { lr = 0; labelStatus->setText(QString("lr=%1").arg(lr)); });
			connect(pushButtonRightward, &QPushButton::released, [this]()->void { lr = 0; labelStatus->setText(QString("lr=%1").arg(lr)); });
			connect(pushButtonUpward, &QPushButton::released, [this]()->void { ud = 0; labelStatus->setText(QString("ud=%1").arg(ud)); });
			connect(pushButtonDownward, &QPushButton::released, [this]()->void { ud = 0; labelStatus->setText(QString("ud=%1").arg(ud)); });
		}

		//3.Group3 setting
		gridLayoutRotation->addWidget(doubleSpinBoxRotationPitch, 0, 0);
		gridLayoutRotation->addWidget(doubleSpinBoxRotationYaw, 0, 1);
		gridLayoutRotation->addWidget(doubleSpinBoxRotationRoll, 0, 2);
		gridLayoutRotation->addWidget(pushButtonMinusPitch, 1, 1);
		gridLayoutRotation->addWidget(pushButtonMinusYaw, 2, 0);
		gridLayoutRotation->addWidget(pushButtonPlusYaw, 2, 2);
		gridLayoutRotation->addWidget(pushButtonPlusPitch, 3, 1);
		gridLayoutRotation->addWidget(pushButtonMinusRoll, 4, 0);
		gridLayoutRotation->addWidget(pushButtonPlusRoll, 4, 2);
		{
			doubleSpinBoxRotationPitch->setRange(0, 100); doubleSpinBoxRotationPitch->setValue(0.1);
			doubleSpinBoxRotationYaw->setRange(0, 100); doubleSpinBoxRotationYaw->setValue(0.1);
			doubleSpinBoxRotationRoll->setRange(0, 100); doubleSpinBoxRotationRoll->setValue(0.1);
			connect(pushButtonMinusPitch, &QPushButton::pressed, [this]()->void { pitch = -doubleSpinBoxRotationPitch->value(); labelStatus->setText(QString("pitch=%1").arg(pitch)); });
			connect(pushButtonPlusPitch, &QPushButton::pressed, [this]()->void { pitch = doubleSpinBoxRotationPitch->value(); labelStatus->setText(QString("pitch=%1").arg(pitch)); });
			connect(pushButtonMinusYaw, &QPushButton::pressed, [this]()->void { yaw = -doubleSpinBoxRotationYaw->value(); labelStatus->setText(QString("yaw=%1").arg(yaw)); });
			connect(pushButtonPlusYaw, &QPushButton::pressed, [this]()->void { yaw = doubleSpinBoxRotationYaw->value(); labelStatus->setText(QString("yaw=%1").arg(yaw)); });
			connect(pushButtonMinusRoll, &QPushButton::pressed, [this]()->void { roll = -doubleSpinBoxRotationRoll->value(); labelStatus->setText(QString("roll=%1").arg(roll)); });
			connect(pushButtonPlusRoll, &QPushButton::pressed, [this]()->void { roll = doubleSpinBoxRotationRoll->value(); labelStatus->setText(QString("roll=%1").arg(roll)); });
			connect(pushButtonMinusPitch, &QPushButton::released, [this]()->void { pitch = 0; labelStatus->setText(QString("pitch=%1").arg(pitch)); });
			connect(pushButtonPlusPitch, &QPushButton::released, [this]()->void { pitch = 0; labelStatus->setText(QString("pitch=%1").arg(pitch)); });
			connect(pushButtonMinusYaw, &QPushButton::released, [this]()->void { yaw = 0; labelStatus->setText(QString("yaw=%1").arg(yaw)); });
			connect(pushButtonPlusYaw, &QPushButton::released, [this]()->void { yaw = 0; labelStatus->setText(QString("yaw=%1").arg(yaw)); });
			connect(pushButtonMinusRoll, &QPushButton::released, [this]()->void { roll = 0; labelStatus->setText(QString("roll=%1").arg(roll)); });
			connect(pushButtonPlusRoll, &QPushButton::released, [this]()->void { roll = 0; labelStatus->setText(QString("roll=%1").arg(roll)); });
		}

		//4.Group4 setting
		gridLayoutStatus->addWidget(labelStatus, 0, 0);
	}

public:
	QHBoxLayout* hboxLayoutMain = new QHBoxLayout(this);
	QGroupBox* groupBoxTrans = new QGroupBox("Trans", this);
	QGroupBox* groupBoxStatus = new QGroupBox("Status", this);
	QGroupBox* groupBoxRotation = new QGroupBox("Rotation", this);

	QGridLayout* gridLayoutTrans = new QGridLayout(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransFB = new QDoubleSpinBox(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransLR = new QDoubleSpinBox(groupBoxTrans);
	QDoubleSpinBox* doubleSpinBoxTransUD = new QDoubleSpinBox(groupBoxTrans);
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
	QPushButton* pushButtonMinusPitch = new QPushButton("MinusPitch", groupBoxRotation);
	QPushButton* pushButtonPlusPitch = new QPushButton("PlusPitch", groupBoxRotation);
	QPushButton* pushButtonMinusYaw = new QPushButton("MinusYaw", groupBoxRotation);
	QPushButton* pushButtonPlusYaw = new QPushButton("PlusYaw", groupBoxRotation);
	QPushButton* pushButtonMinusRoll = new QPushButton("MinusRoll", groupBoxRotation);
	QPushButton* pushButtonPlusRoll = new QPushButton("PlusRoll", groupBoxRotation);

	QGridLayout* gridLayoutStatus = new QGridLayout(groupBoxStatus);
	QLabel* labelStatus = new QLabel("status", groupBoxStatus);
};

int main(int argc, char** argv) { WbtRC::TestMe(argc, argv); }