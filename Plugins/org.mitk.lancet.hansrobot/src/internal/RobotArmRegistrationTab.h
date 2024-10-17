#pragma once
#include <qwidget.h>
#include <qobject.h>
#include "ui_HansRobotControls.h"
#include "LancetJakaRobot.h"
#include "FileIO.h"
#include "AimCamera.h"

#include <QmitkSingleNodeSelectionWidget.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateProperty.h>
#include <mitkNodePredicateCompositeBase.h>
#include <mitkSurface.h>
#include <mitkNodePredicateBase.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateOr.h>
#include <mitkColorProperty.h>
#include "robotRegistration.h"
#include <vtkMatrix4x4.h>

#include <LancetRobotRegistration.h>
class RobotArmRegistrationTab : public QWidget
{

	Q_OBJECT
public:
	int countNum;
public:
	RobotArmRegistrationTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent = nullptr);

public slots:
	void updateUiCaputreCount(int cnt);

	//void flashLable(Ui::HansRobotControls* lable);
private:
	void xp();
	void yp();
	void zp();
	void xm();
	void ym();
	void zm();
	void rxp();
	void ryp();
	void rzp();
	void rxm();
	void rym();
	void rzm();

	void captureRobot();
	void CapturePose(bool);
	void waitMove();
	void autoCollection();
	
	void RepeatPositionTest();
	void AboslutePositionTest();
private:
	QWidget* m_TabPage;
	Ui::HansRobotControls m_ui;
	//AbstractRobot* m_Robot;
	LancetJakaRobot* m_Robot;
	mitk::DataStorage* m_dataStorage;
	AbstractCamera* m_Camera;
	RobotRegistration m_RobotRegistration;
	bool isAutoCollectionFlag;
	//void flashLable(Eigen::Vector3d tempTip, QLabel* label);

	LancetRobotRegistration* m_LancetRobotRegistration;
};