#include "ConnectionTab.h"

ConnectionTab::ConnectionTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent)
{
	m_ui = ui;
	m_Robot = dynamic_cast<LancetJakaRobot*>( aRobot);

	m_Camera = aCamera;
	m_dataStorage = aDataStorage;
	if (!m_dataStorage)
	{
		std::cout << "m_dataStorage is nullptr" << std::endl;
	}//here
	InitConnection();
}


void ConnectionTab::InitConnection()
{
	connect(m_ui.pushButton_connectArm_3, &QPushButton::clicked, this, &ConnectionTab::connectRobot);
	connect(m_ui.pushButton_powerOn_3, &QPushButton::clicked, this, &ConnectionTab::powerOn);
	connect(m_ui.pushButton_powerOff_3, &QPushButton::clicked, this, &ConnectionTab::powerOff);
	connect(m_ui.pushButton_connectCamrea_3, &QPushButton::clicked, this, &ConnectionTab::connectCamera);
	connect(m_Camera, &AimCamera::CameraUpdateClock, this, &ConnectionTab::upDateUi);
	connect(m_ui.pushButton_updataData_3, &QPushButton::clicked, this, &ConnectionTab::updateData);
}


void ConnectionTab::connectRobot()
{
	m_Robot->Connect();
}
void ConnectionTab::powerOn()
{
	m_Robot->PowerOn();
}



/**
 * @brief 
 * @param tempTip 
 * @param label 
*/
void ConnectionTab::flashLable(Eigen::Vector3d tempTip, QLabel* label)
{
	if (tempTip[0]==0 && tempTip[1] == 0 && tempTip[2] == 0)
	{
		if (label != nullptr)
		{
			QString str = "x:0  y:0 z:0";
			label->setText(str);
			/*std::cout << str << std::endl;*/
			label->setStyleSheet("QLabel { color : red; }");
		}
	}
	else 
	{
		if (label != nullptr)
		{
			QString str = "x:" + QString::number(tempTip[0]) + " "
				+ "y:" + QString::number(tempTip[1]) + " "
				+ "z:" + QString::number(tempTip[2]);
			label->setText(str);
			label->setStyleSheet("QLabel { color : green; }");
			/*std::cout << str << std::endl;*/
			
		}
	}
	
}
void ConnectionTab::upDateUi()
{
	//AimCamera::GetToolMatrixByName
	flashLable(m_Camera->GetToolTipByName("RobotBaseRF"),m_ui.Spine_RobotBaseRFDataLabel_3);
	flashLable(m_Camera->GetToolTipByName("RobotEndRF"), m_ui.Spine_RobotEndRFDataLabel_3);
	flashLable(m_Camera->GetToolTipByName("PatientRF"), m_ui.Spine_PatientRFDataLabel_3);
	flashLable(m_Camera->GetToolTipByName("Probe"), m_ui.Spine_ProbeDataLabel_3);
}



void ConnectionTab::powerOff()
{
	m_Robot->PowerOff();
}

void ConnectionTab::connectCamera()
{
	m_Camera->Connect();
}
void ConnectionTab::updateData()
{
	std::vector<std::string> tempToolsName = {
	"RobotBaseRF",
	"RobotEndRF",
	"PatientRF",
	"Probe"
	};
	m_Camera->InitToolsName(tempToolsName);
	m_Camera->Start();
}
