#include <LancetRobotRegistration.h>

LancetRobotRegistration::LancetRobotRegistration(AbstractRobot* aRobot, AbstractCamera* aCamera)
{
	m_Robot = aRobot;  
	m_Camera = aCamera; 
}

void LancetRobotRegistration::setTCPToFlange()
{
	m_Robot->SetTCPToFlange();
}

void LancetRobotRegistration::recordInitialPos()
{
	m_Robot->RecordInitialPos();
}

void LancetRobotRegistration::goToInitialPos()
{
	m_Robot->GoToInitialPos();
}

void LancetRobotRegistration::xp()
{
	double templeArray[3] = { 1, 0.0, 0.0 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::yp()
{
	double templeArray[3] = { 0.0, 1, 0.0 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::zp()
{
	double templeArray[3] = { 0.0, 0.0, 1 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::xm()
{
	double templeArray[3] = { -1, 0.0, 0.0 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::ym()
{
	double templeArray[3] = { 0.0, -1, 0.0 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::zm()
{
	double templeArray[3] = { 0.0, 0.0, -1 };
	m_Robot->Translate(templeArray, Distance);
}

void LancetRobotRegistration::rxp()
{
	double templeArray[3] = { 1, 0.0, 0.0 };
	m_Robot->Rotate(templeArray, Angle);
}

void LancetRobotRegistration::ryp()
{
	double templeArray[3] = { 0.0, 1, 0.0 };
	m_Robot->Rotate(templeArray, Angle);
}

void LancetRobotRegistration::rzp()
{
	double templeArray[3] = { 0.0, 0.0, 1 };
	m_Robot->Rotate(templeArray, Angle);
}

void LancetRobotRegistration::rxm()
{
	double templeArray[3] = { -1, 0.0, 0.0 };
	m_Robot->Rotate(templeArray, Angle);
}

void LancetRobotRegistration::rym()
{
	double templeArray[3] = { 0.0, -1, 0.0 };
	m_Robot->Rotate(templeArray, Angle);
}

void LancetRobotRegistration::rzm()
{
	double templeArray[3] = { 0.0, 0.0, -1 };
	m_Robot->Rotate(templeArray, Angle);
}

int LancetRobotRegistration::captureRobot()
{
	if (m_RobotRegistration.PoseCount() < 5) //The first five translations, 
	{
		capturePose(true);
	}
	else if (m_RobotRegistration.PoseCount() >= 5 && m_RobotRegistration.PoseCount() < 10)
	{
		capturePose(false);
	}
	else
	{
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		m_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);
		robotEndToFlangeMatrix->Invert();
		std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;
	}
	return m_RobotRegistration.PoseCount();
}

void LancetRobotRegistration::capturePose(bool translationOnly)
{
	std::cout << "CapturePose" <<std::endl;
	//get T_BaseToFlanger
	vtkSmartPointer<vtkMatrix4x4>T_BaseToFlange = m_Robot->GetBaseToFlange();
	PrintDataHelper::CoutMatrix("T_BaseToFlange", T_BaseToFlange);
	//get T_CameraToEnd
	vtkSmartPointer<vtkMatrix4x4>T_CameraToEnd = m_Camera->GetToolMatrixByName("RobotEndRF");
	PrintDataHelper::CoutMatrix("T_CameraToEnd", T_CameraToEnd);
	//get T_BaseRFToCamera
	vtkSmartPointer<vtkMatrix4x4>T_BaseRFToCamera= m_Camera->GetToolMatrixByName("RobotBaseRF");
	T_BaseRFToCamera->Invert();

	PrintDataHelper::CoutMatrix("T_BaseRFToCamera", T_BaseRFToCamera);
	//get T_BaseRFToEnd
	vtkSmartPointer<vtkMatrix4x4> T_BaseRFToEnd= vtkSmartPointer<vtkMatrix4x4> ::New();

	vtkMatrix4x4::Multiply4x4(T_BaseRFToCamera, T_CameraToEnd, T_BaseRFToEnd);

	////Robotic arm registration
	m_RobotRegistration.AddPoseWithVtkMatrix(T_BaseToFlange, T_BaseRFToEnd, translationOnly);
	emit countPose(m_RobotRegistration.PoseCount());
}

void LancetRobotRegistration::waitMove()
{
	std::cout << "waitMove" << std::endl;
	bool bDone = false;
	while (!bDone)
	{
		int nRet = HRIF_IsMotionDone(0, 0, bDone);
		if (bDone == false)
		{
			QThread::msleep(10);
			QApplication::processEvents();
			std::cout << "is still moving" << std::endl;
		}
		else
		{
			int ret = HRIF_GrpStop(0, 0);
			std::cout << "robot is stop please continue" << std::endl;
			return;
		}
	}
}
void LancetRobotRegistration::Sleep(int msec)
{
	QTime dieTime = QTime::currentTime().addMSecs(msec);

	while (QTime::currentTime() < dieTime)

		QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
void LancetRobotRegistration::autoCollection()
{
	replaceRegistration();
	setTCPToFlange();
	recordInitialPos();
	std::cout << "Starting automatic registration!" << std::endl;
	int moveCount = 1;
	m_RobotRegistration.RemoveAllPose();
	if (isAutoCollectionFlag)
	{
		m_Robot->SetTCPToFlange();
		isAutoCollectionFlag = FALSE;
	}
	while (moveCount <= 10)
	{
		m_Robot->GoToInitialPos();
		waitMove();
		switch (moveCount) {
		case 1:
			xp();
			break;
		case 2:
			yp();
			break;
		case 3:
			zp();
			break;
		case 4:
			xm();
			break;
		case 5:
			ym();
			break;
		case 6:
			rxp();
			break;
		case 7:
			ryp();
			break;
		case 8:
			rzp();
			break;
		case 9:
			rxm();
			break;
		case 10:
			rym();
			break;
		default:
			return;
		}
		waitMove();
		Sleep(1000);
		m_Camera->UpdateData();
		captureRobot();
		Sleep(1000);
		moveCount++;
	}
	if (m_RobotRegistration.PoseCount() == 10)
	{
		m_Robot->GoToInitialPos();
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		m_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);
		std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;
		std::cout << "Fully automatic registration successed!" << std::endl;
	}
}

void LancetRobotRegistration::setDistance(int aDistance)
{
	Distance = aDistance;
}

void LancetRobotRegistration::setAngle(int aAngle)
{
	Angle = aAngle;
}

int LancetRobotRegistration::replaceRegistration()
{
	m_RobotRegistration.RemoveAllPose();
	emit countPose(m_RobotRegistration.PoseCount());
	int num=0;
	countPose(num);
	return num;
}

const double* LancetRobotRegistration::getBaseToBaseRF() const
{
	return T_BaseToBaseRF;
}

const double* LancetRobotRegistration::getFlangeToEndRF() const
{
	return T_FlangeToEndRF;
}


