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

void LancetRobotRegistration::replaceRegistration()
{
	m_RobotRegistration.RemoveAllPose();
	//m_Controls.lineEdit_collectedRoboPose->setText(QString::number(0));
}

void LancetRobotRegistration::saveArmMatrix()
{
	// Open a folder selection dialog
	QString folderPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder to Save", QString(getenv("USERPROFILE")) + "\\Desktop\\save");

	// Check if the user canceled the selection
	if (folderPath.isEmpty()) {
		std::cout << "No folder selected for saving." << std::endl; // Output message to console
		return; // Cancel operation
	}

	// Construct file paths
	std::string baseToBaseRFPath = (folderPath + "/T_BaseToBaseRF.txt").toStdString();
	std::string flangeToEndRFPath = (folderPath + "/T_FlangeToEndRF.txt").toStdString();

	// Save T_BaseToBaseRF
	std::ofstream robotMatrixFile(baseToBaseRFPath);
	if (robotMatrixFile.is_open())
	{
		for (int i = 0; i < 16; i++)
		{
			robotMatrixFile << T_BaseToBaseRF[i];
			if (i != 15)
			{
				robotMatrixFile << ",";
			}
			else
			{
				robotMatrixFile << ";";
			}
		}
		robotMatrixFile << std::endl;
		robotMatrixFile.close();
		std::cout << "T_BaseToBaseRF saved successfully." << std::endl; // Confirmation message
	}
	else
	{
		std::cout << "Cannot open file: T_BaseToBaseRF.txt" << std::endl; // Output error message
	}

	// Save T_FlangeToEndRF
	std::ofstream robotMatrixFile1(flangeToEndRFPath);
	if (robotMatrixFile1.is_open())
	{
		for (int i = 0; i < 16; i++)
		{
			robotMatrixFile1 << T_FlangeToEndRF[i];
			if (i != 15)
			{
				robotMatrixFile1 << ",";
			}
			else
			{
				robotMatrixFile1 << ";";
			}
		}
		robotMatrixFile1 << std::endl;
		robotMatrixFile1.close();
		std::cout << "T_FlangeToEndRF saved successfully." << std::endl; // Confirmation message
	}
	else
	{
		std::cout << "Cannot open file: T_FlangeToEndRF.txt" << std::endl; // Output error message
	}

	std::cout << "saveArmMatrix operation completed." << std::endl; // Indicate completion
}


void LancetRobotRegistration::reuseArmMatrix()
{
	// Open a folder selection dialog
	QString folderPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder", QString(getenv("USERPROFILE")) + "\\Desktop\\save");

	// Check if the user canceled the selection
	if (folderPath.isEmpty()) {
		std::cout << "No folder selected" << std::endl; // Output message to console
		return; // Cancel operation
	}

	// Construct file paths
	std::string baseToBaseRFPath = (folderPath + "/T_BaseToBaseRF.txt").toStdString();
	std::string flangeToEndRFPath = (folderPath + "/T_FlangeToEndRF.txt").toStdString();

	// Import T_BaseToBaseRF
	std::ifstream inputFile(baseToBaseRFPath);
	if (inputFile.is_open())
	{
		std::string line;
		if (std::getline(inputFile, line))
		{
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ','))
			{
				T_BaseToBaseRF[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else
	{
		std::cout << "Cannot open file: T_BaseToBaseRF.txt" << std::endl; // Output error message
	}
	PrintDataHelper::CoutMatrix("T_BaseToBaseRF", T_BaseToBaseRF);
	// Import T_FlangeToEndRF
	std::ifstream inputFile2(flangeToEndRFPath);
	if (inputFile2.is_open())
	{
		std::string line2;
		if (std::getline(inputFile2, line2))
		{
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ','))
			{
				T_FlangeToEndRF[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else
	{
		std::cout << "Cannot open file: T_FlangeToEndRF.txt" << std::endl; // Output error message
	}

	// Print T_FlangeToEndRF
	PrintDataHelper::CoutMatrix("T_FlangeToEndRF", T_FlangeToEndRF);
}

