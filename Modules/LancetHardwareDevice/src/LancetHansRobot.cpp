#include "LancetHansRobot.h"

LancetHansRobot::LancetHansRobot()
{
	m_InitialPos = vtkSmartPointer<vtkMatrix4x4>::New();
	m_FlangeToTCP = vtkSmartPointer<vtkMatrix4x4>::New();
}

void LancetHansRobot::Connect()
{
	this->SetRobotIpAddress("192.168.0.10");
	unsigned int boxID = 0;
	unsigned short nPort = 10003;
	int nRet = HRIF_Connect(boxID, m_IpAddress, nPort);
}

void LancetHansRobot::Disconnect()
{
	HRIF_DisConnect(0);
}

void LancetHansRobot::PowerOn()
{
	int nGrpEnable = HRIF_GrpEnable(0, 0);
}

void LancetHansRobot::PowerOff()
{
	int nGrpDisable = HRIF_GrpDisable(0, 0);
}

void LancetHansRobot::Translate(double x, double y, double z)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity();
	matrix->SetElement(0, 3, x);
	matrix->SetElement(1, 3, y);
	matrix->SetElement(2, 3, z);
	PrintDataHelper::CoutMatrix("Translate", matrix);
	RobotTransformInTCP(matrix->GetData());
}

void LancetHansRobot::Translate(double* aDirection, double aLength)
{
	this->Translate(aDirection[0] * aLength, aDirection[1] * aLength, aDirection[2] * aLength);
}



void LancetHansRobot::Rotate(double* aDirection, double aAngle)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(matrix);
	transform->RotateWXYZ(aAngle, aDirection);
	vtkSmartPointer<vtkMatrix4x4> retMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(retMatrix);
	RobotTransformInTCP(retMatrix->GetData());
}

void LancetHansRobot::RecordInitialPos()
{
	m_InitialPos->DeepCopy(GetBaseToTCP());
}

void LancetHansRobot::GoToInitialPos()
{
	auto rotation = this->GetEulerByMatrix(m_InitialPos);
	auto translation = this->GetTranslationPartByMatrix(m_InitialPos);

	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	auto joints = this->CalculateInverse(translation, rotation);

	auto tcpTranslation = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());

	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 执行路点运动
	int nRet = HRIF_WayPointEx(0, 0, nMoveType, translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2],
		joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], tcpTranslation[0], tcpTranslation[1], tcpTranslation[2], tcpEuler[0], tcpEuler[1], tcpEuler[2],
		dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
		nIOState, strCmdID);
}

void LancetHansRobot::SetTCPToFlange()
{
	double dTcp_X = 0;
	double dTcp_Y = 0;
	double dTcp_Z = 0;
	double dTcp_Rx = 0;
	double dTcp_Ry = 0;
	double dTcp_Rz = 0;
	int nRet = HRIF_SetTCP(0, 0, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
}

bool LancetHansRobot::SetTCP(vtkMatrix4x4* aMatrix)
{
	auto euler = this->GetEulerByMatrix(aMatrix);
	auto trans = this->GetTranslationPartByMatrix(aMatrix);

	m_FlangeToTCP->DeepCopy(aMatrix);
	//Config TCP(MATRIX , TCPName)
	int nRet = HRIF_SetTCP(0, 0, trans[0], trans[1], trans[2], euler[0], euler[1], euler[2]);
	//set tcp to robot

	if (nRet == 0) {
		std::cout << "line  TCP Set succeed" << std::endl;
		return true;
	}
	else {
		std::cout << "line TCP Set failed" << std::endl;
		return false;
	}
}

std::vector<double> LancetHansRobot::GetJointAngles()
{
	// 定义关节实际位置
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	// 读取关节实际位置
	int nRet = HRIF_ReadActJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	std::vector<double> ret = { dJ1, dJ2, dJ3, dJ4, dJ5, dJ6 };
	
	return ret;
}

void LancetHansRobot::SetJointAngles(std::vector<double> aJointAngles)
{
	if (aJointAngles.size() != 6)
		return;
	
	auto targetPosition = this->CalculateForward(aJointAngles);

	auto tcpTrans = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());
	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 执行路点运动
	int nRet = HRIF_WayPointEx(0, 0, nMoveType, targetPosition[0], targetPosition[1], targetPosition[2], targetPosition[3], targetPosition[4], targetPosition[5],
		aJointAngles[0], aJointAngles[1], aJointAngles[2], aJointAngles[3], aJointAngles[4], aJointAngles[5],
		tcpTrans[0], tcpTrans[1], tcpTrans[2], tcpEuler[0], tcpEuler[1], tcpEuler[2],
		dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
		nIOState, strCmdID);
}

vtkSmartPointer<vtkMatrix4x4> LancetHansRobot::GetBaseToTCP()
{
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	// 基座坐标转换为用户坐标
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
	std::cout << " dX:" << dX << " dy:" << dY << " dZ:" << dZ << " dRx:" << dRx << " dRy:" << dRy << " dRz:" << dRz << std::endl;
	//auto angles = CalculateForward(GetJointAngles());

	Eigen::Matrix3d rotation = GetRotationMatrixByEuler(dRx, dRy, dRz);
	Eigen::Vector3d translation(dX, dY, dZ);
	auto ret = GetMatrixByRotationAndTranslation(rotation, translation);
	PrintDataHelper::CoutMatrix("GetBaseToTCP", ret);
	return ret;
}

vtkSmartPointer<vtkMatrix4x4> LancetHansRobot::GetFlangeToTCP()
{
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();
	ret->DeepCopy(m_FlangeToTCP);
	return ret;
}

vtkSmartPointer<vtkMatrix4x4> LancetHansRobot::GetBaseToFlange()
{
	auto TBase2TCP = this->GetBaseToTCP();
	PrintDataHelper::CoutMatrix("TBase2TCP_GetBaseToFlange()", TBase2TCP);
	auto TTCP2Flange = this->GetFlangeToTCP();
	PrintDataHelper::CoutMatrix("TTCP2Flange_GetBaseToFlange()", TTCP2Flange);
	TTCP2Flange->Invert();
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix4x4::Multiply4x4(TBase2TCP, TTCP2Flange, ret);
	return ret;
}

void LancetHansRobot::RobotTransformInBase(double* aMatrix)
{
	vtkSmartPointer<vtkMatrix4x4> baseToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	baseToTarget->DeepCopy(aMatrix);
	auto euler = this->GetEulerByMatrix(baseToTarget);
	auto translation = this->GetTranslationPartByMatrix(baseToTarget);
	
	

	PrintDataHelper::CoutMatrix("RobotTransformInBase",baseToTarget);
	PrintDataHelper::CoutArray(euler, "euler");
	PrintDataHelper::CoutArray(translation, "translation");
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	auto joints = this->CalculateInverse(translation, euler);
	PrintDataHelper::CoutVector(joints ,"joints");
	auto tcpTranslation = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());
	PrintDataHelper::CoutArray(tcpTranslation, "tcpTranslation");
	PrintDataHelper::CoutArray(tcpEuler, "tcpEuler");
	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 执行路点运动
	int nRet = HRIF_WayPointEx(0, 0, nMoveType, translation[0], translation[1], translation[2], euler[0], euler[1], euler[2],
		joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], tcpTranslation[0], tcpTranslation[1], tcpTranslation[2], tcpEuler[0], tcpEuler[1], tcpEuler[2],
		dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
		nIOState, strCmdID);
	cout <<"Robot Arm error code:"<< nRet << endl;
}

void LancetHansRobot::RobotTransformInTCP(double* aMatrix)
{
	auto baseToTCP = this->GetBaseToTCP();

	vtkSmartPointer<vtkMatrix4x4> tcpToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	tcpToTarget->DeepCopy(aMatrix);

	vtkSmartPointer<vtkMatrix4x4> baseToTarget = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix4x4::Multiply4x4(baseToTCP, tcpToTarget, baseToTarget);

	RobotTransformInBase(baseToTarget->GetData());
}

std::vector<double> LancetHansRobot::GetCartStiffParams()
{
	std::cout << "Hans Robot did not support get Stiff Params" << std::endl;
	return std::vector<double>();
}

bool LancetHansRobot::SetCartStiffParams(std::vector<double> aStiff)
{
	if (aStiff.size() != 6)
		return false;
	// 设置刚度参数
	int nRet = HRIF_SetStiffParams(0, 0, aStiff[0], aStiff[1], aStiff[2], aStiff[3], aStiff[4], aStiff[5]);
	return nRet == 0 ? true : false;
}

std::vector<double> LancetHansRobot::GetCartDampParams()
{
	std::cout << "Hans Robot did not support get Damp Params" << std::endl;
	return std::vector<double>();
}

bool LancetHansRobot::SetCartDampParams(std::vector<double> aDamp)
{
	if (aDamp.size() != 6)
		return false;
	int nRet = HRIF_SetDampParams(0, 0, aDamp[0], aDamp[1], aDamp[2], aDamp[3], aDamp[4], aDamp[5]);
	return nRet == 0 ? true : false;
}

std::vector<std::vector<double>> LancetHansRobot::GetJointAngleLimits()
{
	std::cout << "Hans Robot did not support get JointAngleLimits" << std::endl;
	return std::vector<std::vector<double>>();
}

bool LancetHansRobot::SetVelocity(double aVelocity)
{
	dVelocity = aVelocity;
	return true;
}

void LancetHansRobot::WaitMove()
{
	bool status;
	QThread::msleep(20);
	while (true)
	{
		int nRet = HRIF_IsMotionDone(0, 0, status);
		if (status != 0)
		{
			break;
		}
		else
		{
			QThread::msleep(1);
			QApplication::processEvents();
		}
	}
	HRIF_GrpStop(0, 0);
}



Eigen::Matrix3d LancetHansRobot::GetRotationMatrixByEuler(double rx, double ry, double rz)
{
	Eigen::Matrix3d rotationMatrix;

	Eigen::AngleAxisd rotationZ(rz / 57.29578049044297, Eigen::Vector3d::UnitZ()); // yaw
	Eigen::AngleAxisd rotationY(ry / 57.29578049044297, Eigen::Vector3d::UnitY()); // pitch
	Eigen::AngleAxisd rotationX(rx / 57.29578049044297, Eigen::Vector3d::UnitX());
	rotationMatrix = rotationZ * rotationY * rotationX;
	return rotationMatrix;


}

vtkSmartPointer<vtkMatrix4x4> LancetHansRobot::GetMatrixByRotationAndTranslation(Eigen::Matrix3d aRotation, Eigen::Vector3d aTranslation)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity();
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			matrix->SetElement(row, col, aRotation(row, col));
		}
	}	matrix->SetElement(0, 3, aTranslation[0]);
	matrix->SetElement(1, 3, aTranslation[1]);
	matrix->SetElement(2, 3, aTranslation[2]);
	return matrix;
}

Eigen::Vector3d LancetHansRobot::GetEulerByMatrix(vtkMatrix4x4* m)
{
	return this->CalculateZYXEulerByRotation(this->GetRotationPartByMatrix(m));
}

Eigen::Matrix3d LancetHansRobot::GetRotationPartByMatrix(vtkMatrix4x4* m)
{
	auto data = m->GetData();
	Eigen::Matrix3d res;
	res << data[0], data[1], data[2],
		data[4], data[5], data[6],
		data[8], data[9], data[10];
	return res;
}

Eigen::Vector3d LancetHansRobot::GetTranslationPartByMatrix(vtkMatrix4x4* m)
{
	double x = m->GetElement(0, 3);
	double y = m->GetElement(1, 3);
	double z = m->GetElement(2, 3);
	return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d LancetHansRobot::CalculateZYXEulerByRotation(Eigen::Matrix3d m)
{


	Eigen::Vector3d eulerAngle = m.eulerAngles(2, 1, 0); //zyx

	Eigen::Vector3d ret;

	// 将弧度转换为角度并赋值
	ret[0] = 180 * eulerAngle[2] / vtkMath::Pi(); // Z (yaw)
	ret[1] = 180 * eulerAngle[1] / vtkMath::Pi(); // Y (pitch)
	ret[2] = 180 * eulerAngle[0] / vtkMath::Pi(); // X (roll)
	PrintDataHelper::CoutArray(ret, "ret");

	return ret;
}

std::vector<double> LancetHansRobot::CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle)
{
	auto flange2TCP = this->GetFlangeToTCP();
	auto tcpTranslation = this->GetTranslationPartByMatrix(flange2TCP);
	auto flange2TCPEuler = this->GetEulerByMatrix(flange2TCP);
	// 定义工具坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 定义参考关节位置变量
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
		double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	// 定义转换结果
	double dTargetJ1 = 0; double dTargetJ2 = 0; double dTargetJ3 = 90;
	double dTargetJ4 = 0; double dTargetJ5 = 90; double dTargetJ6 = 0;

	auto joints = this->GetJointAngles();
	// 求逆解
	int nRet = HRIF_GetInverseKin(0, 0, aTranslation[0], aTranslation[1], aTranslation[2], aEulerAngle[0], aEulerAngle[1], aEulerAngle[2],
		tcpTranslation[0], tcpTranslation[1], tcpTranslation[2], flange2TCPEuler[0], flange2TCPEuler[1], flange2TCPEuler[2],
		dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz,
		joints[0], joints[1], joints[2], joints[3], joints[4], joints[5],
		dTargetJ1, dTargetJ2, dTargetJ3, dTargetJ4, dTargetJ5, dTargetJ6);
	std::vector<double> ret{ dTargetJ1, dTargetJ2, dTargetJ3, dTargetJ4, dTargetJ5, dTargetJ6 };

	return ret;
}
//wating for test, there is a bugfuction. 2024.10.8
std::vector<double> LancetHansRobot::CalculateForward(std::vector<double> aJointAngles)
{
	/*auto tcpTranslation = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	std::cout << "CalculateForward_tcpTranslation:" << tcpTranslation[0] << " " << tcpTranslation[1] << " " << tcpTranslation[2] << std::endl;
	auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());*/

	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	// 定义转换后的空间位置结果
	double dTarget_X = 0; double dTarget_Y = 0; double dTarget_Z = 0;
	double dTarget_Rx = 0; double dTarget_Ry = 0; double dTarget_Rz = 0;
	// 求正解
	/*int nRet = HRIF_GetForwardKin(0, 0, aJointAngles[0], aJointAngles[1], aJointAngles[2], aJointAngles[3], aJointAngles[4], aJointAngles[5],
		tcpTranslation[0], tcpTranslation[1], tcpTranslation[2], tcpEuler[0], tcpEuler[1], tcpEuler[2], dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz,
		dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);*/

		int nRet = HRIF_GetForwardKin(0, 0, aJointAngles[0], aJointAngles[1], aJointAngles[2], aJointAngles[3], aJointAngles[4], aJointAngles[5],
			0,0, 0, 0, 0, 0, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz,
			dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);
	std::vector<double> ret = { dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz };
	//std::cout << nRet << "CalculateForward" << std::endl;
	PrintDataHelper::CoutVector(ret,"ret");
	return ret;
}
string LancetHansRobot::GetErrorCodeString(int errorCode)
{
	int nRet = 0;
	string tmpstr;
	nRet =HRIF_GetErrorCodeStr(0,errorCode,tmpstr);
	return tmpstr;
}