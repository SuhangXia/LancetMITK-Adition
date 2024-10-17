#include "LancetJakaRobot.h"


LancetJakaRobot::LancetJakaRobot()
{
	m_InitialPos = vtkSmartPointer<vtkMatrix4x4>::New();
	m_FlangeToTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	SetRobotIpAddress("10.5.5.100");
	JAKAZuRobot m_Robot= JAKAZuRobot();
	std::cout << "JAKA constrction function" << std::endl;
}

void LancetJakaRobot::Connect()
{
	m_Robot.login_in(m_IpAddress);
	std::cout << "JAKA Connect()" << std::endl;

}

void LancetJakaRobot::Disconnect()
{
	//TODO
	std::cout << "TODO Disconnect" << std::endl;
}

void LancetJakaRobot::PowerOn()
{
	CHECK_ERROR_AND_RETURN(m_Robot.enable_robot());
}

void LancetJakaRobot::PowerOff()
{
	CHECK_ERROR_AND_RETURN(m_Robot.disable_robot());
}

void LancetJakaRobot::Translate(double x, double y, double z)
{
	/*vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(GetBaseToTCP());
	PrintDataHelper::CoutMatrix("baseToTcp", matrix);
	auto euler = this->GetEulerByMatrix(matrix);
	PrintDataHelper::CoutArray(euler, "original_tcpEuler");
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(matrix);
	matrix->SetElement(0, 3, matrix->GetElement(0, 3) + x);
	matrix->SetElement(1, 3, matrix->GetElement(1, 3) + y);
	matrix->SetElement(2, 3, matrix->GetElement(2, 3) + z);
	RobotTransformInBase(matrix->GetData());
	PrintDataHelper::CoutMatrix("baseToTcp2", matrix);*/
	std::cout << "jaka Translate() Begin" << std::endl;
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity();
	matrix->SetElement(0, 3, x);
	matrix->SetElement(1, 3, y);
	matrix->SetElement(2, 3, z);
	PrintDataHelper::CoutMatrix("jaka Translate", matrix);
	RobotTransformInTCP(matrix->GetData());
}

void LancetJakaRobot::Translate(double* aDirection, double aLength)
{
	this->Translate(aDirection[0] * aLength, aDirection[1] * aLength, aDirection[2] * aLength);
}

void LancetJakaRobot::Rotate(double* aDirection, double aAngle)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(GetBaseToTCP());

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(matrix);

	transform->RotateWXYZ(aAngle, aDirection);

	vtkSmartPointer<vtkMatrix4x4> retMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(retMatrix);

	RobotTransformInBase(retMatrix->GetData());
}

void LancetJakaRobot::RecordInitialPos()
{
	m_InitialPos->DeepCopy(GetBaseToTCP());
}

void LancetJakaRobot::GoToInitialPos()
{
	auto rotation = this->GetEulerByMatrix(m_InitialPos);
	auto translation = this->GetTranslationPartByMatrix(m_InitialPos);
	std::vector<double> joints = this->CalculateInverse(translation, rotation);
	JointValue* joint_pos = new JointValue();
	for (int i = 0; i < 6; i++)
	{
		joint_pos->jVal[i]= joints[i];
	}
	MoveMode move_mode=ABS;
	BOOL is_block = FALSE;
	auto tcpTranslation = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());

	m_Robot.joint_move(joint_pos, move_mode, is_block, dJointVel, dJointAcc, dJointErr, nullptr);
	delete joint_pos;
}

void LancetJakaRobot::SetTCPToFlange()
{
	int id = 0;
	CartesianPose* toolTcp = new CartesianPose();
	const char* toolName="Flange";
	toolTcp->rpy.rx= 0;
	toolTcp->rpy.ry= 0;
	toolTcp->rpy.rz= 0;
	toolTcp->tran.x= 0;
	toolTcp->tran.y= 0;
	toolTcp->tran.z= 0;
	m_Robot.set_tool_data(id, toolTcp, toolName);
	delete toolTcp;
}

bool LancetJakaRobot::SetTCP(vtkMatrix4x4* aMatrix)
{
	auto euler = this->GetEulerByMatrix(aMatrix);
	auto trans = this->GetTranslationPartByMatrix(aMatrix);

	m_FlangeToTCP->DeepCopy(aMatrix);
	//Config TCP(MATRIX , TCPName)
	int id = 0;
	CartesianPose* toolTcp = new CartesianPose();
	const char* toolName = "Flange";
	
	toolTcp->tran.x = trans[0];
	toolTcp->tran.y = trans[1];
	toolTcp->tran.z = trans[2];
	toolTcp->rpy.rx = euler[0];
	toolTcp->rpy.ry = euler[1];
	toolTcp->rpy.rz = euler[2];
	errno_t ret = m_Robot.set_tool_data(id, toolTcp, toolName);
	if (ret != 0)
	{
		std::cout << "set tool data failed" << std::endl;
		return false;
	}
	delete toolTcp;
	return true;
}

std::vector<double> LancetJakaRobot::GetJointAngles()
{
	return std::vector<double>();
}

void LancetJakaRobot::SetJointAngles(std::vector<double> aJointAngles)
{
}

vtkSmartPointer<vtkMatrix4x4> LancetJakaRobot::GetBaseToTCP()
{
	CartesianPose* tcp_position = new CartesianPose();
	// 基座坐标转换为用户坐标
	m_Robot.get_tcp_position(tcp_position);
	
	std::cout << " dX:" << tcp_position->tran.x << " dy:" << tcp_position->tran.y << " dZ:" << tcp_position->tran.z << " dRx:" << tcp_position->rpy.rx << " dRy:" << tcp_position->rpy.ry << " dRz:" << tcp_position->rpy.rz << std::endl;
	//auto angles = CalculateForward(GetJointAngles());

	Eigen::Matrix3d rotation = GetRotationMatrixByEuler(tcp_position->rpy.rx, tcp_position->rpy.ry, tcp_position->rpy.rz);
	Eigen::Vector3d translation(tcp_position->tran.x, tcp_position->tran.y, tcp_position->tran.z);
	auto ret = GetMatrixByRotationAndTranslation(rotation, translation);
	PrintDataHelper::CoutMatrix("GetBaseToTCP", ret);
	delete tcp_position;
	return ret;
}

vtkSmartPointer<vtkMatrix4x4> LancetJakaRobot::GetFlangeToTCP()
{
	return vtkSmartPointer<vtkMatrix4x4>();
}

vtkSmartPointer<vtkMatrix4x4> LancetJakaRobot::GetBaseToFlange()
{
	return vtkSmartPointer<vtkMatrix4x4>();
}

void LancetJakaRobot::RobotTransformInBase(double* aMatrix)
{
	std::cout << "jaka RobotTransformInBase() Begin" << std::endl;
	vtkSmartPointer<vtkMatrix4x4> baseToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	baseToTarget->DeepCopy(aMatrix);

	auto euler = this->GetEulerByMatrix(baseToTarget);
	auto translation = this->GetTranslationPartByMatrix(baseToTarget);

	PrintDataHelper::CoutMatrix("RobotTransformInBase", baseToTarget);
	PrintDataHelper::CoutArray(euler, "euler");
	PrintDataHelper::CoutArray(translation, "translation");

	std::cout << "jaka before CalculateInverse" << std::endl;
	// 计算关节角度
	auto joints = this->CalculateInverse(translation, euler);
	PrintDataHelper::CoutVector(joints, "jaka after culculate inverse joints");

	//auto tcpTranslation = this->GetTranslationPartByMatrix(this->GetFlangeToTCP());
	//auto tcpEuler = this->GetEulerByMatrix(this->GetFlangeToTCP());

	//PrintDataHelper::CoutArray(tcpTranslation, "tcpTranslation");
	//PrintDataHelper::CoutArray(tcpEuler, "tcpEuler");

	// 设置运动模式和其他参数
	MoveMode move_mode = ABS; // 运动模式（假设使用绝对运动）
	BOOL is_block = FALSE; // 是否阻塞

	// 执行关节运动
	JointValue* joint_pos = new JointValue(); // 创建关节位置对象
	for (int i = 0; i < 6; i++)
	{
		joint_pos->jVal[i] = joints[i]; // 将计算得到的关节角度赋值给 joint_pos
	}
	std::cout << "jaka before m_Robot.joint_move" << std::endl;
	// 调用 joint_move 函数
	m_Robot.joint_move(joint_pos, move_mode, is_block, dJointVel, dJointAcc, dJointErr, nullptr);
	std::cout << "jaka after m_Robot.joint_move" << std::endl;
	// 清理内存
	delete joint_pos;

}


void LancetJakaRobot::RobotTransformInTCP(double* aMatrix)
{
	/*auto baseToTCP = this->GetBaseToTCP();

	vtkSmartPointer<vtkMatrix4x4> tcpToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	tcpToTarget->DeepCopy(aMatrix);

	vtkSmartPointer<vtkMatrix4x4> baseToTarget = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix4x4::Multiply4x4(baseToTCP, tcpToTarget, baseToTarget);

	RobotTransformInBase(baseToTarget->GetData());*/
	std: cout<< "jaka RobotTransformInTCP()" << std::endl;
	double pose[6] = {};
	vtkSmartPointer<vtkMatrix4x4> TBase2Tcp = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2Tcp->DeepCopy(this->GetBaseToTCP());
	PrintDataHelper::CoutMatrix("Current Base2TCP Matrix:", TBase2Tcp);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->PreMultiply();
	transform->SetMatrix(TBase2Tcp);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix->DeepCopy(aMatrix);
	transform->Concatenate(vtkMatrix);

	vtkSmartPointer<vtkMatrix4x4> newBase2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(newBase2TCP);
	this->RobotTransformInBase(newBase2TCP->GetData());
}

std::vector<double> LancetJakaRobot::GetCartStiffParams()
{
	std::cout << "Hans Robot did not support get Stiff Params" << std::endl;
	return std::vector<double>();
}

bool LancetJakaRobot::SetCartStiffParams(std::vector<double> aStiff)
{
	return false;
}

std::vector<double> LancetJakaRobot::GetCartDampParams()
{
	return std::vector<double>();
}

bool LancetJakaRobot::SetCartDampParams(std::vector<double> aDamp)
{
	return false;
}

std::vector<std::vector<double>> LancetJakaRobot::GetJointAngleLimits()
{
	return std::vector<std::vector<double>>();
}

bool LancetJakaRobot::SetVelocity(double aVelocity)
{
	return false;
}

void LancetJakaRobot::WaitMove()
{
	QThread::msleep(20);
	while (true)
	{
		BOOL* status{};
		m_Robot.is_in_pos(status);
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
	m_Robot.motion_abort();
}

Eigen::Matrix3d LancetJakaRobot::GetRotationMatrixByEuler(double rx, double ry, double rz)
{
	Eigen::Matrix3d rotationMatrix;

	Eigen::AngleAxisd rotationZ(rz / 57.29578049044297, Eigen::Vector3d::UnitZ()); // yaw
	Eigen::AngleAxisd rotationY(ry / 57.29578049044297, Eigen::Vector3d::UnitY()); // pitch
	Eigen::AngleAxisd rotationX(rx / 57.29578049044297, Eigen::Vector3d::UnitX());
	rotationMatrix = rotationZ * rotationY * rotationX;
	return rotationMatrix;
}

vtkSmartPointer<vtkMatrix4x4> LancetJakaRobot::GetMatrixByRotationAndTranslation(Eigen::Matrix3d aRotation, Eigen::Vector3d aTranslation)
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

Eigen::Vector3d LancetJakaRobot::GetEulerByMatrix(vtkMatrix4x4* m)
{
	return this->CalculateXYZEulerByRotation(this->GetRotationPartByMatrix(m));
}

Eigen::Vector3d LancetJakaRobot::CalculateXYZEulerByRotation(Eigen::Matrix3d m)
{
	// Calculate the Euler angles in XYZ order
	Eigen::Vector3d eulerAngle = m.eulerAngles(0, 1, 2); // xyz

	Eigen::Vector3d ret;

	// Convert radians to degrees and assign values
	ret[0] = 180 * eulerAngle[0] / vtkMath::Pi(); // X (roll)
	ret[1] = 180 * eulerAngle[1] / vtkMath::Pi(); // Y (pitch)
	ret[2] = 180 * eulerAngle[2] / vtkMath::Pi(); // Z (yaw)

	PrintDataHelper::CoutArray(ret, "ret");

	return ret;
}

Eigen::Matrix3d LancetJakaRobot::GetRotationPartByMatrix(vtkMatrix4x4* m)
{
	auto data = m->GetData();
	Eigen::Matrix3d res;
	res << data[0], data[1], data[2],
		data[4], data[5], data[6],
		data[8], data[9], data[10];
	return res;
}

Eigen::Vector3d LancetJakaRobot::GetTranslationPartByMatrix(vtkMatrix4x4* m)
{
	double x = m->GetElement(0, 3);
	double y = m->GetElement(1, 3);
	double z = m->GetElement(2, 3);
	return Eigen::Vector3d(x, y, z);
}

std::vector<double> LancetJakaRobot::CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle)
{
	std::cout << "CalculateInverse()Begin" << std::endl;
	/*auto flange2TCP = this->GetFlangeToTCP();
	auto tcpTranslation = this->GetTranslationPartByMatrix(flange2TCP);
	auto flange2TCPEuler = this->GetEulerByMatrix(flange2TCP);
	auto joints = this->GetJointAngles();*/
	// 求逆解
	JointValue* ref_pos=new JointValue();
	m_Robot.get_joint_position(ref_pos);//读取当前关节位置，作为参考位置
	CartesianPose* tcp_pos=new CartesianPose();;
	tcp_pos->tran.x = aTranslation[0];
	tcp_pos->tran.y = aTranslation[1];
	tcp_pos->tran.z = aTranslation[2];
	tcp_pos->rpy.rx = aEulerAngle[0];
	tcp_pos->rpy.ry = aEulerAngle[1];
	tcp_pos->rpy.rz = aEulerAngle[2];
	JointValue* joint_pos=new JointValue();
	std::cout << "before m_Robot.kine_inverse(ref_pos, tcp_pos, joint_pos);" << std::endl;
	m_Robot.kine_inverse(ref_pos, tcp_pos, joint_pos);
	std::cout << "after m_Robot.kine_inverse(ref_pos, tcp_pos, joint_pos);" << std::endl;
	std::vector<double> ret{ joint_pos->jVal[0], joint_pos->jVal[1], joint_pos->jVal[2], joint_pos->jVal[3], joint_pos->jVal[4], joint_pos->jVal[5] };
	delete ref_pos;
	delete joint_pos;
	delete tcp_pos;
	return ret;
}

