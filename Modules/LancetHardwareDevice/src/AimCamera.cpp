#include "AimCamera.h"

void AimCamera::Connect()
{
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(m_AimHandle);
	Aim_SetEthernetConnectIP(m_AimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(m_AimHandle, I_ETHERNET, mPosDataPara);

	if (rlt == AIMOOE_OK)
	{
		std::cout << "connect success";
	}
	else {

		std::cout << "connect failed" << std::endl;
		//return;
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	std::cout << "The selected folder address :" << filename.toStdString() << std::endl;;
	rlt = Aim_SetToolInfoFilePath(m_AimHandle, filename.toLatin1().data());

	if (rlt == AIMOOE_OK)
	{

		std::cout << "set filenemae success" << std::endl;
	}
	else {

		std::cout << "set filenemae failed" << std::endl;
		//return;
	}

	int size = 0;
	Aim_GetCountOfToolInfo(m_AimHandle, size);

	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];

		rlt = Aim_GetAllToolFilesBaseInfo(m_AimHandle, toolarr);

		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				/*		char* ptool = toolarr[i].name;
						QString toolInfo = QString("Tool Name：") + QString::fromLocal8Bit(ptool);
						m_Controls.textBrowser->append(toolInfo);*/
			}
		}
		delete[] toolarr;
	}
	else {
		std::cout << "There are no tool identification files in the current directory:";
	}

	std::cout << "End of connection";
	rlt = AIMOOE_OK;
}

void AimCamera::Disconnect()
{
	Aim_API_Close(m_AimHandle);
}

void AimCamera::Start()
{
	if (rlt != AIMOOE_OK)
	{
		std::cout << "Please add Aim files first and check device connected" << std::endl;
		return;
	}
	std::cout << "Start Camera" << std::endl;
	if (m_ToolMatrixMap.size() <= 0)
		return;
	if (!m_CameraUpdateTimer)
	{
		m_CameraUpdateTimer = new QTimer();
	}

	connect(m_CameraUpdateTimer, &QTimer::timeout, this, &AimCamera::UpdateData);

	m_CameraUpdateTimer->start(100);
}

void AimCamera::Stop()
{
	if (m_CameraUpdateTimer && m_CameraUpdateTimer->isActive())
	{
		// 停止定时器
		m_CameraUpdateTimer->stop();

		// 断开定时器的信号连接
		disconnect(m_CameraUpdateTimer, &QTimer::timeout, this, &AimCamera::UpdateData);
	}
}

Eigen::Vector3d AimCamera::GetToolTipByName(std::string aToolName)
{
	if (m_ToolTipMap.count(aToolName) > 0)
	{
		return m_ToolTipMap[aToolName];
	}
	std::cout << "No corresponding object can be found as: " << aToolName << std::endl;
	return Eigen::Vector3d(0,0,0);
}

vtkSmartPointer<vtkMatrix4x4> AimCamera::GetToolMatrixByName(std::string aToolName)
{
	if (m_ToolTipMap.count(aToolName) > 0)
	{
		return m_ToolMatrixMap[aToolName];
	}
	std::cout << "No corresponding object can be found as: " << aToolName << std::endl;
	return vtkSmartPointer<vtkMatrix4x4>();
}

void AimCamera::InitToolsName(std::vector<std::string> aToolsName)
{
	this->ResetTools();
	for (int i = 0; i < aToolsName.size(); ++i)
	{
		if (m_ToolMatrixMap.count(aToolsName[i]) > 0)
			continue;
		vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
		Eigen::Vector3d tip(0, 0, 0);
		m_ToolMatrixMap.emplace(std::pair(aToolsName[i], m));
		m_ToolTipMap.emplace(std::pair(aToolsName[i], tip));
	}
}

T_AimToolDataResult* AimCamera::GetNewToolData()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(m_AimHandle, I_ETHERNET, markerSt, statusSt);
	if (rlt == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;//新建一个值指，将指针清空用于存数据
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;

	rlt = Aim_FindToolInfo(m_AimHandle, markerSt, mtoolsrlt, 0);//获取数据
	T_AimToolDataResult* prlt = mtoolsrlt;//将获取完数据的从mtoolsrlt给prlt指针

	return prlt;
}

bool AimCamera::UpdateCameraToToolMatrix(T_AimToolDataResult* aToolData, const std::string aName)
{
	QLabel* label = nullptr;
	if (aToolData->toolname == aName)
	{
		if (m_ToolLabelMap.count(aName) > 0)
		{
			 label = m_ToolLabelMap[aName];
		}
		if (aToolData->validflag)
		{
			//获取相机数据
			Eigen::Vector3d camera2ToolTranslation;
			Eigen::Matrix3d camera2ToolRotation;
			camera2ToolTranslation[0] = aToolData->Tto[0];
			camera2ToolTranslation[1] = aToolData->Tto[1];
			camera2ToolTranslation[2] = aToolData->Tto[2];
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					camera2ToolRotation(i, j) = (double)aToolData->Rto[i][j];
				}
			}
			//拼接矩阵
			auto matrix = m_ToolMatrixMap[aName];

			matrix->DeepCopy(GetMatrixByRotationAndTranslation(camera2ToolRotation, camera2ToolTranslation));
			Eigen::Vector3d tip(aToolData->tooltip[0], aToolData->tooltip[1], aToolData->tooltip[2]);
			m_ToolTipMap[aName] = tip;

			if (label != nullptr)
			{
				QString str = "x:" + QString::number(aToolData->tooltip[0]) + "\n "
					+ "y:" + QString::number(aToolData->tooltip[1]) + " \n "
					+ "z:" + QString::number(aToolData->tooltip[2]);
				label->setText(str);
				label->setStyleSheet(this->m_GreenLabelStyleSheet);
			}
			return true;
		}
		else
		{
			if (label != nullptr)
			{
				QString str = "x:nan  y:nan  z:nan";
				label->setText(str);
				label->setStyleSheet(this->m_GreenLabelStyleSheet);
			}
			return false;
		}
	}
	return false;
}

void AimCamera::UpdateData()
{
	auto prlt = GetNewToolData();
	if (rlt == AIMOOE_OK)//判断是否采集成功
	{
		do
		{
			for (auto it = m_ToolMatrixMap.begin(); it != m_ToolMatrixMap.end(); ++it)
			{
				UpdateCameraToToolMatrix(prlt, it->first.c_str());
			}

			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
	}
	else
	{
		delete prlt;
	}
	emit CameraUpdateClock();
}
