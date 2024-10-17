#include "AriemediCamera.h"

AriemediCamera::AriemediCamera()
{
	m_Tracker = ARMDCombinedAPI();
}

void AriemediCamera::Connect()
{
	std::string hostname;
	std::string IP;
	DeviceScan* deviceScan = new DeviceScan();
	while (deviceScan->getDeviceInfo().size() == 0)
	{
		deviceScan->updateDeviceInfo();
	}

	for (auto& x : deviceScan->getDeviceInfo())
	{
		hostname = x.first;
		IP = x.second;

		std::cout << "The firt scaned RT device hostname is " << hostname << ", and IP is " << IP << std::endl;
		break;
	}
	int errorCode = m_Tracker.connect(hostname);
	if (errorCode == 0)
	{
		std::cout << "Successed!" << std::endl;
		std::cout << "Local IP address is " << m_Tracker.getConnectionIPs()[0] << std::endl;
		std::cout << "RT device IP address is " << m_Tracker.getConnectionIPs()[1] << std::endl;
		//updateFlage = true;
		return;
	}
	else
	{
		std::cout << ConnectionStatus::toString(m_Tracker.getConnectionStatus()) << endl;
		std::cout << "Failed!" << endl;
		return;
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	//filename.append("/");

	m_Tracker.loadPassiveToolAROM(filename.toStdString());
}

void AriemediCamera::Disconnect()
{
	m_Tracker.disconnect();
}

void AriemediCamera::Start()
{
	m_Tracker.startTracking();
	m_Tracker.startImaging();
	if (m_ToolMatrixMap.size() <= 0)
		return;
	if (!m_CameraUpdateTimer)
	{
		m_CameraUpdateTimer = new QTimer();
	}

	connect(m_CameraUpdateTimer, &QTimer::timeout, this, &AriemediCamera::UpdateData);

	m_CameraUpdateTimer->start(100);
}

void AriemediCamera::Stop()
{
	if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
	{
		m_Tracker.disconnect();
	}
	else
	{
		m_Tracker.stopTracking();
		m_Tracker.stopImaging();
		m_Tracker.disconnect();
	}
	//m_Tracker.stopImaging();
	if (m_CameraUpdateTimer && m_CameraUpdateTimer->isActive())
	{
		// 停止定时器
		m_CameraUpdateTimer->stop();

		// 断开定时器的信号连接
		disconnect(m_CameraUpdateTimer, &QTimer::timeout, this, &AriemediCamera::UpdateData);
	}
}

Eigen::Vector3d AriemediCamera::GetToolTipByName(std::string aToolName)
{
	return m_ToolTipMap[aToolName];
}

vtkSmartPointer<vtkMatrix4x4> AriemediCamera::GetToolMatrixByName(std::string aToolName)
{
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();
	ret->DeepCopy(m_ToolMatrixMap[aToolName]);
	return ret;
}

void AriemediCamera::InitToolsName(std::vector<std::string> aToolsName)
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

void AriemediCamera::UpdateImageData()
{
	if (DeviceAlert::Normal != m_Tracker.getSystemAlert())
		std::cout << DeviceAlert::toString(m_Tracker.getSystemAlert()) << std::endl;

	//interuption check
	if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
	{
		std::cout << ConnectionStatus::toString(m_Tracker.getConnectionStatus()) << std::endl;
		return;
	}

	std::vector<int> imageSize = m_Tracker.getImageSize();

	m_LeftImage.reset(new char[imageSize.at(0) * imageSize.at(1)]);
	m_RightImage.reset(new char[imageSize.at(0) * imageSize.at(1)]);

	memcpy(m_LeftImage.get(), m_Tracker.getLeftImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));
	memcpy(m_RightImage.get(), m_Tracker.getRightImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));
}

void AriemediCamera::DisplayArea(short width, short height, short leftX, short leftY, short rightX, short rightY)
{
	m_Tracker.setAreaDisplay(width, height, leftX, leftY, rightX, rightY);
}


std::pair<const char*, const char*> AriemediCamera::GetImageData()
{
	return std::pair(m_LeftImage.get(), m_RightImage.get());
}

void AriemediCamera::HideArea()
{
	m_Tracker.setAreaHidden();
}

std::pair<double, double> AriemediCamera::GetImageSize()
{
	if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
	{
		//std::cout << ConnectionStatus::toString(m_Tracker.getConnectionStatus()) << std::endl;
		return std::pair(0,0);
	}

	std::vector<int> imageSize = m_Tracker.getImageSize();
	return std::pair(imageSize.at(0), imageSize.at(1));
}

void AriemediCamera::ConvertRomToARom(std::string out, std::string in)
{
	m_Tracker.convert2AROM(out, in);
}

bool AriemediCamera::UpdateCameraToToolMatrix(ToolTrackingData aToolTrackingData)
{
	std::string trackingToolName = aToolTrackingData.name;

	if (m_ToolMatrixMap.count(trackingToolName) == 0)
		return false;

	auto matrix = m_ToolMatrixMap[trackingToolName];
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			matrix->SetElement(i, j, aToolTrackingData.transform.matrix[i][j]);
		}
	}

	Eigen::Vector3d pt(aToolTrackingData.transform.tx, aToolTrackingData.transform.ty, aToolTrackingData.transform.tz);

	m_ToolTipMap[trackingToolName] = pt;
	return true;
}

void AriemediCamera::UpdateData()
{
	//show system alert information
	if (DeviceAlert::Normal != m_Tracker.getSystemAlert())
		cout << DeviceAlert::toString(m_Tracker.getSystemAlert()) << endl;

	//interuption check
	if (ConnectionStatus::Interruption == m_Tracker.getConnectionStatus())
	{
		cout << ConnectionStatus::toString(m_Tracker.getConnectionStatus()) << endl;
		return;
	}
	m_Tracker.trackingUpdate();
	std::vector<MarkerPosition> allMarkers = m_Tracker.getAllMarkers();
	//cout << "All markers number is " << allMarkers.size() << endl;

	std::vector<ToolTrackingData> toolData = m_Tracker.getTrackingData(allMarkers);
	for (int i = 0; i < toolData.size(); i++)
	{
		UpdateCameraToToolMatrix(toolData[i]);
	}
	UpdateImageData();

	emit CameraUpdateClock();

}