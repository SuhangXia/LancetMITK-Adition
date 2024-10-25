#include "AriemediCamera.h"

AriemediCamera::AriemediCamera() : m_ImageUpdateTimer(new QTimer())
{
	m_Tracker = new ARMDCombinedAPI();
}

AriemediCamera::~AriemediCamera()
{
	m_IsStart = false;
	//Stop();
	Disconnect();
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
	int errorCode = m_Tracker->connect(hostname);
	if (errorCode == 0)
	{
		std::cout << "Successed!" << std::endl;
		std::cout << "Local IP address is " << m_Tracker->getConnectionIPs()[0] << std::endl;
		std::cout << "RT device IP address is " << m_Tracker->getConnectionIPs()[1] << std::endl;
		//updateFlage = true;
	}
	else
	{
		std::cout << ConnectionStatus::toString(m_Tracker->getConnectionStatus()) << endl;
		std::cout << "Failed!" << endl;
		return;
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	m_Tracker->loadPassiveToolAROM(filename.toStdString());
}

void AriemediCamera::Disconnect()
{
	m_Tracker->disconnect();
}

void AriemediCamera::Start()
{
	m_Tracker->startTracking();
	m_Tracker->startImaging();
	if (m_ToolMatrixMap.size() <= 0)
		return;

	if (!m_CameraUpdateTimer)
	{
		m_ImageUpdateTimer = new QTimer(this);
		m_CameraUpdateTimer = new QTimer(this);
	}
	//QtConcurrent::run([this] { StartUpdateThread(); });
	//connect(m_ImageUpdateTimer, &QTimer::timeout, this, &AriemediCamera::RequestUpdateImage);
	connect(m_CameraUpdateTimer, &QTimer::timeout, this, &AriemediCamera::RequestUpdateTracking);

	//m_ImageUpdateTimer->start(5);
	m_CameraUpdateTimer->start(100);
	m_IsStart = true;
	this->start();
}

void AriemediCamera::Stop()
{
	if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
	{
		m_Tracker->disconnect();
	}
	else
	{
		m_Tracker->stopTracking();
		m_Tracker->stopImaging();
		m_Tracker->disconnect();
	}
	//m_Tracker->stopImaging();
	if (m_CameraUpdateTimer && m_CameraUpdateTimer->isActive())
	{
		// 停止定时器
		m_CameraUpdateTimer->stop();
		//m_ImageUpdateTimer->stop();
		// 断开定时器的信号连接
		disconnect(m_CameraUpdateTimer, &QTimer::timeout, this, &AriemediCamera::RequestUpdateTracking);
		//disconnect(m_ImageUpdateTimer, &QTimer::timeout, this, &AriemediCamera::RequestUpdateImage);
	}
	m_IsStart = false;
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

void AriemediCamera::run()
{
	while (m_IsStart)
	{
		UpdateData();
		UpdateImageData();
		//m_Tracker->trackingUpdate();

		//		if (m_Tracker->getReconstructionStatus() == TransmissionStatus::ReconstructionReady)
		//{
		//	ReconstructPointCloud Reconstructor = ReconstructPointCloud(m_Tracker->getReconstructParameters());
		//	Reconstructor.setReconstructData(m_Tracker->getReconstructionData());
		//	m_PointCloud = Reconstructor.getPointCloud();
		//
		//	//return GetCameraPointCloudDataNode(pc);
		//}

		//std::cout << m_Tracker->getReconstructionStatus() << std::endl;

	}
}

void AriemediCamera::UpdateImageData()
{
	if (DeviceAlert::Normal != m_Tracker->getSystemAlert())
		std::cout << DeviceAlert::toString(m_Tracker->getSystemAlert()) << std::endl;

	//interuption check
	if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
	{
		std::cout << ConnectionStatus::toString(m_Tracker->getConnectionStatus()) << std::endl;
		return;
	}
	m_Tracker->trackingUpdate();
	std::vector<int> imageSize = m_Tracker->getImageSize();
	int channels = 1;
	if (!m_LeftImage || imageSize.at(0) * imageSize.at(1) != m_PreviousImageSize) {
		m_LeftImage.reset(new char[imageSize.at(0) * imageSize.at(1) * channels]);
		m_RightImage.reset(new char[imageSize.at(0) * imageSize.at(1) * channels]);
		m_PreviousImageSize = imageSize.at(0) * imageSize.at(1);
	}

	memcpy(m_LeftImage.get(), m_Tracker->getLeftImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));
	memcpy(m_RightImage.get(), m_Tracker->getRightImagingData(), imageSize.at(0) * imageSize.at(1) * sizeof(char));

	emit ImageUpdateClock(m_LeftImage.get(), m_RightImage.get(), imageSize.at(0), imageSize.at(1));
}

void AriemediCamera::DisplayArea(short width, short height, short leftX, short leftY, short rightX, short rightY)
{
	m_Tracker->setAreaDisplay(width, height, leftX, leftY, rightX, rightY);
}


std::pair<char*, char*> AriemediCamera::GetImageData()
{
	return std::pair(m_LeftImage.get(), m_RightImage.get());
}

void AriemediCamera::HideArea()
{
	m_Tracker->setAreaHidden();
}

std::pair<double, double> AriemediCamera::GetImageSize()
{
	if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
	{
		return std::pair(0, 0);
	}

	std::vector<int> imageSize = m_Tracker->getImageSize();
	return std::pair(imageSize.at(0), imageSize.at(1));
}

void AriemediCamera::ConvertRomToARom(std::string out, std::string in)
{
	m_Tracker->convert2AROM(out, in);
}

void AriemediCamera::SetAreaDisplay(short leftX, short leftY, short rightX, short rightY, short width, short height)
{
	m_Tracker->setAreaDisplay(width, height, leftX, leftY, rightX, rightY);
}

char* AriemediCamera::GetRightImage()
{
	return m_Tracker->getRightImagingData();
}

mitk::DataNode::Pointer AriemediCamera::GetPointCloud()
{
	/*m_Tracker->reconstructPointCloud(1920, 1200, 0, 0, 0, 0);


	std::cout << m_Tracker->getReconstructionStatus() << std::endl;
	if (m_Tracker->getReconstructionStatus() == TransmissionStatus::ReconstructionReady)
	{
		ReconstructPointCloud Reconstructor = ReconstructPointCloud(m_Tracker->getReconstructParameters());
		Reconstructor.setReconstructData(m_Tracker->getReconstructionData());
		std::vector<std::vector<float>> pc = Reconstructor.getPointCloud();
		return GetCameraPointCloudDataNode(pc);
	}*/
	m_Tracker->reconstructPointCloud(1920, 1200, 0, 0, 0, 0);
	if (m_PointCloud.size() == 0)
		return nullptr;
	return GetCameraPointCloudDataNode(m_PointCloud);
	//return nullptr;
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

mitk::DataNode::Pointer AriemediCamera::GetCameraPointCloudDataNode(std::vector<std::vector<float>> aPointCloud)
{
	mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
	std::cout << "GetCameraPointCloudDataNode Size: " << aPointCloud.size() << std::endl;
	for (int i = 0; i < aPointCloud.size(); i++)
	{
		mitk::PointSet::PointType pt;
		pt[0] = aPointCloud[i][0];
		pt[1] = aPointCloud[i][1];
		pt[2] = aPointCloud[i][2];
		pointSet->SetPoint(i, pt);
	}

	mitk::DataNode::Pointer pointSetNode = mitk::DataNode::New();
	pointSetNode->SetData(pointSet);
	pointSetNode->SetName("PointCloud");
	pointSetNode->SetFloatProperty("pointsize", 1);
	pointSetNode->SetColor(1, 1, 0); //yellow

	return pointSetNode;
}

void AriemediCamera::RequestUpdateImage()
{
	UpdateImageData();
}

void AriemediCamera::RequestUpdateTracking()
{
	emit CameraUpdateClock();
}

void AriemediCamera::UpdateData()
{
	//show system alert information
	if (DeviceAlert::Normal != m_Tracker->getSystemAlert())
		std::cout << DeviceAlert::toString(m_Tracker->getSystemAlert()) << endl;

	//interuption check
	if (ConnectionStatus::Interruption == m_Tracker->getConnectionStatus())
	{
		std::cout << ConnectionStatus::toString(m_Tracker->getConnectionStatus()) << endl;
		return;
	}
	m_Tracker->trackingUpdate();
	std::vector<MarkerPosition> allMarkers = m_Tracker->getAllMarkers();
	//cout << "All markers number is " << allMarkers.size() << endl;
	std::vector<ToolTrackingData> toolData = m_Tracker->getTrackingData(allMarkers);
	for (int i = 0; i < toolData.size(); i++)
	{
		UpdateCameraToToolMatrix(toolData[i]);
	}

	//emit CameraUpdateClock();
}