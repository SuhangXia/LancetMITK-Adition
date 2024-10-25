#include "CameraConnectionTab.h"

CameraConnectionTab::CameraConnectionTab(Ui::InstantiationCameraControls ui, mitk::DataStorage* aDataStorage,
	AriemediCamera* aAriemediCamera, QWidget* parent)
{
	m_UI = ui;
	m_DataStorage = aDataStorage;
	m_Camera = aAriemediCamera;

	InitConnection();
}

void CameraConnectionTab::InitConnection()
{
	connect(m_UI.CameraConnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraConnectBtnClicked);
	connect(m_UI.CameraDisconnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraDisconnectBtnClicked);
	connect(m_UI.CameraStartBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraStartBtnClicked);
	connect(m_UI.DrawRectBtn, &QPushButton::clicked, this, &CameraConnectionTab::DrawRectBtnClicked);
	connect(m_UI.GeneratePointCloudBtn, &QPushButton::clicked, this, &CameraConnectionTab::GeneratePointCloudBtnClicked);

	connect(m_Camera, &AriemediCamera::CameraUpdateClock, this, &CameraConnectionTab::UpdateUIDisplay);
	connect(m_Camera, &AriemediCamera::ImageUpdateClock, this, &CameraConnectionTab::UpdateUIImages);
}

void CameraConnectionTab::CameraConnectBtnClicked()
{
	m_Camera->Connect();
}

void CameraConnectionTab::CameraDisconnectBtnClicked()
{
	m_Camera->Stop();
}

void CameraConnectionTab::CameraStartBtnClicked()
{
	std::vector<std::string> toolsName = { "PKADrill" };
	m_Camera->InitToolsName(toolsName);
	
	m_Camera->Start();
	auto imageSize = m_Camera->GetImageSize();
	m_Camera->SetAreaDisplay(10, 20, 10, 20, 500, 500);
	std::cout << "m_ImageWidth: " << m_ImageWidth << std::endl;
	std::cout << "m_ImageHeight: " << m_ImageHeight << std::endl;
}

void CameraConnectionTab::DrawRectBtnClicked()
{
	m_CameraRectLabel->StartDraw();
}

void CameraConnectionTab::GeneratePointCloudBtnClicked()
{
	auto node =  m_Camera->GetPointCloud();
	if (!node)
	{
		std::cout << "GetPointCloud is nullptr" << std::endl;
		return;
	}
	m_DataStorage->Add(node);
}

void CameraConnectionTab::UpdateUIDisplay()
{
	UpdateUIToolsData();
}

void CameraConnectionTab::UpdateUIToolsData()
{
	GetAndUpdateToolTip(m_Camera->GetToolTipByName("PKADrill"), m_UI.label_2);
}

void CameraConnectionTab::UpdateUIImages(char* leftImage, char* rightImage, double cols, double rows)
{
	if (leftImage != nullptr)
	{
		QImage leftimg((const uchar*)leftImage, cols, rows, cols, QImage::Format_Indexed8);
		m_UI.LeftImageLabel->setPixmap(QPixmap::fromImage(leftimg));
		m_UI.LeftImageLabel->setScaledContents(true);
		m_UI.LeftImageLabel->setAlignment(Qt::AlignCenter);
	}
	if (rightImage != nullptr)
	{
		QImage rightimg((const uchar*)rightImage, cols, rows, cols, QImage::Format_Indexed8);
		m_UI.RightImageLabel->setPixmap(QPixmap::fromImage(rightimg));
		m_UI.RightImageLabel->setScaledContents(true);
		m_UI.RightImageLabel->setAlignment(Qt::AlignCenter);
	}
}

void CameraConnectionTab::InitUI()
{
	m_UI.label->setText("Probe");
}

void CameraConnectionTab::GetAndUpdateToolTip(Eigen::Vector3d tempTip, QLabel* label)
{
	if (tempTip[0] == 0 && tempTip[1] == 0 && tempTip[2] == 0)
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

void CameraConnectionTab::UpdateSingleImage(char* aImage, QLabel* aLabel)
{
	QImage image(reinterpret_cast<uchar*>(aImage), m_ImageWidth, m_ImageHeight, QImage::Format_Grayscale8);

	aLabel->setPixmap(QPixmap::fromImage(image));
	//std::cout << "UpdateSingleImage" << std::endl;
}


