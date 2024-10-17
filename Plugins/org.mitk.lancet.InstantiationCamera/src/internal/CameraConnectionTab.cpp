#include "CameraConnectionTab.h"

CameraConnectionTab::CameraConnectionTab(Ui::InstantiationCameraControls ui, mitk::DataStorage* aDataStorage,
	AriemediCamera* aAriemediCamera, QWidget* parent)
{
	m_UI = ui;
	m_DataStorage = aDataStorage;
	m_Camera = aAriemediCamera;
}

void CameraConnectionTab::InitConnection()
{
	connect(m_UI.CameraConnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraConnectBtnClicked);
	connect(m_UI.CameraDisconnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraDisconnectBtnClicked);
	connect(m_UI.CameraStartBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraStartBtnClicked);

	connect(m_Camera, &AriemediCamera::CameraUpdateClock, this, &CameraConnectionTab::UpdateUIDisplay);
}

void CameraConnectionTab::CameraConnectBtnClicked()
{
	m_Camera->Connect();
}

void CameraConnectionTab::CameraDisconnectBtnClicked()
{
	m_Camera->Disconnect();
}

void CameraConnectionTab::CameraStartBtnClicked()
{
	m_Camera->Start();
}

void CameraConnectionTab::UpdateUIDisplay()
{
	UpdateUIToolsData();
	UpdateUIImages();
}

void CameraConnectionTab::UpdateUIToolsData()
{

}

void CameraConnectionTab::UpdateUIImages()
{
}
