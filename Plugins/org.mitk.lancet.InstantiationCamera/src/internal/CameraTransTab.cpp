#include "CameraTransTab.h"

CameraTransTab::CameraTransTab(Ui::InstantiationCameraControls ui, AriemediCamera* aAriemediCamera, QWidget* parent)
{
	m_UI = ui;
	m_Camera = aAriemediCamera;
	InitConnection();
	
	LoadData();
}

void CameraTransTab::InitConnection()
{
	connect(m_UI.ConverRomFileBtn, &QPushButton::clicked, this, &CameraTransTab::ConverRomFileBtnClicked);
	connect(m_UI.SelectARomPathBtn, &QPushButton::clicked, this, &CameraTransTab::SelectARomPathBtnClicked);
	connect(m_UI.SaveARomPathLineEdit, &QLineEdit::textChanged, this, &CameraTransTab::OnSaveARomPathLineEditTextChanged);
	connect(m_Camera, &AriemediCamera::ImageUpdateClock, this, &CameraTransTab::UpdateImage);
}

void CameraTransTab::ConverRomFileBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	auto files = FileIO::GetPathFilesWithFileType(filename.toStdString(), ".rom");
	for (auto file : files)
	{
		auto romfile = FileIO::CombinePath(filename.toStdString(), file);
		
		std::cout << file << std::endl;
		
	}
}

void CameraTransTab::OnSaveARomPathLineEditTextChanged()
{
	QSettings settings("Lancet", "InstantiationCamera");
	settings.setValue("AriemediSaveFilePath", m_UI.SaveARomPathLineEdit->text());  // 保存 LineEdit 内容
}

void CameraTransTab::SelectARomPathBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	m_UI.SaveARomPathLineEdit->setText(filename);
}

void CameraTransTab::UpdateImage()
{
	//m_RightVideoLabel->SetImage(m_Camera->GetRightImage(), 1920, 1200);
}

void CameraTransTab::LoadData()
{
	QSettings settings("Lancet", "InstantiationCamera");
	QString content = settings.value("AriemediSaveFilePath", "").toString();  // 加载已保存的内容
	m_UI.SaveARomPathLineEdit->setText(content);  // 将内容设置到 LineEdit
}
