/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "CzxTest.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>
#include <math.h>
#include "CalculationHelper.h"


const std::string CzxTest::VIEW_ID = "org.mitk.views.czxtest";

void CzxTest::SetFocus()
{
	// m_Controls.buttonPerformImageProcessing->setFocus();
}

void CzxTest::CreateQtPartControl(QWidget* parent)
{
	// create GUI widgets from the Qt Designer's .ui file
	m_Controls.setupUi(parent);
	InitUI();
	InitGlobalVariable();
	PKARenderHelper::InitSurfaceSelector(this->GetDataStorage(), m_Controls.mitkNodeSelectWidget_surface_regis);
	PKARenderHelper::InitSurfaceSelector(this->GetDataStorage(), m_Controls.PrePlanSelectModelWidget);
	PKARenderHelper::InitImageSelector(this->GetDataStorage(), m_Controls.PrePlanSelectImageWidget);
	PKARenderHelper::InitPointSetSelector(this->GetDataStorage(), m_Controls.mitkNodeSelectWidget_landmark_src);
	PKARenderHelper::InitPointSetSelector(this->GetDataStorage(), m_Controls.selectedPointSetWidget);
	PKARenderHelper::InitPointSetSelector(this->GetDataStorage(), m_Controls.PKAReUseRegistrationPointsSelectWidget);
	PKARenderHelper::InitSurfaceSelector(this->GetDataStorage(), m_Controls.IntraOsteotomyEndDrillTipWidget);
	PKARenderHelper::InitBoundingBoxSelector(this->GetDataStorage(), m_Controls.FemurClippedPlaneBoundingBoxWidget);
	PKARenderHelper::InitBoundingBoxSelector(this->GetDataStorage(), m_Controls.TibiaClippedPlaneBoundingBoxWidget);

	InitHardwareDeviceTabConnection();

	InitRobotRegistrationTabConnection();

	InitTCPRegistrationTabConnection();
	connect(m_Controls.VerifyImageRegistrationAccuracyBtn, &QPushButton::clicked, this, &CzxTest::VerifyImageRegistrationAccuracyBtnClicked);

	connect(m_Controls.LocateHipCenterBtn, &QPushButton::clicked, this, &CzxTest::LocateHipCenterBtnClicked);

	m_RequestRenderTimer = new QTimer(this);
	connect(m_RequestRenderTimer, SIGNAL(timeout()), this, SLOT(RenderViewAndWidget()));
	m_RequestRenderTimer->start(100);

	//TestTab
	connect(m_Controls.FemurOpacitySlider, &QSlider::valueChanged, this, [this](int value) {
		this->ChangeModelOpacityBtnClicked("femurSurface", value);
		});
	connect(m_Controls.TibiaOpacitySlider, &QSlider::valueChanged, this, [this](int value) {
		ChangeModelOpacityBtnClicked("tibiaSurface", value);
		});
	connect(m_Controls.SetLandmarkPointBtn, &QPushButton::clicked, this, &CzxTest::SetLandmarkPointBtnClicked);

	m_shortCut = new QShortcut(m_Controls.tabWidget);
	m_shortCut->setKey(tr("ctrl+b"));//ctrl
	m_shortCut->setAutoRepeat(false);

	connect(m_shortCut, &QShortcut::activated, this, &CzxTest::PrintNode);
	connect(m_Controls.selectedPointSetWidget, &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
		this, &CzxTest::OnCurrentSelectionChanged);
	auto renderWindowPart = this->GetRenderWindowPart();

	if (nullptr != renderWindowPart)
		this->RenderWindowPartActivated(renderWindowPart);

	this->OnCurrentSelectionChanged(m_Controls.selectedPointSetWidget->GetSelectedNodes());
	connect(m_Controls.CalcualtePointSetCenterBtn, &QPushButton::clicked, this, &CzxTest::CalcualtePointSetCenterBtnClicked);
	connect(m_Controls.RotateTibiaByCenterBtn, &QPushButton::clicked, this, &CzxTest::RotateTibiaByCenterBtnClicked);

	connect(m_Controls.ClockWiseBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnRotateModelButtonClicked(true);
		});
	connect(m_Controls.AntiClockWiseBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnRotateModelButtonClicked(false);
		});
	InitPreplanTabConnection();

	InitMakeProsthesisConfigTabConnection();

	InitModelRegistrationTabConnection();

	InitModelDispalyTabConnection();

	InitIntraPlanTabConnection();

	InitIntraOsteotomyTabConnection();

	connect(m_Controls.RegisterMalleolusMedialisBtn, &QPushButton::clicked, this,
		[this]() { RegisterMalleolus(MalleolusPoint::MalleolusMedialis); });
	connect(m_Controls.RegisterLateralMalleolusBtn, &QPushButton::clicked, this,
		[this]() {RegisterMalleolus(MalleolusPoint::LateralMalleolus); });

	connect(m_Controls.CaptureVerifyBtn, &QPushButton::clicked, this, &CzxTest::CapturVerifyBtnClicked);
	connect(m_Controls.ClearVerifyPointBtn, &QPushButton::clicked, this, &CzxTest::ClearVerifyPointBtnClicked);
	connect(m_Controls.VerifyPointBtn, &QPushButton::clicked, this, &CzxTest::VerifyPointBtnClicked);

	std::cout << "ITK Version: " << itk::Version::GetITKVersion() << std::endl;

	connect(m_Controls.tabWidget, &QTabWidget::currentChanged, this, &CzxTest::OnTabChanged);
}

void CzxTest::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
	const QList<mitk::DataNode::Pointer>& nodes)
{
	std::string nodeName;
	// iterate all selected objects, adjust warning visibility
	foreach(mitk::DataNode::Pointer node, nodes)
	{
		if (node.IsNull())
		{
			return;
		}

		node->GetName(nodeName);

		if (node != nullptr)
		{
			m_currentSelectedNode = node;
			m_Controls.textBrowser->append(QString::fromStdString(m_currentSelectedNode->GetName()));
			float color[3];
			node->GetColor(color);
			m_Controls.ModelColorRLineEdit->setText(QString::number(color[0]));
			m_Controls.ModelColorGLineEdit->setText(QString::number(color[1]));
			m_Controls.ModelColorBLineEdit->setText(QString::number(color[2]));
		}
	}
}

void CzxTest::RenderWindowPartActivated(mitk::IRenderWindowPart* renderWindowPart)
{
	m_Controls.poinSetListWidget->AddSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("axial")->GetSliceNavigationController());
	m_Controls.poinSetListWidget->AddSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("sagittal")->GetSliceNavigationController());
	m_Controls.poinSetListWidget->AddSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("coronal")->GetSliceNavigationController());
	
	m_PreoPreparation = new PreoPreparation(m_PKADianaAimHardwareDevice);
	m_WorldAxes = PKARenderHelper::GenerateAxesActor();
	vtkSmartPointer<vtkMatrix4x4> worldAxesMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	worldAxesMatrix->Identity();
	m_WorldAxes->SetUserMatrix(worldAxesMatrix);
	//PKARenderHelper::AddActor(renderWindowPart, m_WorldAxes);
	m_BoundingBoxInteraction = new BoundingBoxInteraction(this->GetDataStorage(), renderWindowPart);

	auto renderingManager = mitk::RenderingManager::GetInstance();
	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	auto controller = renderWindow->GetSliceNavigationController();

	renderWindow->UpdateCrosshairVisibility(false);
	renderWindow->CrosshairVisibilityChanged(false);
	/*renderWindow->GetRenderWindow()->Render();
	Render3D();
	renderingManager->RequestUpdateAll();*/
}

void CzxTest::RenderWindowPartDeactivated(mitk::IRenderWindowPart* renderWindowPart)
{
	m_Controls.poinSetListWidget->RemoveSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("axial")->GetSliceNavigationController());
	m_Controls.poinSetListWidget->RemoveSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("sagittal")->GetSliceNavigationController());
	m_Controls.poinSetListWidget->RemoveSliceNavigationController(renderWindowPart->GetQmitkRenderWindow("coronal")->GetSliceNavigationController());
}



void CzxTest::ConnectRobotBtnClicked()
{
	m_PKADianaAimHardwareDevice->ConnectRobot();
	//m_PKAHardwareDevice->ConnectKuka();
	//m_PKAHardwareDevice->StartKukaTracking(m_Controls.m_StatusWidgetKukaToolToShow);
}

void CzxTest::ConnectCameraClicked()
{
	std::filesystem::path currentPath = std::filesystem::current_path();
	std::cout << "Current working directory is: " << currentPath << std::endl;
	m_PKADianaAimHardwareDevice->ConnectCamera();

	//m_PKAHardwareDevice->ConnectNDI(m_Controls.m_StatusWidgetVegaToolToShow);
}

void CzxTest::UpdateCameraBtnClicked()
{
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}
	std::vector<QLabel*> labels{ m_Controls.PKARobotBaseRF,m_Controls.PKARobotEndRF, m_Controls.PKAFemurRF,m_Controls.PKAProbe, m_Controls.PKATibiaRF, m_Controls.PKADrill };
	m_PKADianaAimHardwareDevice->InitQLabels(labels);
	connect(m_AimoeVisualizeTimer, &QTimer::timeout, [this]() {
		m_PKADianaAimHardwareDevice->UpdateCamera();
		m_ToolDisplayHelper->UpdateTool();
		});
	m_AimoeVisualizeTimer->start(100);
}

void CzxTest::PowerOffBtnClicked()
{
	if (m_PKADianaAimHardwareDevice)
		m_PKADianaAimHardwareDevice->RobotPowerOff();
}

void CzxTest::PowerOnBtnClicked()
{
	if (m_PKADianaAimHardwareDevice)
		m_PKADianaAimHardwareDevice->RobotPowerOn();
}

void CzxTest::SelfCheckBtnClicked()
{
	m_PKAHardwareDevice->KukaSelfCheck();
}

bool CzxTest::Translate(const double axis[3])
{
	m_PKADianaAimHardwareDevice->Translate(axis, m_Controls.TranslateDistanceLineEdit->text().toDouble());
	return true;
	//return m_PKAHardwareDevice->Translate(axis, m_Controls.TranslateDistanceLineEdit->text().toDouble(), m_Controls.RobotMoveType->currentIndex());
}

bool CzxTest::Rotate(const double axis[3])
{
	m_PKADianaAimHardwareDevice->Rotate(axis, m_Controls.RotateAngleLineEdit->text().toDouble());
	return true;
	//return m_PKAHardwareDevice->Rotate(axis, m_Controls.RotateAngleLineEdit->text().toDouble(), m_Controls.RobotMoveType->currentIndex());
}

//TODO
void CzxTest::RobotAutoRegistationBtnClicked()
{
	//m_PKAHardwareDevice->AutoRobotRegistration();
	m_PKADianaAimHardwareDevice->RobotAutoRegistration();
}

void CzxTest::automoveBtnClicked()
{
	//m_PKAHardwareDevice->OldAutoMove();
}

void CzxTest::StopRobotMoveBtnClicked()
{
	m_PKADianaAimHardwareDevice->StopMove();
}

void CzxTest::ClearRobotErrorInfoBtnClicked()
{
	m_PKADianaAimHardwareDevice->CleanRobotErrorInfo();
}

//TCamera2Base TBase2End  TEnd2Flange  
double CzxTest::VerifyKneeModelAccuracy(KneeModel modelType)
{
	//make sure robot flange is toolEnd
	m_PKADianaAimHardwareDevice->GetProbeTip();
	auto probePos = m_PKADianaAimHardwareDevice->GetProbeTip();
	if (modelType == KneeModel::Femur)
	{
		//获取探针点
		//获得影像中 股骨验证点的位置
	}
	else
	{
		//获取探针点
		//获得影像中 股骨验证点的位置
	}
	return 0.0;
}

void CzxTest::DrillEndAccuracyVerifyBtnClicked()
{
	Eigen::Vector3d probeTip = m_PKADianaAimHardwareDevice->GetProbeTip();
	Eigen::Vector3d drillEnd = m_PKADianaAimHardwareDevice->GetDrillEndInCamera();
	double result = CalculationHelper::CalculateTwoPointsDistance(probeTip, drillEnd);
	m_Controls.DrillEndAccuracyResultLineEdit->setText(QString::number(result));
}

void CzxTest::SetTcpToFlangeBtnClicked()
{
	m_PKADianaAimHardwareDevice->SetTCP2Flange();
	//m_PKAHardwareDevice->SetTcpToFlange();
}

void CzxTest::RecordInitPosBtnClicked()
{
	m_PKADianaAimHardwareDevice->RecordIntialPos();
	//m_PKAHardwareDevice->RecordInitPos();
}

void CzxTest::GoToInitPosBtnClicked()
{
	m_PKADianaAimHardwareDevice->GoToInitPos();
	//m_PKAHardwareDevice->GoToInitPos();
}

void CzxTest::CaptureRobotBtnClicked()
{
	m_Controls.CaptureCountLineEdit->setText(QString::number(m_PKADianaAimHardwareDevice->CaptureRobot()));
	//m_PKAHardwareDevice->CaptureRobot();
}

void CzxTest::ResetRobotRegistrationBtnClicked()
{
	m_Controls.CaptureCountLineEdit->setText(QString::number(m_PKADianaAimHardwareDevice->ResetRobotRegistration()));
	//m_Controls.CaptureCountLineEdit->setText(QString::number(m_PKAHardwareDevice->ResetRobotRegistration()));
}

void CzxTest::SaveRobotRegistrationBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	std::string baseRF2ToBaseFileName = "T_BaseRFToBase.txt";
	std::string flangeToEndRFFileName = "T_FlangeToEndRF.txt";

	FileIO::SaveMatrix2File(FileIO::CombinePath(filename.toStdString(), baseRF2ToBaseFileName).string(), PKAData::m_TBaseRF2Base);
	FileIO::SaveMatrix2File(FileIO::CombinePath(filename.toStdString(), flangeToEndRFFileName).string(), PKAData::m_TFlange2EndRF);
}

void CzxTest::ReuseRobotRegistationBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	std::string baseToBaseRFFileName = "T_BaseRFToBase.txt";
	std::string flangeToEndRFFileName = "T_FlangeToEndRF.txt";
	vtkSmartPointer<vtkMatrix4x4> baseRFToBase = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> flangeToEndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	FileIO::ReadTextFileAsvtkMatrix(FileIO::CombinePath(filename.toStdString(), baseToBaseRFFileName).string(), baseRFToBase);
	FileIO::ReadTextFileAsvtkMatrix(FileIO::CombinePath(filename.toStdString(), flangeToEndRFFileName).string(), flangeToEndRF);
	PKAData::m_TBaseRF2Base->DeepCopy(baseRFToBase);
	PKAData::m_TFlange2EndRF->DeepCopy(flangeToEndRF);
	PrintDataHelper::CoutMatrix("PKAData::m_TBaseRF2Base", PKAData::m_TBaseRF2Base);
	PrintDataHelper::CoutMatrix("PKAData::m_TFlange2EndRF", PKAData::m_TFlange2EndRF);
	//m_PKAHardwareDevice->ReuseRobotRegistation();
}

void CzxTest::ReadRobotJointAnglesBtnClicked()
{
	auto angles = m_PKADianaAimHardwareDevice->GetJointAngles();
	for (int i = 0; i < angles.size(); ++i)
	{
		m_RobotJointAngleLineEdits[i]->setText(QString::number(angles[i] * 180 / PI));
	}
}

void CzxTest::SetRobotJointAnglesBtnClicked()
{
	double angles[7] = { 0.0 };
	for (int i = 0; i < m_RobotJointAngleLineEdits.size(); ++i)
	{
		angles[i] = m_RobotJointAngleLineEdits[i]->text().toDouble() / 180 * PI;
	}

	bool ret = m_PKADianaAimHardwareDevice->SetJointAngles(angles);
}

void CzxTest::GetRobotJointsLimitBtnClicked()
{
	auto range = m_PKADianaAimHardwareDevice->GetJointsPositionRange();
	for (int i = 0; i < range[0].size(); ++i)
	{
		range[0][i] = range[0][i] * 180 / PI;
		range[1][i] = range[1][i] * 180 / PI;
	}
	PrintDataHelper::AppentTextBrowserVector(m_Controls.textBrowser, "min Range: ", range[0]);
	PrintDataHelper::AppentTextBrowserVector(m_Controls.textBrowser, "max Range: ", range[1]);
}

void CzxTest::SetRobotPositionModeBtnClicked()
{
	bool ret = m_PKADianaAimHardwareDevice->SetPositionMode();
	if (ret)
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置为位置模式"));
	}
	else
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置失败"));
	}
}

void CzxTest::SetRobotJointsImpedanceModelBtnClicked()
{
	bool ret = m_PKADianaAimHardwareDevice->SetJointImpendanceMode();
	if (ret)
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置为关节空间阻抗模式"));
	}
	else
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置失败"));
	}
}

void CzxTest::SetRobotCartImpedanceModeBtnClicked()
{
	bool ret = m_PKADianaAimHardwareDevice->SetCartImpendanceMode();
	if (ret)
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置为笛卡尔空间阻抗模式"));
	}
	else
	{
		m_Controls.textBrowser->append(QString::fromLocal8Bit("设置失败"));
	}
}

void CzxTest::GetProbeEndPos(TCPRegistraionMethod method)
{
	int& currentCount = (method == TCPRegistraionMethod::FourPoints) ? ProbEndCountOne : ProbEndCountTwo;
	int maxCount = (method == TCPRegistraionMethod::FourPoints) ? 3 : 2;
	QLineEdit* countLineEdit = (method == TCPRegistraionMethod::FourPoints) ? m_Controls.FourProbeEndCountLineEdit : m_Controls.ThreeProbeEndCountLineEdit;
	std::vector<Eigen::Vector3d>& probeEndVector = (method == TCPRegistraionMethod::FourPoints) ? probeEndOneVector : probeEndTwoVector;

	if (currentCount == maxCount)
	{
		QMessageBox msgBox;
		msgBox.setText(QString::fromLocal8Bit("警告"));
		msgBox.setInformativeText(QString::fromLocal8Bit("采集数量已达上限，请重置"));
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
		return;
	}

	currentCount++;
	countLineEdit->setText(QString::number(currentCount));

	Eigen::Vector3d vec;
	if (PKAData::m_VegaSource && PKAData::m_VegaSource->IsConnected())
	{
		auto vegaData = PKAData::m_VegaSource->GetOutput("Probe");
		vec[0] = vegaData->GetAffineTransform3D()->GetOffset()[0];
		vec[1] = vegaData->GetAffineTransform3D()->GetOffset()[1];
		vec[2] = vegaData->GetAffineTransform3D()->GetOffset()[2];
	}
	else
	{
		std::cout << "cannot get probe end Data";
	}
	std::string str = "Probe End Pos " + std::to_string(currentCount) + ": ";
	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, vec, str.c_str());

	probeEndVector[currentCount - 1] = vec;
}

void CzxTest::SetDrillEndTCPBtnClicked()
{
	if (!m_PKADianaAimHardwareDevice)
		return;
	m_PKADianaAimHardwareDevice->SetTCP(m_PKADianaAimHardwareDevice->CalculateFlangeToDrillEnd());
}

void CzxTest::DisplayCameraToTCPAxesBtnClicked()
{
	auto renderpart = this->GetRenderWindowPart();
	if (!m_IsDisplayTCP)
	{
		m_IsDisplayTCP = !m_IsDisplayTCP;

		m_PKADianaAimHardwareDevice->DisplayBase2TCP(renderpart);
		connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateTCPAxesActor()));
	}
	else
	{
		m_IsDisplayTCP = !m_IsDisplayTCP;
		disconnect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateTCPAxesActor()));
		m_PKADianaAimHardwareDevice->HideBase2Tcp(renderpart);
	}
}

void CzxTest::SetTCPByProbeTipBtnClicked()
{
	vtkSmartPointer<vtkMatrix4x4> TBase2Flange = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2Flange->DeepCopy(m_PKADianaAimHardwareDevice->GetBase2TCP());

	//auto rotation = CalculationHelper::GetRotationFromMatrix4x4(TBase2Flange);
	auto probeTipInCamera = m_PKADianaAimHardwareDevice->GetProbeTip();

	vtkSmartPointer<vtkMatrix4x4> TCamera2BaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TBaseRF2Base = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2BaseRF->DeepCopy(PKAData::m_TCamera2BaseRF);
	TBaseRF2Base->DeepCopy(PKAData::m_TBaseRF2Base);
	vtkSmartPointer<vtkMatrix4x4> TFlange2Camera = vtkSmartPointer<vtkMatrix4x4>::New();

	TFlange2Camera->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TCamera2BaseRF, TBaseRF2Base, TBase2Flange));
	TFlange2Camera->Invert();

	auto probeTipInFlange = CalculationHelper::TransformByMatrix(probeTipInCamera, TFlange2Camera);
	vtkSmartPointer<vtkTransform> newTCPTransform = vtkSmartPointer<vtkTransform>::New();
	newTCPTransform->Translate(probeTipInFlange.data());
	vtkSmartPointer<vtkMatrix4x4> newTCP = vtkSmartPointer<vtkMatrix4x4>::New();

	newTCPTransform->GetMatrix(newTCP);
	m_PKADianaAimHardwareDevice->SetTCP(newTCP);
}

void CzxTest::ReadRobotImpedaBtnClicked()
{
	auto Impeda = m_PKADianaAimHardwareDevice->GetRobotImpeda();
	m_Controls.arrStiffXLineEdit->setText(QString::number(Impeda[0]));
	m_Controls.arrStiffYLineEdit->setText(QString::number(Impeda[1]));
	m_Controls.arrStiffZLineEdit->setText(QString::number(Impeda[2]));
	m_Controls.arrStiffRXLineEdit->setText(QString::number(Impeda[3]));
	m_Controls.arrStiffRYLineEdit->setText(QString::number(Impeda[4]));
	m_Controls.arrStiffRZLineEdit->setText(QString::number(Impeda[5]));
	m_Controls.DampingRatioLineEdit->setText(QString::number(Impeda[6]));
}

void CzxTest::SetRobotImpedaBtnClicked()
{
	double data[7] = {
		m_Controls.arrStiffXLineEdit->text().toDouble(),
	m_Controls.arrStiffYLineEdit->text().toDouble(),
	m_Controls.arrStiffZLineEdit->text().toDouble(),
	m_Controls.arrStiffRXLineEdit->text().toDouble(),
	m_Controls.arrStiffRYLineEdit->text().toDouble(),
	m_Controls.arrStiffRZLineEdit->text().toDouble(),
	m_Controls.DampingRatioLineEdit->text().toDouble()
	};
	m_PKADianaAimHardwareDevice->SetRobotImpeda(data);
}

void CzxTest::SaveImageRegistrationResultBtnClicked()
{
	if (PKAData::m_VegaToolStorage.IsNotNull())
	{
		QFileDialog* fileDialog = new QFileDialog;
		fileDialog->setDefaultSuffix("IGTToolStorage");
		QString suffix = "IGT Tool Storage (*.IGTToolStorage)";

		// 获取用户桌面的路径
		QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

		// 设置默认文件名为桌面路径加上存储对象的名称
		QString defaultFileName = desktopPath + "/" + QString::fromStdString(PKAData::m_VegaToolStorage->GetName());
		QString filename = fileDialog->getSaveFileName(nullptr, tr("Save Navigation Tool Storage"), defaultFileName, suffix, &suffix);

		if (filename.isEmpty()) return; // 用户取消保存操作

		// 检查文件后缀
		QFileInfo file(filename);
		if (file.suffix().isEmpty()) filename += ".IGTToolStorage";

		// 序列化工具存储对象
		mitk::NavigationToolStorageSerializer::Pointer mySerializer = mitk::NavigationToolStorageSerializer::New();

		try
		{
			mySerializer->Serialize(filename.toStdString(), PKAData::m_VegaToolStorage);
		}
		catch (const mitk::IGTIOException& e)
		{
			m_Controls.textBrowser->append(QString::fromStdString("Error: " + std::string(e.GetDescription())));
			return;
		}

		m_Controls.textBrowser->append(QString::fromStdString(PKAData::m_VegaToolStorage->GetName() + " saved"));
	}
}

void CzxTest::VerifyImageRegistrationAccuracyBtnClicked()
{
	KneeModel registrationType = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	vtkSmartPointer<vtkMatrix4x4> TCamera2RF = vtkSmartPointer<vtkMatrix4x4>::New();
	mitk::Surface::Pointer surface;
	if (registrationType == KneeModel::Femur)
	{
		TCamera2RF->DeepCopy(PKAData::m_TCamera2FemurRF);
		surface = this->GetDataStorage()->GetNamedObject<mitk::Surface>(PKAData::m_FemurClippedSurfaceNodeName.toStdString());
	}
	else
	{
		TCamera2RF->DeepCopy(PKAData::m_TCamera2TibiaRF);
		surface = this->GetDataStorage()->GetNamedObject<mitk::Surface>(PKAData::m_TibiaClippedSurfaceNodeName.toStdString());
	}

	//vtkSmartPointer<vtkKdTree> pKdTree =
	//	vtkSmartPointer<vtkKdTree>::New();
	//pKdTree->BuildLocatorFromPoints(surface->GetVtkPolyData()->GetPoints());
	//double rms = 0;
	//auto verifyNode = this->GetDataStorage()->GetNamedNode(mNodeName_FemurRegistrationVerify.toLatin1());
	//verifyNode->SetVisibility(true);

	////创建Sphere的球，然后加入
	////auto mitkRenderWindow = multiWidget->GetRenderWindow("3d");
	//mitk::PointSetVtkMapper3D::Pointer vtk3DMapper =
	//	dynamic_cast<mitk::PointSetVtkMapper3D*>(verifyNode->GetMapper(2));
	//vtkSmartPointer<vtkPropAssembly> prop =
	//	dynamic_cast<vtkPropAssembly*>(vtk3DMapper->GetVtkProp(this->ui->widgetFemurRegisterRender->GetRenderWindow("3d")->GetRenderer()));

	//int count = 0;
	//for (int i = 0; i < screenPointWidget->GetNumberOfPoints(); i++)
	//{
	//	mitk::PointSet::PointType point = screenPointWidget->GetPoint(i);
	//	double cache[3];
	//	pTransformVerify->TransformPoint(point.GetDataPointer(), cache);
	//	double distance;
	//	pKdTree->FindClosestPoint(cache, distance);
	//	rms += distance;
	//	double rgb[3] = { 0, 0, 0 };
	//	if (distance < 0.5)
	//	{
	//		rgb[1] = 1;
	//	}
	//	else if (distance < 1.5)
	//	{
	//		rgb[0] = rgb[1] = 1;
	//	}
	//	else if (distance < 5)
	//	{
	//		rgb[0] = 1;
	//	}
	//	else
	//	{
	//		continue;
	//	}
	//	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	//	sphere->SetRadius(2.0);
	//	sphere->SetCenter(cache);
	//	sphere->SetThetaResolution(20);
	//	sphere->SetPhiResolution(20);

	//	vtkNew<vtkPolyDataMapper> mapper;
	//	mapper->SetInputConnection(sphere->GetOutputPort());

	//	vtkNew<vtkActor> actor;
	//	actor->SetMapper(mapper);
	//	actor->GetProperty()->SetColor(rgb[0], rgb[1], rgb[2]);
	//	prop->AddPart(actor);
	//	count++;
	//}
	//rms /= screenPointWidget->GetNumberOfPoints();
	//dataStorage->GetNamedNode(mNodeName_Probe.toLatin1())->SetVisibility(true);
	//dataStorage->GetNamedNode(mNodeName_FemurVerifyPoint.toLatin1())->SetVisibility(true);
	//this->isProbeing = true;
	//actionIndex = 1;
	////PrintLog << "ValidateTransformedVegaToFemurClippedICPRMS: " << rms;
	////PrintLog << "Validate point number: " << count;
	//qDebug() << "ValidateTransformedVegaToFemurClippedICPRMS: " << rms;
	//qDebug() << "Validate point number: " << count;
	//if (rms > 2.0)
	//{
	//	QMessageBox::information(NULL, tr("提示"), tr("误差较大，建议重新配准，点击\"清除所有点\"\n请按照蓝色的点循序进行配准！"), tr("关闭"));
	//}
}

void CzxTest::SaveImageRegistraionResultBtnClicked()
{
	KneeModel kneeModel = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	if (kneeModel == KneeModel::Femur)
	{
		//std::string camera2FemurPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TCamera2FemurImageMatrix.txt";
		std::string femurRF2FemurPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TFemurRF2FemurImageMatrix.txt";

		//FileIO::SaveMatrix2File(camera2FemurPath, PKAData::m_TCamera2FemurImage);
		FileIO::SaveMatrix2File(femurRF2FemurPath, PKAData::m_TFemurRF2FemurImage);
	}
	else
	{
		//std::string camera2TibiaPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TCamera2TibiaImageMatrix.txt";
		std::string tibiaRF2TibiaPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TTibiaRF2TibiaImageMatrix.txt";
		//FileIO::SaveMatrix2File(camera2TibiaPath, PKAData::m_TCamera2TibiaImage);
		FileIO::SaveMatrix2File(tibiaRF2TibiaPath, PKAData::m_TTibiaRF2TibiaImage);
	}
}

void CzxTest::ReuseImageRegistrationBtnClicked()
{
	KneeModel kneeModel = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	if (kneeModel == KneeModel::Femur)
	{
		std::string femurRF2FemurPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TFemurRF2FemurImageMatrix.txt";
		vtkSmartPointer<vtkMatrix4x4> femurRF2Femur = vtkSmartPointer<vtkMatrix4x4>::New();
		FileIO::ReadTextFileAsvtkMatrix(femurRF2FemurPath, femurRF2Femur);
		PrintDataHelper::CoutMatrix("PKAData::m_TFemurRF2FemurImage", femurRF2Femur);
		PKAData::m_TFemurRF2FemurImage->DeepCopy(femurRF2Femur);
	}
	else
	{
		std::string tibiaRF2TibiaPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TTibiaRF2TibiaImageMatrix.txt";
		vtkSmartPointer<vtkMatrix4x4> tibiaRF2Tibia = vtkSmartPointer<vtkMatrix4x4>::New();
		FileIO::ReadTextFileAsvtkMatrix(tibiaRF2TibiaPath, tibiaRF2Tibia);
		PrintDataHelper::CoutMatrix("PKAData::m_TTibiaRF2TibiaImage", tibiaRF2Tibia);
		PKAData::m_TTibiaRF2TibiaImage->DeepCopy(tibiaRF2Tibia);
	}
}

void CzxTest::OnPKAReUseRegistrationPointsSelectWidgetChanged()
{
	if (!m_Controls.PKAReUseRegistrationPointsSelectWidget->GetSelectedNode())
		return;
	mitk::PointSet* pointSet = (mitk::PointSet*)m_Controls.PKAReUseRegistrationPointsSelectWidget->GetSelectedNode()->GetData();
	int size = pointSet->GetSize();
	m_SelectedRegistrationPoint->Clear();
	for (int i = 0; i < size; ++i)
	{
		auto point = pointSet->GetPoint(i);
		m_SelectedRegistrationPoint->InsertPoint(point, i);
	}
	m_Controls.CurrentRegistrationCountLineEdit->setText(QString::number(size));
	std::cout << "OnPKAReUseRegistrationPointsSelectWidgetChanged: " << m_SelectedRegistrationPoint->GetSize() << std::endl;
}

void CzxTest::UseRegistrationBtnClicked()
{
	KneeModel kneeModel = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	if (kneeModel == KneeModel::Femur)
	{
		m_ToolDisplayHelper->UsingFemurRegistration();
	}
	else
	{
		m_ToolDisplayHelper->UsingTibiaRegistration();
	}
}

void CzxTest::CapturVerifyBtnClicked()
{
	vtkSmartPointer<vtkMatrix4x4> TRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	Eigen::Vector3d probePos = m_PKADianaAimHardwareDevice->GetProbeTip();
	if (m_Controls.FemurVerifyPointBtn->isChecked())
	{
		TRF2Camera->DeepCopy(PKAData::m_TCamera2FemurRF);
		TRF2Camera->Invert();
		PKAData::m_FemurVerifyPointInFemurRF = CalculationHelper::TransformByMatrix(probePos, TRF2Camera);
		m_IsCaptureFemurVerifyPoint = true;
	}
	else
	{
		TRF2Camera->DeepCopy(PKAData::m_TCamera2TibiaRF);
		TRF2Camera->Invert();
		PKAData::m_TibiaVerifyPointInTibiaRF = CalculationHelper::TransformByMatrix(probePos, TRF2Camera);
		m_IsCaptureTibiaVerifyPoint = true;
	}
}

void CzxTest::ClearVerifyPointBtnClicked()
{
	if (m_Controls.FemurVerifyPointBtn->isChecked())
	{
		m_IsCaptureFemurVerifyPoint = false;
	}
	else
	{
		m_IsCaptureTibiaVerifyPoint = false;
	}
}

void CzxTest::VerifyPointBtnClicked()
{
	double distance = 0.0;
	Eigen::Vector3d verifyPointInRF = m_PKADianaAimHardwareDevice->GetProbeTip();
	vtkSmartPointer<vtkMatrix4x4> TRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	if (m_Controls.FemurVerifyPointBtn->isChecked() && m_IsCaptureFemurVerifyPoint)
	{
		TRF2Camera->DeepCopy(PKAData::m_TCamera2FemurRF);
		TRF2Camera->Invert();
		verifyPointInRF = CalculationHelper::TransformByMatrix(m_PKADianaAimHardwareDevice->GetProbeTip(), TRF2Camera);
		distance = CalculationHelper::CalculateTwoPointsDistance(PKAData::m_FemurVerifyPointInFemurRF, verifyPointInRF);
	}

	if (m_Controls.TibiaVerifyPointBtn->isChecked() && m_IsCaptureTibiaVerifyPoint)
	{
		TRF2Camera->DeepCopy(PKAData::m_TCamera2TibiaRF);
		TRF2Camera->Invert();
		verifyPointInRF = CalculationHelper::TransformByMatrix(m_PKADianaAimHardwareDevice->GetProbeTip(), TRF2Camera);
		distance = CalculationHelper::CalculateTwoPointsDistance(PKAData::m_TibiaVerifyPointInTibiaRF, verifyPointInRF);
	}
	m_Controls.VerifyPointDistanceLabel->setText(QString::number(distance));
}

void CzxTest::RegisterMalleolus(MalleolusPoint malleolusPoint)
{
	auto pos = m_PKADianaAimHardwareDevice->GetProbeTip();
	vtkSmartPointer<vtkMatrix4x4> TTibiaRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibiaRF2Camera->DeepCopy(PKAData::m_TCamera2TibiaRF);
	TTibiaRF2Camera->Invert();
	pos = CalculationHelper::TransformByMatrix(pos, TTibiaRF2Camera);
	//TODO  读取探针末端信息 转换到tibiaRF坐标系下
	if (malleolusPoint == MalleolusPoint::MalleolusMedialis)
	{
		PKAData::m_IntraMedialMalleolusInTibiaRF = pos;
		PrintDataHelper::CoutArray(pos, "RegisterMalleolus MalleolusMedialis: ");
	}
	else
	{
		PKAData::m_IntraLateralMalleolusInTibiaRF = pos;
		PrintDataHelper::CoutArray(pos, "RegisterMalleolus LateralMalleolus: ");
	}
}

void CzxTest::SelectRegistrationPointBtnClicked()
{
	std::cout << "SelectRegistrationPointBtnClicked" << std::endl;
	KneeModel registrationType = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	Eigen::Vector3d pointData = m_PKADianaAimHardwareDevice->GetProbeTip();
	int size = m_SelectedRegistrationPoint->GetSize();
	std::cout << "m_SelectedRegistrationPoint->GetSize()" << m_SelectedRegistrationPoint->GetSize() << std::endl;
	mitk::PointSet::PointType pt;
	Eigen::Vector3d pointInRF;
	if (registrationType == KneeModel::Femur)
	{
		vtkSmartPointer<vtkMatrix4x4> TFemurRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
		TFemurRF2Camera->DeepCopy(PKAData::m_TCamera2FemurRF);
		TFemurRF2Camera->Invert();
		pointInRF = CalculationHelper::TransformByMatrix(pointData, TFemurRF2Camera);
	}
	else
	{
		vtkSmartPointer<vtkMatrix4x4> TTibiaRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
		TTibiaRF2Camera->DeepCopy(PKAData::m_TCamera2TibiaRF);
		TTibiaRF2Camera->Invert();
		pointInRF = CalculationHelper::TransformByMatrix(pointData, TTibiaRF2Camera);
	}

	pt[0] = pointInRF[0];
	pt[1] = pointInRF[1];
	pt[2] = pointInRF[2];
	PrintDataHelper::CoutArray(pointInRF, "Point in Femurrf:");
	m_SelectedRegistrationPoint->InsertPoint(size, pt);

	m_Controls.CurrentRegistrationCountLineEdit->setText(QString::number(m_SelectedRegistrationPoint->GetSize()));

	UpdateRegistrationPointState(size + 1);
}

void CzxTest::RemoveLastRegistrationPointBtnClicked()
{
	m_SelectedRegistrationPoint->RemovePointAtEnd();
	std::cout << "RemoveLastRegistrationPointBtnClicked" << std::endl;
	m_Controls.CurrentRegistrationCountLineEdit->setText(QString::number(m_SelectedRegistrationPoint->GetSize()));
}

void CzxTest::ClearRegistrationPointBtnClicked()
{
	m_SelectedRegistrationPoint->Clear();
	std::cout << "ClearRegistrationPointBtnClicked" << std::endl;
	m_Controls.CurrentRegistrationCountLineEdit->setText(QString::number(m_SelectedRegistrationPoint->GetSize()));
}

void CzxTest::OnMitkNodeSelectWidget_landmark_srcChanged()
{
	if (!m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode())
		return;
	mitk::PointSet* points = (mitk::PointSet*)m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode()->GetData();
	m_Controls.TotalRegistrationCountLineEdit->setText(QString::number(points->GetSize()));
}

//将相机下的点的坐标转换到骨头Renference下 得到FemurRF2FemurImage关系
void CzxTest::RegisterImageBtnClicked()
{
	KneeModel registrationType = m_Controls.ImageRegistrationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	mitk::PointSet* modelPoints = (mitk::PointSet*)m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode()->GetData();
	mitk::Surface* modelSurface = (mitk::Surface*)m_Controls.mitkNodeSelectWidget_surface_regis->GetSelectedNode()->GetData();
	m_ModelRegistration->SetICPInput(m_SelectedRegistrationPoint, modelPoints, modelSurface);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkPoints> transformedPoints = vtkSmartPointer<vtkPoints>::New();
	double registrationVerifyPointColor[3] = { 0.1,0.1,0.1 };
	if (registrationType == KneeModel::Femur)
	{
		m_ModelRegistration->ComputeFemur(PKAData::m_IntraHipCenterInFemurRF, PKAData::m_HipCenter);
		std::cout << "Femur Registration" << std::endl;
		auto datanode = this->GetDataStorage()->GetNamedNode("FemurSelectedRegistrationPoint");
		if (!datanode)
		{
			mitk::DataNode::Pointer modelRegistrationDataNode = mitk::DataNode::New();
			modelRegistrationDataNode->SetName("FemurSelectedRegistrationPoint");
			modelRegistrationDataNode->SetData(m_SelectedRegistrationPoint);
			this->GetDataStorage()->Add(modelRegistrationDataNode);
		}
		else
		{
			datanode->SetData(m_SelectedRegistrationPoint);
		}
		vtkSmartPointer<vtkMatrix4x4> TFemurRF2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
		TFemurRF2FemurImage->DeepCopy(m_ModelRegistration->GetICPMatrix());
		TFemurRF2FemurImage->Invert();
		PKAData::m_TFemurRF2FemurImage->DeepCopy(TFemurRF2FemurImage);

		vtkSmartPointer<vtkPoints> verifyPoints = m_ModelRegistration->GetScreenPoint();
		vtkSmartPointer<vtkTransform> verifyPointsTrasnform = vtkSmartPointer<vtkTransform>::New();
		verifyPointsTrasnform->SetMatrix(m_ModelRegistration->GetICPMatrix());

		verifyPointsTrasnform->TransformPoints(verifyPoints, transformedPoints);
		PKARenderHelper::AddvtkPointsToMitk(this->GetDataStorage(), transformedPoints, PKAData::m_FemurRegistrationVerifyNodeName.toStdString(), 2, registrationVerifyPointColor);
	}
	else
	{
		m_ModelRegistration->ComputeTibia(PKAData::m_IntraMedialMalleolusInTibiaRF, PKAData::m_IntraLateralMalleolusInTibiaRF,
			PKAData::m_MedialMalleolus, PKAData::m_LateralMalleolus);
		auto datanode = this->GetDataStorage()->GetNamedNode("TibiaSelectedRegistrationPoint");
		if (!datanode)
		{
			mitk::DataNode::Pointer modelRegistrationDataNode = mitk::DataNode::New();
			modelRegistrationDataNode->SetName("TibiaSelectedRegistrationPoint");
			modelRegistrationDataNode->SetData(m_SelectedRegistrationPoint);
			this->GetDataStorage()->Add(modelRegistrationDataNode);
		}
		else
		{
			datanode->SetData(m_SelectedRegistrationPoint);
		}

		vtkSmartPointer<vtkMatrix4x4> TRF2Image = vtkSmartPointer<vtkMatrix4x4>::New();
		TRF2Image->DeepCopy(m_ModelRegistration->GetICPMatrix());
		TRF2Image->Invert();
		PKAData::m_TTibiaRF2TibiaImage->DeepCopy(TRF2Image);

		vtkSmartPointer<vtkPoints> verifyPoints = m_ModelRegistration->GetScreenPoint();
		vtkSmartPointer<vtkTransform> verifyPointsTrasnform = vtkSmartPointer<vtkTransform>::New();
		verifyPointsTrasnform->SetMatrix(m_ModelRegistration->GetICPMatrix());
		verifyPointsTrasnform->TransformPoints(verifyPoints, transformedPoints);
		PKARenderHelper::AddvtkPointsToMitk(this->GetDataStorage(), transformedPoints, PKAData::m_TibiaRegistrationVerifyNodeName.toStdString(), 2, registrationVerifyPointColor);
	}
	m_Controls.textBrowser->append(QString::number(m_ModelRegistration->GetRegistrationRMS()));
	std::cout << "ModelRegistration rms: " << m_ModelRegistration->GetRegistrationRMS() << std::endl;
}

void CzxTest::StartSelectPointBtnClicked()
{
	std::cout << "StartSelectPointBtnClicked" << std::endl;
	auto node = this->GetDataStorage()->GetNamedNode("CopyRegistrationPointDataNode");
	if (node)
	{
		this->GetDataStorage()->Remove(node);
	}
	m_SelectedRegistrationPoint->Clear();
	m_Controls.CurrentRegistrationCountLineEdit->setText(QString::number(0));
	mitk::PointSet::Pointer copyPointSet = mitk::PointSet::New();
	mitk::DataNode::Pointer m_CopyRegistrationPointDataNode = mitk::DataNode::New();
	mitk::PointSet* targetPointSet = (mitk::PointSet*)m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode()->GetData();

	int size = targetPointSet->GetSize();
	for (int i = 0; i < size; ++i)
	{
		auto scalarType = targetPointSet->GetPoint(i).data();
		Eigen::Vector3d point = Eigen::Vector3d(scalarType[0], scalarType[1], scalarType[2]);
		std::string sizeStr = std::to_string(i);
		sizeStr = "StartSelectPointBtnClicked Point " + sizeStr;
		PrintDataHelper::CoutArray(point, sizeStr.c_str());
		mitk::PointSet::PointType pt;
		pt[0] = point[0];
		pt[1] = point[1];
		pt[2] = point[2];
		copyPointSet->InsertPoint(i, pt);
	}
	std::cout << "copyPointSet Size: " << copyPointSet->GetSize() << std::endl;
	m_CopyRegistrationPointDataNode->SetData(copyPointSet);
	//m_CopyRegistrationPointDataNode->Print(std::cout);
	m_CopyRegistrationPointDataNode->SetName("CopyRegistrationPointDataNode");

	m_CopyRegistrationPointDataNode->SetFloatProperty("pointsize", 1);
	m_CopyRegistrationPointDataNode->SetColor(1, 1, 1);

	this->GetDataStorage()->Add(m_CopyRegistrationPointDataNode);

	//创建Sphere的球，然后加入
	mitk::PointSetVtkMapper3D::Pointer vtk3DMapper = dynamic_cast<mitk::PointSetVtkMapper3D*>(m_CopyRegistrationPointDataNode->GetMapper(2));
	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	vtkSmartPointer<vtkPropAssembly> prop = dynamic_cast<vtkPropAssembly*>(vtk3DMapper->GetVtkProp(renderWindow->GetRenderer()));
	for (int index = 0; index < copyPointSet->GetSize(); index++)
	{
		vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
		sphere->SetRadius(3.0f / 2.0);
		sphere->SetCenter(copyPointSet->GetPoint(index)[0], copyPointSet->GetPoint(index)[1], copyPointSet->GetPoint(index)[2]);
		sphere->SetThetaResolution(20);
		sphere->SetPhiResolution(20);

		vtkNew<vtkPolyDataMapper> mapper;
		mapper->SetInputConnection(sphere->GetOutputPort());

		vtkNew<vtkActor> actor;
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(1, 1, 1);
		prop->AddPart(actor);
	}
	UpdateRegistrationPointState(0);
}

void CzxTest::UpdateRegistrationPointState(int index)
{
	std::cout << "UpdateRegistrationPointState index: " << index << std::endl;
	if (index < 0)
		return;
	auto m_CopyRegistrationPointDataNode = this->GetDataStorage()->GetNamedNode("CopyRegistrationPointDataNode");
	mitk::PointSet* pointSet = dynamic_cast<mitk::PointSet*>(m_CopyRegistrationPointDataNode->GetData());
	//m_CopyRegistrationPointDataNode
	mitk::PointSetVtkMapper3D::Pointer vtk3DMapperCopy =
		dynamic_cast<mitk::PointSetVtkMapper3D*>(m_CopyRegistrationPointDataNode->GetMapper(2));
	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	auto vtkmapper = vtk3DMapperCopy->GetVtkProp(renderWindow->GetRenderer());

	vtkSmartPointer<vtkPropAssembly> propCopy = dynamic_cast<vtkPropAssembly*>(vtkmapper);
	auto actors = vtkSmartPointer<vtkPropCollection>::New();
	propCopy->GetParts()->InitTraversal();
	propCopy->GetActors(actors);
	//std::cout << "propCopy->GetParts()->GetNumberOfItems()" << propCopy->GetParts()->GetNumberOfItems() << std::endl;
	for (int i = 0; i < index && i < propCopy->GetParts()->GetNumberOfItems(); i++)
	{
		auto propobj = propCopy->GetParts()->GetNextProp();
		if (propobj)
		{
			propobj->VisibilityOff();
		}
		else
		{
			std::cout << "propobj is null" << std::endl;
		}
	}
	////auto indexPre = index - 1;

	vtkActor* actorBlue = vtkActor::SafeDownCast(propCopy->GetParts()->GetNextProp());
	if (actorBlue)
	{
		//std::cout << "UpdateRegistrationPointState actorBlue" << std::endl;
		actorBlue->GetProperty()->SetColor(0, 0, 1);

		vtkSmartPointer<vtkSphereSource> sphereTemp = vtkSmartPointer<vtkSphereSource>::New();
		sphereTemp->SetRadius(3.0);
		sphereTemp->SetThetaResolution(20);
		sphereTemp->SetPhiResolution(20);
		sphereTemp->SetCenter(pointSet->GetPoint(index)[0], pointSet->GetPoint(index)[1], pointSet->GetPoint(index)[2]);
		actorBlue->GetMapper()->SetInputConnection(sphereTemp->GetOutputPort());

		if ((index % 5) == 0)
		{
			for (int i = 0; i < 4; i++)
			{
				vtkActor* actorTemp = vtkActor::SafeDownCast(propCopy->GetParts()->GetNextProp());
				if (!actorTemp)
				{
					//std::cout << "UpdateRegistrationPointState actorTemp is nullptr" << std::endl;
					return;
				}
				actorTemp->GetProperty()->SetColor(0, 0, 1);
			}
		}
	}
}

void CzxTest::LocateHipCenterBtnClicked()
{
	if (FemurPositions.size() >= 100)
	{
		QMessageBox msgBox;
		msgBox.setText(QString::fromLocal8Bit("Tip"));
		msgBox.setInformativeText(QString::fromLocal8Bit("采集数量已达上限，请重置"));
		msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
		msgBox.setDefaultButton(QMessageBox::Cancel);
		int ret = msgBox.exec();

		if (ret == QMessageBox::Cancel) {
			return;
		}
	}
	double error = 0.0;
	m_PreoPreparation->SelectFemurPositions(m_Controls.HipCenterPosProgressBar, error);
	m_Controls.textBrowser->append("femur hip error: " + QString::number(error));
}

void CzxTest::InitUI()
{
	//Set Text background and foreground color
	m_Controls.textBrowser->setStyleSheet("QTextBrowser { background-color: white; color: black; }");
	//Init progressbar for select hip center
	m_Controls.HipCenterPosProgressBar->setRange(0, 100);
	m_Controls.HipCenterPosProgressBar->setValue(0);

	m_Controls.TibiaOpacitySlider->setValue(100);
	m_Controls.TibiaOpacitySlider->setTickInterval(10);
	m_Controls.FemurOpacitySlider->setValue(100);
	m_Controls.FemurOpacitySlider->setTickInterval(10);
}

void CzxTest::InitHardwareDeviceTabConnection()
{
	connect(m_Controls.ConnectKukaBtn, &QPushButton::clicked, this, &CzxTest::ConnectRobotBtnClicked);
	connect(m_Controls.PowerOnBtn, &QPushButton::clicked, this, &CzxTest::PowerOnBtnClicked);
	connect(m_Controls.PowerOffBtn, &QPushButton::clicked, this, &CzxTest::PowerOffBtnClicked);
	connect(m_Controls.SelfCheckBtn, &QPushButton::clicked, this, &CzxTest::SelfCheckBtnClicked);
	connect(m_Controls.ConnectNDIBtn, &QPushButton::clicked, this, &CzxTest::ConnectCameraClicked);
	connect(m_Controls.UpdateCameraBtn, &QPushButton::clicked, this, &CzxTest::UpdateCameraBtnClicked);
}

void CzxTest::InitRobotRegistrationTabConnection()
{
	//Translate
	connect(m_Controls.RobotXPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 1, 0,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotXMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { -1, 0,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotYPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 1,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotYMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, -1,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotZPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 0,1 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotZMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 0,-1 };
		Translate(movementDirection);
		});

	//Rotate
	connect(m_Controls.RobotRXPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 1, 0,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRXMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { -1, 0,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRYPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 1,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRYMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, -1,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRZPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 0,1 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRZMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 0,-1 };
		Rotate(rotationDirection);
		});

	connect(m_Controls.SetTcpToFlangeBtn, &QPushButton::clicked, this, &CzxTest::SetTcpToFlangeBtnClicked);
	connect(m_Controls.RecordInitPosBtn, &QPushButton::clicked, this, &CzxTest::RecordInitPosBtnClicked);
	connect(m_Controls.GoToInitPosBtn, &QPushButton::clicked, this, &CzxTest::GoToInitPosBtnClicked);
	connect(m_Controls.CaptureRobotBtn, &QPushButton::clicked, this, &CzxTest::CaptureRobotBtnClicked);
	connect(m_Controls.RobotAutoRegistationBtn, &QPushButton::clicked, this, &CzxTest::RobotAutoRegistationBtnClicked);
	connect(m_Controls.ResetRobotRegistrationBtn, &QPushButton::clicked, this, &CzxTest::ResetRobotRegistrationBtnClicked);
	connect(m_Controls.SaveRobotRegistrationBtn, &QPushButton::clicked, this, &CzxTest::SaveRobotRegistrationBtnClicked);
	connect(m_Controls.ReuseRobotRegistationBtn, &QPushButton::clicked, this, &CzxTest::ReuseRobotRegistationBtnClicked);
	connect(m_Controls.automoveBtn, &QPushButton::clicked, this, &CzxTest::automoveBtnClicked);
	connect(m_Controls.StopRobotMoveBtn, &QPushButton::clicked, this, &CzxTest::StopRobotMoveBtnClicked);
	connect(m_Controls.ClearRobotErrorInfoBtn, &QPushButton::clicked, this, &CzxTest::ClearRobotErrorInfoBtnClicked);

	connect(m_Controls.ReadRobotJointAnglesBtn, &QPushButton::clicked, this, &CzxTest::ReadRobotJointAnglesBtnClicked);
	connect(m_Controls.SetRobotJointAnglesBtn, &QPushButton::clicked, this, &CzxTest::SetRobotJointAnglesBtnClicked);
	connect(m_Controls.GetRobotJointsLimitBtn, &QPushButton::clicked, this, &CzxTest::GetRobotJointsLimitBtnClicked);

	connect(m_Controls.SetRobotPositionModeBtn, &QPushButton::clicked, this, &CzxTest::SetRobotPositionModeBtnClicked);
	connect(m_Controls.SetRobotJointsImpedanceModelBtn, &QPushButton::clicked, this, &CzxTest::SetRobotJointsImpedanceModelBtnClicked);
	connect(m_Controls.SetRobotCartImpedanceModeBtn, &QPushButton::clicked, this, &CzxTest::SetRobotCartImpedanceModeBtnClicked);
}

void CzxTest::InitTCPRegistrationTabConnection()
{
	connect(m_Controls.GetFourProbeEndPosBtn, &QPushButton::clicked, this, [=]() {
		GetProbeEndPos(TCPRegistraionMethod::FourPoints);
		});
	connect(m_Controls.GetThreeProbeEndPosBtn, &QPushButton::clicked, this, [=]() {
		GetProbeEndPos(TCPRegistraionMethod::ThreePoints);
		});
	connect(m_Controls.SetDrillEndTCPBtn, &QPushButton::clicked, this, &CzxTest::SetDrillEndTCPBtnClicked);
	connect(m_Controls.DisplayCameraToTCPAxesBtn, &QPushButton::clicked, this, &CzxTest::DisplayCameraToTCPAxesBtnClicked);
	connect(m_Controls.SetTCPByProbeTipBtn, &QPushButton::clicked, this, &CzxTest::SetTCPByProbeTipBtnClicked);
	connect(m_Controls.ReadRobotImpedaBtn, &QPushButton::clicked, this, &CzxTest::ReadRobotImpedaBtnClicked);
	connect(m_Controls.SetRobotImpedaBtn, &QPushButton::clicked, this, &CzxTest::SetRobotImpedaBtnClicked);
}

void CzxTest::InitIntraRobotMoveConnection()
{
	connect(m_Controls.RobotMoveToFemurDistalBtn, &QPushButton::clicked, this, &CzxTest::RobotMoveToFemurDistalBtnClicked);
	connect(m_Controls.RobotMoveToFemurPosteriorChamferBtn, &QPushButton::clicked, this, &CzxTest::RobotMoveToFemurPosteriorChamferBtnClicked);
	connect(m_Controls.RobotMoveToFemurPosteriorBtn, &QPushButton::clicked, this, &CzxTest::RobotMoveToFemurPosteriorBtnClicked);
	connect(m_Controls.RobotMoveToTibiaProximalBtn, &QPushButton::clicked, this, &CzxTest::RobotMoveToTibiaProximalBtnClicked);
}

void CzxTest::InitPreplanTabConnection()
{
	connect(m_Controls.PreFemurKneeCenterBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::FemurKneeCenter); });
	connect(m_Controls.PreHipCenterBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::HipCenter); });
	connect(m_Controls.PreLateralFemuralEpicondyleBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::LateralFemuralEpicondyle); });
	connect(m_Controls.PreMedialFemuralEpicondyleBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::MedialFemuralEpicondyle); });
	connect(m_Controls.PreLateralMalleolusBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::LateralMalleolus); });
	connect(m_Controls.PreMedialMalleolusBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::MedialMalleolus); });
	connect(m_Controls.PreTibiaKneeCenterBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::TibiaKneeCenter); });
	connect(m_Controls.PreRotationalLandmarkBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::RotationalLandmark); });
	connect(m_Controls.PreLateralTibialEpicondyleBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::LateralTibialEpicondyle); });
	connect(m_Controls.PreMedialTibialEpicondyleBtn, &QPushButton::clicked, this, [this]() {OnPrePointSelect(PrePointCaptureType::MedialTibialEpicondyle); });
	connect(m_Controls.PreCapturePointBtn, &QPushButton::clicked, this, &CzxTest::PreCapturePointBtnClicked);
	connect(m_Controls.CalculateAxisBtn, &QPushButton::clicked, this, &CzxTest::CalculateAxisBtnClicked);
	connect(m_Controls.UseGizmoBtn, &QPushButton::clicked, this, &CzxTest::UseGizmoBtnClicked);
	connect(m_Controls.BrandComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnBrandComboBoxChanged);
	connect(m_Controls.ProsthesisTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnProsthesisTypeComboxBoxChanged);
	connect(m_Controls.ProsthesisKindComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnProsthesisKindComboBoxChanged);
	connect(m_Controls.ApplyProsthesisBtn, &QPushButton::clicked, this, &CzxTest::ApplyProsthesisBtnClicked);
	connect(m_Controls.CalculateProsBtn, &QPushButton::clicked, this, &CzxTest::CalculateProsBtnClicked);
	connect(m_Controls.ApplyTiltAngleBtn, &QPushButton::clicked, this, &CzxTest::ApplyTiltAngleBtnClicked);
	connect(m_Controls.ApplyRotationAngleBtn, &QPushButton::clicked, this, &CzxTest::ApplyRotationAngleBtnClicked);
	connect(m_Controls.ApplyVaAngleBtn, &QPushButton::clicked, this, &CzxTest::ApplyVaAngleBtnClicked);
	connect(m_Controls.ProudPlusBtn, &QPushButton::clicked, this, [this]() {CzxTest::MoveModelByStepBtnClicked(true); });
	connect(m_Controls.ProudMinusBtn, &QPushButton::clicked, this, [this]() {CzxTest::MoveModelByStepBtnClicked(false); });

	connect(m_Controls.PrePlanViewType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnPrePlanViewTypeChanged);
	connect(m_Controls.ProsUpTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnTranslateModelButtonClicked(Direction::Up);
		});
	connect(m_Controls.ProsDownTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnTranslateModelButtonClicked(Direction::Down);
		});
	connect(m_Controls.ProsRightTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnTranslateModelButtonClicked(Direction::Right);
		});
	connect(m_Controls.ProsLeftTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnPrePlanViewTypeChanged(0);
			OnTranslateModelButtonClicked(Direction::Left);
		});
	connect(m_Controls.PrePlanSelectImageWidget, &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
		this, &CzxTest::OnPrePlanSelectImageWidgetSelectChanged);

	connect(m_Controls.AngleCalculationTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnAngleCalculationTypeComboBoxChanged);
}

void CzxTest::InitIntraPlanTabConnection()
{
	connect(m_Controls.StartIntraBtn, &QPushButton::clicked, this, &CzxTest::StartIntraBtnClicked);
	connect(m_Controls.IntraPlanViewType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CzxTest::OnIntraPlanViewTypeChanged);
	connect(m_Controls.IntraProsUpTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			TranslateIntraPros(Direction::Up);
		});
	connect(m_Controls.IntraProsDownTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			TranslateIntraPros(Direction::Down);
		});
	connect(m_Controls.IntraProsRightTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			TranslateIntraPros(Direction::Right);
		});
	connect(m_Controls.IntraProsLeftTransBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			TranslateIntraPros(Direction::Left);
		});
	connect(m_Controls.IntraClockWiseBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			RotatateIntraPros(true);
		});
	connect(m_Controls.IntraAntiClockWiseBtn, &QPushButton::clicked, this, [this]()
		{
			OnIntraPlanViewTypeChanged(0);
			RotatateIntraPros(false);
		});
	connect(m_Controls.SaveIntraDataBtn, &QPushButton::clicked, this, &CzxTest::SaveIntraDataBtnClicked);
}

void CzxTest::InitModelRegistrationTabConnection()
{
	connect(m_Controls.mitkNodeSelectWidget_landmark_src, &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
		this, &CzxTest::OnMitkNodeSelectWidget_landmark_srcChanged);
	connect(m_Controls.SelectRegistrationPointBtn, &QPushButton::clicked, this, &CzxTest::SelectRegistrationPointBtnClicked);
	connect(m_Controls.StartSelectPointBtn, &QPushButton::clicked, this, &CzxTest::StartSelectPointBtnClicked);

	connect(m_Controls.SaveImageRegistraionResultBtn, &QPushButton::clicked, this, &CzxTest::SaveImageRegistraionResultBtnClicked);
	connect(m_Controls.ReuseImageRegistrationBtn, &QPushButton::clicked, this, &CzxTest::ReuseImageRegistrationBtnClicked);
	connect(m_Controls.RegisterImageBtn, &QPushButton::clicked, this, &CzxTest::RegisterImageBtnClicked);
	connect(m_Controls.UseRegistrationBtn, &QPushButton::clicked, this, &CzxTest::UseRegistrationBtnClicked);

	connect(m_Controls.CutFemurClippedPlaneBtn, &QPushButton::clicked, this, &CzxTest::CutFemurClippedPlaneBtnClicked);
	connect(m_Controls.SelectFemurClippedPlanePointBtn, &QPushButton::clicked, this, &CzxTest::SelectFemurClippedPlanePointBtnClicked);
	connect(m_Controls.CutTibiaClippedPlaneBtn, &QPushButton::clicked, this, &CzxTest::CutTibiaClippedPlaneBtnClicked);
	connect(m_Controls.SelectTibiaClippedPlanePointBtn, &QPushButton::clicked, this, &CzxTest::SelectTibiaClippedPlanePointBtnClicked);
	connect(m_Controls.PKAReUseRegistrationPointsSelectWidget, &QmitkSingleNodeSelectionWidget::CurrentSelectionChanged,
		this, &CzxTest::OnPKAReUseRegistrationPointsSelectWidgetChanged);
}

void CzxTest::InitModelDispalyTabConnection()
{
	connect(m_Controls.DisplayPKADrillBtn, &QPushButton::clicked, this, &CzxTest::DisplayPKADrillBtnClicked);
	connect(m_Controls.DisplayProbeBtn, &QPushButton::clicked, this, &CzxTest::DisplayProbeBtnClicked);
	connect(m_Controls.DisplayBluntProbeBtn, &QPushButton::clicked, this, &CzxTest::DisplayBluntProbeBtnClicked);
	connect(m_Controls.DisplayHansRobotModelBtn, &QPushButton::clicked, this, &CzxTest::DisplayHansRobotModelBtnClicked);
	connect(m_Controls.RotateModelAndSaveBtn, &QPushButton::clicked, this, &CzxTest::RotateModelAndSaveBtnClicked);
	connect(m_Controls.SaveRobotJoint2LinkBtn, &QPushButton::clicked, this, &CzxTest::SaveRobotJoint2LinkBtnClicked);
	connect(m_Controls.DisplayDiania7RobotBtn, &QPushButton::clicked, this, &CzxTest::DisplayDiania7RobotBtnClicked);
	connect(m_Controls.DisplayDianaRobotV2Btn, &QPushButton::clicked, this, &CzxTest::DisplayDianaRobotV2BtnClicked);
	connect(m_Controls.RenderBtn, &QPushButton::clicked, this, &CzxTest::RenderBtnClicked);
}

void CzxTest::InitMakeProsthesisConfigTabConnection()
{
	connect(m_Controls.DrawPlaneBtn, &QPushButton::clicked, this, &CzxTest::DrawPlaneBtnClicked);
	connect(m_Controls.ReadConfigBtn, &QPushButton::clicked, this, &CzxTest::ReadConfigBtnClicked);
	connect(m_Controls.SwitchNodeNameBtn, &QPushButton::clicked, this, &CzxTest::SwitchNodeNameBtnClicked);
	connect(m_Controls.WriteJsonBtn, &QPushButton::clicked, this, &CzxTest::WriteJsonBtnClicked);
	connect(m_Controls.SelectJsonPathBtn, &QPushButton::clicked, this, &CzxTest::SelectJsonPathBtnClicked);
	connect(m_Controls.ReadJsonBtn, &QPushButton::clicked, this, &CzxTest::ReadJsonBtnClicked);
	connect(m_Controls.ChangeModelColorBtn, &QPushButton::clicked, this, &CzxTest::ChangeModelColorBtnClicked);
	connect(m_Controls.WriteTuoJsonBtn, &QPushButton::clicked, this, &CzxTest::WriteTuoJsonBtnClicked);
	connect(m_Controls.GenerativeImplantSecurityBoundaryBtn, &QPushButton::clicked, this, &CzxTest::GenerativeImplantSecurityBoundaryBtnClicked);
	connect(m_Controls.GenerativeTraySecurityBoundaryBtn, &QPushButton::clicked, this, &CzxTest::GenerativeTraySecurityBoundaryBtnClicked);
}

void CzxTest::InitIntraOsteotomyTabConnection()
{
	//intra model translation Plus
	connect(m_Controls.IntraDrillEndTipXplusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = CalculationHelper::GetXAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});
	connect(m_Controls.IntraDrillEndTipYplusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = CalculationHelper::GetYAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});
	connect(m_Controls.IntraDrillEndTipZplusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = CalculationHelper::GetZAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});

	connect(m_Controls.IntraDrillEndTipXminusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = -CalculationHelper::GetXAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});
	connect(m_Controls.IntraDrillEndTipYminusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = -CalculationHelper::GetYAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});
	connect(m_Controls.IntraDrillEndTipZminusBtn, &QPushButton::clicked, this, [this]()
		{
			double length = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();
			auto node = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto matrix = node->GetData()->GetGeometry()->GetVtkMatrix();
			Eigen::Vector3d direction = -CalculationHelper::GetZAxisFromVTKMatrix(matrix);
			PKARenderHelper::TranslateModel(direction.data(), node, length);
			m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
			Drill();
		});

	//intra model rotate Plus
	connect(m_Controls.IntraDrillEndTipRXplusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = CalculationHelper::GetXAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});
	connect(m_Controls.IntraDrillEndTipRYplusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = CalculationHelper::GetYAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});
	connect(m_Controls.IntraDrillEndTipRZplusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = CalculationHelper::GetZAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});

	// intraModelTranslation minus
	connect(m_Controls.IntraDrillEndTipRXminusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = -CalculationHelper::GetXAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});
	connect(m_Controls.IntraDrillEndTipRYminusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = -CalculationHelper::GetYAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});
	connect(m_Controls.IntraDrillEndTipRZminusBtn, &QPushButton::clicked, this, [this]()
		{
			auto modelDataNode = m_Controls.IntraOsteotomyEndDrillTipWidget->GetSelectedNode();
			auto geo = modelDataNode->GetData()->GetGeometry();
			vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
			matrix->DeepCopy(geo->GetVtkMatrix());
			Eigen::Vector3d direction = -CalculationHelper::GetZAxisFromVTKMatrix(matrix);
			double angle = m_Controls.IntraDrillEndMovingStepLineEdit->text().toDouble();

			RotateIntraDrillEndTipAndUpDateAxes(direction, modelDataNode, angle);
		});

	connect(m_Controls.AddDrillEndTipModelBtn, &QPushButton::clicked, this, &CzxTest::AddDrillEndTipModelBtnClicked);
	connect(m_Controls.GoToDistalInitialPosBtn, &QPushButton::clicked, this, &CzxTest::GoToDistalInitialPosBtnClicked);
	connect(m_Controls.GoToPoteriorInitialPosBtn, &QPushButton::clicked, this, &CzxTest::GoToPoteriorInitialPosBtnClicked);
	connect(m_Controls.GoToPosteriorChamferInitialPosBtn, &QPushButton::clicked, this, &CzxTest::GoToPosteriorChamferInitialPosBtnClicked);
	connect(m_Controls.InitialFemurOsteotomyModelBtn, &QPushButton::clicked, this,
		[this]() {InitalOsteotomyModelBtnClicked("DrillEndTip",
			PKAData::m_FemurImplantNodeName.toStdString(), PKAData::m_FemurClippedSurfaceNodeName.toStdString()); });
	connect(m_Controls.InitialTibiaOsteotomyModelBtn, &QPushButton::clicked, this,
		[this]() {InitalOsteotomyModelBtnClicked("DrillEndTip",
			PKAData::m_TibiaTrayNodeName.toStdString(), PKAData::m_TibiaClippedSurfaceNodeName.toStdString()); });

	connect(m_Controls.AddFemurBoneModelInterSectionBtn, &QPushButton::clicked, this, &CzxTest::AddFemurBoneModelInterSectionBtnClicked);
	connect(m_Controls.AddTibiaBoneModelInterSectionBtn, &QPushButton::clicked, this, &CzxTest::AddTibiaBoneModelInterSectionBtnClicked);
	InitIntraRobotMoveConnection();
}

void CzxTest::InitGlobalVariable()
{
	m_PKAHardwareDevice = new PKAKukaVegaHardwareDevice(this->GetDataStorage());
	m_PKADianaAimHardwareDevice = new PKADianaAimHardwareDevice();
	m_AngleCalculationHelper = new AngleCalculationHelper(this->GetDataStorage());

	PKAData::m_SurgicalSide = PKASurgicalSide::Left;
	m_TiltRadioBtnGroup = new QButtonGroup(this);
	m_TiltRadioBtnGroup->addButton(m_Controls.FrontTiltRadioBtn, 0);
	m_TiltRadioBtnGroup->addButton(m_Controls.BackTiltRadioBtn, 1);
	m_ProsRotationRadioBtnGroup = new QButtonGroup(this);
	m_ProsRotationRadioBtnGroup->addButton(m_Controls.InternalRotationRadioBtn, 0);
	m_ProsRotationRadioBtnGroup->addButton(m_Controls.ExternalRotationRadioBtn, 1);
	m_VaRadioBtnGroup = new QButtonGroup(this);
	m_VaRadioBtnGroup->addButton(m_Controls.ValgusRadioBtn, 0);
	m_VaRadioBtnGroup->addButton(m_Controls.VarusRadioBtn, 1);
	m_SelectedRegistrationPoint = mitk::PointSet::New();
	m_ModelRegistration = new ModelRegistration();
	m_ToolDisplayHelper = new ToolDisplayHelper(this->GetDataStorage(), this->GetRenderWindowPart(), m_PKADianaAimHardwareDevice);

	m_IntraProsGroup = new QButtonGroup(this);
	m_IntraProsGroup->addButton(m_Controls.IntraFemurImplantRadioBtn, 0);
	m_IntraProsGroup->addButton(m_Controls.IntraTibiaTrayRadioBtn, 1);
	m_Controls.IntraFemurImplantRadioBtn->setChecked(true);

	m_VerifyPointRadioBtnGroup = new QButtonGroup(this);
	m_VerifyPointRadioBtnGroup->addButton(m_Controls.FemurVerifyPointBtn, 0);
	m_VerifyPointRadioBtnGroup->addButton(m_Controls.TibiaVerifyPointBtn, 1);

	m_BoundingShapeInteractor = mitk::BoundingShapeInteractor::New();

	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint1AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint2AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint3AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint4AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint5AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint6AngleLineEdit);
	m_RobotJointAngleLineEdits.push_back(m_Controls.RobotJoint7AngleLineEdit);
}

void CzxTest::LoadMITKFile(std::string filePath)
{
	mitk::DataStorage::SetOfObjects::ConstPointer data = mitk::IOUtil::Load(filePath, *this->GetDataStorage());
}

void CzxTest::DisplayAuxiliaryModel()
{
	double color[3] = { 0,1.0,1.0 };
	double orientationColor[3] = { 0,0,1 };
	if (m_FemoralImplant)
	{
		PKARenderHelper::DisplayDirection(this->GetDataStorage(), m_FemoralImplant->GetDistalCut(), m_FemoralImplant->GetDistalCutNormal(), "FemurDistalLine", color);

		PKARenderHelper::DisplayDirection(this->GetDataStorage(), m_FemoralImplant->GetFrontOrientationPoint(), m_FemoralImplant->GetBackOrientationPoint(), "FemurOrientationLine", orientationColor);
		PKARenderHelper::AddPointSetToMitk(this->GetDataStorage(), std::vector<Eigen::Vector3d>{m_FemoralImplant->GetProudPoint()}, "FemurProudPoint", 5, color);
	}
	else if (m_TibiaTray)
	{
		PKARenderHelper::DisplayDirection(this->GetDataStorage(), m_TibiaTray->GetProximal(), m_TibiaTray->GetProximalNormal(), "TibiaDistalLine", color);
		PKARenderHelper::DisplayDirection(this->GetDataStorage(), m_TibiaTray->GetFrontOrientationPoint(), m_TibiaTray->GetBackOrientationPoint(), "TibiaOrientationLine", orientationColor);
		PKARenderHelper::AddPointSetToMitk(this->GetDataStorage(), std::vector<Eigen::Vector3d>{m_TibiaTray->GetProudPoint()}, "TibiaProudPoint", 5, color);
	}
}

void CzxTest::TransformAuxiliaryModel(vtkMatrix4x4* matrix)
{
	vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
	m->DeepCopy(matrix);
	PKARenderHelper::TransformModel(this->GetDataStorage(), "DistalLine", m);
	PKARenderHelper::TransformModel(this->GetDataStorage(), "OrientationLine", m);
	PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_FemurImplantNodeName.toStdString().c_str(), m);
	PKARenderHelper::TransformModel(this->GetDataStorage(), "ProudPoint", m);
	PKARenderHelper::TransformModel(this->GetDataStorage(), "ProudLine", m);
}

Eigen::Vector3d CzxTest::GetTranslateMovementByViewType(ViewType viewType, Direction direction)
{
	Eigen::Vector3d moveDirection(0, 0, 0);
	switch (viewType)
	{
	case ViewType::Axial:
		if (direction == Direction::Up || direction == Direction::Down)
			moveDirection[1] = (direction == Direction::Up) ? -1 : 1;
		else
			moveDirection[0] = (direction == Direction::Left) ? -1 : 1;
		break;
	case ViewType::Sagittal:
		if (direction == Direction::Up || direction == Direction::Down)
			moveDirection[2] = (direction == Direction::Up) ? 1 : -1;
		else
			moveDirection[1] = (direction == Direction::Left) ? -1 : 1;
		break;
	case ViewType::Coronal:
		if (direction == Direction::Up || direction == Direction::Down)
			moveDirection[2] = (direction == Direction::Up) ? 1 : -1;
		else
			moveDirection[0] = (direction == Direction::Left) ? -1 : 1;
		break;
	default:
		return moveDirection;
	}
	return moveDirection;
}

Eigen::Vector3d CzxTest::GetRotationDirectionByViewType(ViewType viewType, bool isUp)
{
	Eigen::Vector3d direction(0, 0, 0);
	switch (viewType)
	{
	case ViewType::Axial:
		direction[2] = isUp ? 1 : -1; break;
	case ViewType::Sagittal:
		direction[0] = isUp ? -1 : 1; break;
	case ViewType::Coronal:
		direction[1] = isUp ? 1 : -1; break;
	default:
		return direction;
	}
	return direction;
}

void CzxTest::OnTabChanged(int aIndex)
{
	std::cout << "OnTabChanged: " << aIndex << std::endl;
	if (aIndex == 8)
	{
		std::cout << "Structural IntraOsteotomy Class" << std::endl;
		m_IntraOsteotomy = new IntraOsteotomy(this->GetDataStorage(), m_PKADianaAimHardwareDevice, m_FemoralImplant, m_TibiaTray);
	}
}

void CzxTest::Render3D()
{
	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	renderWindow->ResetView();
}

void CzxTest::DrawPlaneBtnClicked()
{
	planeCount++;
	std::string nodeName = "plane" + std::to_string(planeCount);
	PKARenderHelper::DrawPlane(this->GetDataStorage(), nodeName);
}

void CzxTest::ChangeModelOpacityBtnClicked(const char* modelName, int value)
{
	mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode(modelName);
	double opacity = value / 100.0;
	node->SetOpacity(opacity);
}

void CzxTest::SetLandmarkPointBtnClicked()
{
	if (m_Controls.LandmarkPointSetNameLineEdit->text() == "PointSetName")
	{
		QMessageBox msgBox;
		msgBox.setText(QString::fromLocal8Bit("Warning"));
		msgBox.setInformativeText(QString::fromLocal8Bit("请设置Point Set Name"));
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
		return;
	}
	OnAddPointSetClicked();
}

void CzxTest::PrintNode()
{
	m_Controls.textBrowser->append("xsdadadasdasdasda");
}

void CzxTest::OnAddPointSetClicked()
{
	mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
	mitk::DataNode::Pointer pointSetNode = mitk::DataNode::New();
	pointSetNode->SetData(pointSet);
	pointSetNode->SetProperty("name", mitk::StringProperty::New(m_Controls.LandmarkPointSetNameLineEdit->text().toStdString()));
	pointSetNode->SetProperty("opacity", mitk::FloatProperty::New(1));
	pointSetNode->SetFloatProperty("pointsize", 5.0);
	pointSetNode->SetColor(1.0, 0.0, 0.0);
	this->GetDataStorage()->Add(pointSetNode);

	m_Controls.selectedPointSetWidget->SetCurrentSelectedNode(pointSetNode);
}

void CzxTest::OnCurrentSelectionChanged(QmitkSingleNodeSelectionWidget::NodeList)
{
	if (!m_Controls.selectedPointSetWidget->GetSelectedNode())
		return;
	m_Controls.poinSetListWidget->SetPointSetNode(m_Controls.selectedPointSetWidget->GetSelectedNode());
}

void CzxTest::CalcualtePointSetCenterBtnClicked()
{
	mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode(m_Controls.LandmarkPointSetNameLineEdit->text().toStdString());

	mitk::PointSet::Pointer pointSet = dynamic_cast<mitk::PointSet*>(node->GetData());
	if (pointSet.IsNotNull())
	{
		// 获取点的数量
		mitk::PointSet::PointsIterator it = pointSet->Begin();
		mitk::PointSet::PointsIterator end = pointSet->End();
		Eigen::Vector3d pointSum;
		pointSum.setZero();

		int size = pointSet->GetSize();
		while (it != end)
		{
			mitk::Point3D point = it->Value();
			pointSum[0] += point[0];
			pointSum[1] += point[1];
			pointSum[2] += point[2];
			std::cout << "Point ID: " << it->Index() << " Position: " << point << std::endl;
			++it;
		}
		pointSum = pointSum / size;
		PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, pointSum, "Point Center: ");
		CondyleCenterPos = pointSum;
	}
	else
	{
		std::cerr << "DataNode does not contain a PointSet" << std::endl;
	}
}

void CzxTest::RotateTibiaByCenterBtnClicked()
{
	if (!m_TibiaBoneModel)
	{
		QString str = "build tibia Bone Model First";
		m_Controls.textBrowser->append(str);
		return;
	}
	mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode("tibiaSurface");
	// 获取当前模型的变换矩阵
	mitk::BaseData::Pointer modelData = node->GetData();

	if (!m_TibiaRotateTimer)
	{
		auto femurMatrix = this->GetDataStorage()->GetNamedNode
		(PKAData::m_FemurClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
		auto tibiaMatrix = this->GetDataStorage()->GetNamedNode
		(PKAData::m_TibiaClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
		PrintDataHelper::CoutMatrix("femurMatrix", femurMatrix);
		PrintDataHelper::CoutMatrix("tibiaMatrix", tibiaMatrix);
		m_TibiaRotateTimer = new QTimer(this);
	}
	if (!m_IsRotateTibia)
	{
		connect(m_TibiaRotateTimer, SIGNAL(timeout()), this, SLOT(OnRotateTibia()));
		m_TibiaRotateTimer->start(150);
		m_IsRotateTibia = !m_IsRotateTibia;
	}
	else
	{
		disconnect(m_TibiaRotateTimer, SIGNAL(timeout()), this, SLOT(OnRotateTibia()));
		auto tibiaGeo = this->GetDataStorage()->GetNamedNode
		(PKAData::m_TibiaClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry();
		auto tibiaMatrix = tibiaGeo->GetVtkMatrix();
		PrintDataHelper::CoutMatrix("tibiaMatrix", tibiaMatrix);
		vtkSmartPointer<vtkMatrix4x4> I = vtkSmartPointer<vtkMatrix4x4>::New();
		I->Identity();
		tibiaGeo->SetIndexToWorldTransformByVtkMatrix(I);
		m_IsRotateTibia = !m_IsRotateTibia;
	}
}

void CzxTest::ChangeModelColorBtnClicked()
{
	float color[3] = { m_Controls.ModelColorRLineEdit->text().toFloat(), m_Controls.ModelColorGLineEdit->text().toFloat() , m_Controls.ModelColorBLineEdit->text().toFloat() };
	m_currentSelectedNode->SetColor(color);
}

void CzxTest::DisplayPKADrillBtnClicked()
{
	if (!m_IsDisplayDrill)
	{
		m_ToolDisplayHelper->AddModle(PKAMarker::PKADrill, this->GetRenderWindowPart());
		m_IsDisplayDrill = !m_IsDisplayDrill;
		std::cout << "m_IsDisplayDrill" << m_IsDisplayDrill << std::endl;
	}
	else
	{
		m_ToolDisplayHelper->RemoveModel(PKAMarker::PKADrill);
		m_IsDisplayDrill = !m_IsDisplayDrill;
	}
}

void CzxTest::DisplayProbeBtnClicked()
{
	if (!m_IsDisplayProbe)
	{
		m_ToolDisplayHelper->AddModle(PKAMarker::PKAProbe, this->GetRenderWindowPart());
		m_IsDisplayProbe = !m_IsDisplayProbe;
		std::cout << "m_IsDisplayProbe" << m_IsDisplayProbe << std::endl;
	}
	else
	{
		m_ToolDisplayHelper->RemoveModel(PKAMarker::PKADrill);
		m_IsDisplayProbe = !m_IsDisplayProbe;
	}
}

void CzxTest::DisplayBluntProbeBtnClicked()
{
	if (!m_IsDisplayBluntProbe)
	{
		m_ToolDisplayHelper->AddModle(PKAMarker::PKABluntProbe, this->GetRenderWindowPart());
		m_IsDisplayBluntProbe = !m_IsDisplayBluntProbe;
		std::cout << "m_IsDisplayBluntProbe" << m_IsDisplayBluntProbe << std::endl;
	}
	else
	{
		m_ToolDisplayHelper->RemoveModel(PKAMarker::PKABluntProbe);
		m_IsDisplayBluntProbe = !m_IsDisplayBluntProbe;
	}
}

void CzxTest::OnPrePlanViewTypeChanged(int index)
{
	QString selectedText = m_Controls.PrePlanViewType->currentText();

	if (selectedText == to_string(ViewType::Axial))
	{
		m_PrePlanViewType = ViewType::Axial;
		// 在横断面上的操作
	}
	else if (selectedText == to_string(ViewType::Sagittal))
	{
		m_PrePlanViewType = ViewType::Sagittal;
	}
	else if (selectedText == to_string(ViewType::Coronal))
	{
		m_PrePlanViewType = ViewType::Coronal;
	}
}


void CzxTest::OnTranslateModelButtonClicked(Direction direction)
{
	auto dataNode = m_Controls.PrePlanSelectModelWidget->GetSelectedNode();

	Eigen::Vector3d moveDirection = GetTranslateMovementByViewType(m_PrePlanViewType, direction);

	PKARenderHelper::TranslateModel(moveDirection.data(), dataNode, m_TranslateLength);
}

void CzxTest::OnRotateModelButtonClicked(bool up)
{
	Eigen::Vector3d direction = GetRotationDirectionByViewType(m_PrePlanViewType, up);

	auto modelDataNode = m_Controls.PrePlanSelectModelWidget->GetSelectedNode();
	mitk::Point3D modelCenter = modelDataNode->GetData()->GetGeometry()->GetCenter();
	double center[3] = { modelCenter[0], modelCenter[1], modelCenter[2] };
	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, "center: ", 3, center);
	PKARenderHelper::RotateModel(direction.data(), center, modelDataNode, m_RotateAngle);
}

void CzxTest::OnPrePlanSelectImageWidgetSelectChanged(QmitkSingleNodeSelectionWidget::NodeList)
{
	if (!m_Controls.PrePlanSelectImageWidget->GetSelectedNode())
		return;
	mitk::Image::Pointer image = dynamic_cast<mitk::Image*>(m_Controls.PrePlanSelectImageWidget->GetSelectedNode()->GetData());
	Render3D();
}

void CzxTest::DisplayHansRobotModelBtnClicked()
{
	std::string hansRobotPath = "E:\\PKAModelData\\HansRobotFrame2\\";
	JointPartDescription baseDescription;
	baseDescription.FileName = hansRobotPath + "BaseBack.stl";
	baseDescription.Name = "Base";
	RobotBase* base = new RobotBase(baseDescription);
	JointPartDescription Joint1Description = { "Joint1", hansRobotPath + "Joint1Back.stl", {-180, 180}, {-218, 0, -90, -90} };
	JointPartDescription Joint2Description = { "Joint2", hansRobotPath + "Joint2Back2.stl", {-180, 180}, {115.75, 380, 180, 90} };
	JointPartDescription Joint3Description = { "Joint3", hansRobotPath + "Joint3.stl", {-180, 180}, {115.75, 0, -90, -90} };
	JointPartDescription Joint4Description = { "Joint4", hansRobotPath + "Joint4.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint5Description = { "Joint5", hansRobotPath + "Joint5.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint6Description = { "Joint6", hansRobotPath + "Joint6Back.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	std::vector<JointPartDescription> partDescriptions;
	partDescriptions.push_back(Joint1Description);
	partDescriptions.push_back(Joint2Description);
	partDescriptions.push_back(Joint3Description);
	partDescriptions.push_back(Joint4Description);
	partDescriptions.push_back(Joint5Description);
	partDescriptions.push_back(Joint6Description);
	std::vector<RobotJoint*> joints;
	for (int i = 0; i < partDescriptions.size(); ++i)
	{
		auto& part = partDescriptions[i];
		/*part.DH[2] = CalculationHelper::FromDegreeToRadian(part.DH[2]);
		part.DH[3] = CalculationHelper::FromDegreeToRadian(part.DH[3]);	*/
		RobotJoint* joint = new RobotJoint(part);
		joints.push_back(std::move(joint));
	}

	m_RobotFrame = new RobotFrame(this->GetDataStorage(), joints, base);
	std::string configPath = "E:\\PKAModelData\\HansRobotFrame2\\RobotJointToLink.json";
	m_RobotFrame->AddRobot(configPath);

	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	m_RobotFrame->AddAxis(iRenderWindowPart);
	Render3D();
	InitRobotSimulation();
	auto currentJoints = m_RobotFrame->GetCurrentJointAngles();
	for (int i = 0; i < m_RobotJointsAngleLineEdit.size(); ++i)
	{
		m_RobotJointsAngleLineEdit[i]->setText(QString::number(currentJoints[i]));
	}
}

void CzxTest::DisplayDiania7RobotBtnClicked()
{
	std::string hansRobotPath = "E:\\PKAModelData\\silin-STLNew\\";
	JointPartDescription baseDescription;
	baseDescription.FileName = hansRobotPath + "Base.stl";
	baseDescription.Name = "Base";
	RobotBase* base = new RobotBase(baseDescription);
	JointPartDescription Joint1Description = { "Joint1", hansRobotPath + "J1.stl", {-180, 180}, {-218, 0, -90, -90} };
	JointPartDescription Joint2Description = { "Joint2", hansRobotPath + "J2.stl", {-180, 180}, {115.75, 380, 180, 90} };
	JointPartDescription Joint3Description = { "Joint3", hansRobotPath + "J3.stl", {-180, 180}, {115.75, 0, -90, -90} };
	JointPartDescription Joint4Description = { "Joint4", hansRobotPath + "J4.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint5Description = { "Joint5", hansRobotPath + "J5.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint6Description = { "Joint6", hansRobotPath + "J6.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint7Description = { "Joint7", hansRobotPath + "J7.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	std::vector<JointPartDescription> partDescriptions;
	partDescriptions.push_back(Joint1Description);
	partDescriptions.push_back(Joint2Description);
	partDescriptions.push_back(Joint3Description);
	partDescriptions.push_back(Joint4Description);
	partDescriptions.push_back(Joint5Description);
	partDescriptions.push_back(Joint6Description);
	partDescriptions.push_back(Joint7Description);
	std::vector<RobotJoint*> joints;
	for (int i = 0; i < partDescriptions.size(); ++i)
	{
		auto& part = partDescriptions[i];
		/*part.DH[2] = CalculationHelper::FromDegreeToRadian(part.DH[2]);
		part.DH[3] = CalculationHelper::FromDegreeToRadian(part.DH[3]);	*/
		RobotJoint* joint = new RobotJoint(part);
		joints.push_back(std::move(joint));
	}
	JointPartDescription toolDescription = { "RobotTool", hansRobotPath + "ShortDrill.STL", {-180, 180}, {420,  93.5 , 90,  90} };
	RobotTool* tool = new RobotTool(toolDescription);
	m_RobotFrame = new RobotFrame(this->GetDataStorage(), joints, base, tool);
	std::string configPath = "E:\\PKAModelData\\silin-STLNew\\RobotJointToLink.json";
	m_RobotFrame->AddRobot(configPath);

	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");
	m_RobotFrame->AddAxis(iRenderWindowPart);
	Render3D();
	InitRobotSimulation();
	auto currentJoints = m_RobotFrame->GetCurrentJointAngles();
	for (int i = 0; i < m_RobotJointsAngleLineEdit.size(); ++i)
	{
		m_RobotJointsAngleLineEdit[i]->setText(QString::number(currentJoints[i]));
	}
}

void CzxTest::DisplayDianaRobotV2BtnClicked()
{
	DisplayDiana7RobotWithOfficialDH();
}

void CzxTest::DisplayDiana7RobotWithOfficialDH()
{
	std::string hansRobotPath = "E:\\PKAModelData\\silin-STLNew\\";
	JointPartDescription baseDescription;
	baseDescription.FileName = hansRobotPath + "Base.stl";
	baseDescription.Name = "Base";
	RobotBase* base = new RobotBase(baseDescription);
	double a[7] = { 0.000000, 0.000000, 0.000000, 0.064000, -0.052000, -0.012000, 0.086000 };
	double alpha[7] = { 3.141593, 1.570882, -1.571508, 1.570849, -1.571323, -1.572120, -1.573178 };
	double d[7] = { -0.285000, 0.000000, -0.458000, 0.000000 ,-0.455000 ,-0.000000, -0.116000 };
	double theta[7] = { 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0 };
	JointPartDescription Joint1Description = { "Joint1", hansRobotPath + "J1.stl", {-180, 180}, {-218, 0, -90, -90} };
	JointPartDescription Joint2Description = { "Joint2", hansRobotPath + "J2.stl", {-180, 180}, {115.75, 380, 180, 90} };
	JointPartDescription Joint3Description = { "Joint3", hansRobotPath + "J3.stl", {-180, 180}, {115.75, 0, -90, -90} };
	JointPartDescription Joint4Description = { "Joint4", hansRobotPath + "J4.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint5Description = { "Joint5", hansRobotPath + "J5.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint6Description = { "Joint6", hansRobotPath + "J6.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	JointPartDescription Joint7Description = { "Joint7", hansRobotPath + "J7.stl", {-180, 180}, {420,  93.5 , 90,  90} };
	std::vector<JointPartDescription> partDescriptions;
	partDescriptions.push_back(Joint1Description);
	partDescriptions.push_back(Joint2Description);
	partDescriptions.push_back(Joint3Description);
	partDescriptions.push_back(Joint4Description);
	partDescriptions.push_back(Joint5Description);
	partDescriptions.push_back(Joint6Description);
	partDescriptions.push_back(Joint7Description);

	std::vector<RobotJoint*> joints;
	for (int i = 0; i < partDescriptions.size(); ++i)
	{
		auto& part = partDescriptions[i];

		part.DH[0] = d[i] * 1000;
		part.DH[1] = a[i] * 1000;

		part.DH[2] = alpha[i];//CalculationHelper::FromDegreeToRadian(alpha[i]);
		part.DH[3] = theta[i];//CalculationHelper::FromDegreeToRadian(theta[i]);	
		std::cout << "Joint" << i << " d: " << part.DH[0] << " a: " << part.DH[1] << " alpha: " << part.DH[2] << " theta: " << part.DH[3] << std::endl;
		RobotJoint* joint = new RobotJoint(part);
		joints.push_back(std::move(joint));
	}
	m_RobotFrame2 = new RobotFrame2(this->GetDataStorage(), joints, base);

	m_RobotFrame2->Display(this->GetRenderWindowPart());
}

void CzxTest::GetCurrentNode2DActorBtnClicked()
{
	//m_currentSelectedNode->GetMapper()
}

void CzxTest::RotateModelAndSaveBtnClicked()
{
	std::string hansRobotPath = "E:\\PKAModelData\\HansRobotFrame2\\";
	auto surface = (mitk::Surface*)this->GetDataStorage()->GetNamedNode("Joint1")->GetData();
	auto polyData = surface->GetVtkPolyData();
	vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter->SetInputData(polyData);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->RotateZ(90);
	//transform->RotateX(180);
	transform->Update();
	filter->SetTransform(transform);
	filter->Update();
	vtkSmartPointer<vtkPolyData> transformedPolyData = vtkSmartPointer<vtkPolyData>::New();
	transformedPolyData->DeepCopy(filter->GetOutput());
	FileIO::WritePolyDataAsSTL(transformedPolyData, hansRobotPath + "Joint1Back.stl");
}

void CzxTest::CalculateJointToLinkMatrix(vtkMatrix4x4* TImage2PreJoint, vtkMatrix4x4* TImage2Joint, vtkMatrix4x4* TPreJoint2Joint)
{
	vtkSmartPointer<vtkMatrix4x4> TPreJoint2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->PreMultiply();
	TPreJoint2Image->DeepCopy(TImage2PreJoint);
	TPreJoint2Image->Invert();
	transform->SetMatrix(TPreJoint2Image);
	transform->Concatenate(TImage2Joint);
	transform->Update();
	transform->GetMatrix(TPreJoint2Joint);
}

void CzxTest::SaveRobotJoint2LinkBtnClicked()
{
	if (!m_RobotFrame)
		return;
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	std::string file = FileIO::CombinePath(filename.toStdString(), "RobotJointToLink.json").string();
	m_RobotFrame->SaveJoints2Links(file);
}

void CzxTest::ConnectJointControl(QPushButton* plusBtn, QPushButton* minusBtn, QLineEdit* lineEdit, int jointIndex, RobotFrame* robotFrame)
{
	// Connect the plus button
	connect(plusBtn, &QPushButton::clicked, [robotFrame, lineEdit, jointIndex]() {
		double angle = robotFrame->GetJointAngleByIndex(jointIndex);
		angle += 5;
		robotFrame->UpdateJoints(jointIndex, angle);
		lineEdit->setText(QString::number(angle));
		});

	// Connect the minus button
	connect(minusBtn, &QPushButton::clicked, [robotFrame, lineEdit, jointIndex]() {
		double angle = robotFrame->GetJointAngleByIndex(jointIndex);
		angle -= 5;
		robotFrame->UpdateJoints(jointIndex, angle);
		lineEdit->setText(QString::number(angle));
		});
}

void CzxTest::InitRobotSimulation()
{
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint1AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint2AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint3AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint4AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint5AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint6AngleLineEdit);
	m_RobotJointsAngleLineEdit.push_back(m_Controls.Joint7AngleLineEdit);

	ConnectJointControl(m_Controls.Joint1PlusBtn, m_Controls.Joint1MinusBtn, m_Controls.Joint1AngleLineEdit, 0, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint2PlusBtn, m_Controls.Joint2MinusBtn, m_Controls.Joint2AngleLineEdit, 1, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint3PlusBtn, m_Controls.Joint3MinusBtn, m_Controls.Joint3AngleLineEdit, 2, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint4PlusBtn, m_Controls.Joint4MinusBtn, m_Controls.Joint4AngleLineEdit, 3, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint5PlusBtn, m_Controls.Joint5MinusBtn, m_Controls.Joint5AngleLineEdit, 4, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint6PlusBtn, m_Controls.Joint6MinusBtn, m_Controls.Joint6AngleLineEdit, 5, m_RobotFrame);
	ConnectJointControl(m_Controls.Joint7PlusBtn, m_Controls.Joint7MinusBtn, m_Controls.Joint7AngleLineEdit, 6, m_RobotFrame);
}

void CzxTest::RenderBtnClicked()
{
	auto iRenderWindowPart = GetRenderWindowPart();
	QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow("3d");

	auto vtkRenderWindow = renderWindow->GetVtkRenderWindow();
	vtkRenderWindow->Render();

	auto renderer = vtkRenderWindow->GetRenderers()->GetFirstRenderer();
	renderer->Render();
	std::cout << "RenderBtnClicked" << std::endl;
}

void CzxTest::OnPrePointSelect(PrePointCaptureType t)
{
	QPushButton* button = qobject_cast<QPushButton*>(sender());
	if (button)
	{
		// Change the button's background color
		button->setStyleSheet("background-color: green");
	}
	m_PrePointCaptureType = t;
	auto nodeName = to_string(t);
	m_Controls.textBrowser->append(QString::fromStdString(nodeName));
	if (nodeName == "unknown")
	{
		std::cout << "OnPrePointSelect Error" << std::endl;
		return;
	}
	mitk::DataNode::Pointer dataNode = this->GetDataStorage()->GetNamedNode(nodeName);
	if (!dataNode)
	{
		m_Controls.textBrowser->append("data node is nullptr");
		mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
		dataNode = mitk::DataNode::New();
		dataNode->SetData(pointSet);
		dataNode->SetProperty("name", mitk::StringProperty::New(nodeName));
		dataNode->SetProperty("opacity", mitk::FloatProperty::New(1));
		dataNode->SetFloatProperty("pointsize", 5.0);
		dataNode->SetColor(1.0, 0.0, 0.0);
		this->GetDataStorage()->Add(dataNode);
	}

	m_Controls.poinSetListWidget->SetPointSetNode(dataNode);
	m_Controls.textBrowser->append("add node to pointSetList");
	m_Controls.poinSetListWidget->OnBtnAddPoint(true);
}

void CzxTest::PreCapturePointBtnClicked()
{
	m_Controls.poinSetListWidget->OnBtnAddPoint(false);
	auto nodeName = to_string(m_PrePointCaptureType);
	auto pointset = dynamic_cast<mitk::PointSet*>(this->GetDataStorage()->GetNamedNode(nodeName)->GetData());
	int pointsNumber = pointset->GetSize();
	m_Controls.textBrowser->append("point Number:" + QString::number(pointsNumber));
	switch (m_PrePointCaptureType)
	{
	case PrePointCaptureType::HipCenter:
		m_Controls.HipCenterCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::MedialFemuralEpicondyle:
		m_Controls.MedialFemuralEpicondyleCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::LateralFemuralEpicondyle:
		m_Controls.LateralFemuralEpicondyleCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::FemurKneeCenter:
		m_Controls.FemurKneeCenterCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::MedialMalleolus:
		m_Controls.MedialMalleolusCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::LateralMalleolus:
		m_Controls.LateralMalleolusCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::RotationalLandmark:
		m_Controls.RotationalLandMarkCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::TibiaKneeCenter:
		m_Controls.TibiaKneeCenterCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::LateralTibialEpicondyle:
		m_Controls.LateralTibialEpicondyleCheckBox->setChecked(true);
		break;
	case PrePointCaptureType::MedialTibialEpicondyle:
		m_Controls.MedialTibialEpicondyleCheckBox->setChecked(true);
		break;
	default:
		break;
	}
}

void CzxTest::CalculateAxisBtnClicked()
{
	m_FemurBoneModel = new PKAPrePlanBoneModel(this->GetDataStorage());
	m_FemurBoneModel->BuildFemurModel();
	m_TibiaBoneModel = new PKAPrePlanBoneModel(this->GetDataStorage());
	m_TibiaBoneModel->BuildTibiaModel();
	std::cout << "BuildTibiaModel" << std::endl;
	if (m_FemoralImplant && !m_TibiaTray)
	{
		m_AngleCalculationHelper->BuildFemurCalculator(m_FemurBoneModel, m_FemoralImplant);
		InitFemurImplantPos();
		std::cout << "BuildFemurCalculator" << std::endl;
	}
	else if (m_TibiaTray && !m_FemoralImplant)
	{
		m_AngleCalculationHelper->BuildTibiaCalculator(m_TibiaBoneModel, m_TibiaTray);
		InitTibiaTrayPos();
		std::cout << "BuildTibiaCalculator" << std::endl;
	}
	else
	{
		m_AngleCalculationHelper->BuildFemurCalculator(m_FemurBoneModel, m_FemoralImplant);
		InitFemurImplantPos();
		std::cout << "BuildFemurCalculator2" << std::endl;

		m_AngleCalculationHelper->BuildTibiaCalculator(m_TibiaBoneModel, m_TibiaTray);
		InitTibiaTrayPos();
		std::cout << "BuildTibiaCalculator2" << std::endl;
	}

	if (!isDisplayBoneAxes)
	{
		//PKARenderHelper::AddActor(GetRenderWindowPart(), m_FemurBoneModel->GetBoneAxes());
		//PKARenderHelper::AddActor(GetRenderWindowPart(), m_TibiaBoneModel->GetBoneAxes());
		isDisplayBoneAxes = !isDisplayBoneAxes;
	}
}

void CzxTest::UseGizmoBtnClicked()
{
	auto modelNode = m_Controls.PrePlanSelectModelWidget->GetSelectedNode();
	if (mitk::Gizmo::HasGizmoAttached(modelNode, GetDataStorage()) == 1)
	{
		mitk::Gizmo::RemoveGizmoFromNode(modelNode, GetDataStorage());
	}
	else
	{
		auto gizmoNode = mitk::Gizmo::AddGizmoToNode(modelNode, GetDataStorage());
	}
}

void CzxTest::OnBrandComboBoxShowed()
{
	std::string brand = m_Controls.BrandComboBox->currentText().toStdString();
	std::vector<std::string> files = FileIO::GetListSubdirectories(FileIO::CombinePath(m_ModelPath.string(), brand).string());
	m_Controls.ProsthesisTypeComboBox->clear();
	for (const auto& file : files)
	{
		m_Controls.ProsthesisTypeComboBox->addItem(QString::fromStdString(file));
	}
}

void CzxTest::OnBrandComboBoxChanged()
{
	std::string brand = m_Controls.BrandComboBox->currentText().toStdString();
	auto brandPath = FileIO::CombinePath(m_ModelPath.string(), brand);
	if (!std::filesystem::exists(brandPath))
	{
		std::cout << "cannnot find path" << brandPath << std::endl;
	}
	m_Controls.ProsthesisTypeComboBox->clear();
	std::vector<std::string> fileDirectory = FileIO::GetListSubdirectories(brandPath.string());

	for (std::string dic : fileDirectory)
	{
		std::filesystem::path dicPath = dic;
		m_Controls.ProsthesisTypeComboBox->addItem(QString::fromStdString(dicPath.filename().string()));
	}
}

//选择tuo或者ke
void CzxTest::OnProsthesisTypeComboxBoxChanged()
{
	std::filesystem::path prosPath = FileIO::CombinePath(m_ModelPath.string(), m_Controls.BrandComboBox->currentText().toStdString(), m_Controls.ProsthesisTypeComboBox->currentText().toStdString());
	std::vector<std::string> fileDirectory = FileIO::GetListSubdirectories(prosPath.string());
	m_Controls.ProsthesisKindComboBox->clear();
	for (std::string file : fileDirectory)
	{
		std::filesystem::path dicPath = file;
		m_Controls.ProsthesisKindComboBox->addItem(QString::fromStdString(dicPath.filename().string()));
	}
}


void CzxTest::OnProsthesisKindComboBoxChanged()
{
	m_Controls.ProsthesisSizeComboBox->clear();
	m_Controls.ProsthesisSizeComboBox->setEnabled(false);
}

void CzxTest::ApplyProsthesisBtnClicked()
{
	std::filesystem::path path = FileIO::CombinePath(m_ModelPath.string(), m_Controls.BrandComboBox->currentText().toStdString(), m_Controls.ProsthesisTypeComboBox->currentText().toStdString(),
		m_Controls.ProsthesisKindComboBox->currentText().toStdString(), m_Controls.ProsthesisSizeComboBox->currentText().toStdString());
	m_Controls.textBrowser->append(QString::fromStdString(path.string()));
	std::filesystem::path dicName = path.parent_path().filename();
	std::cout << dicName << std::endl;
	dicName += ".STL";
	std::cout << "ApplyProsthesisBtnClicked: " << dicName << std::endl;
	if (m_Controls.ProsthesisTypeComboBox->currentText() == "ke")
	{
		mitk::DataNode::Pointer previousNode = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString());
		vtkSmartPointer<vtkMatrix4x4> previewNodeTransform = nullptr;
		if (previousNode)
		{
			previewNodeTransform = vtkSmartPointer<vtkMatrix4x4>::New();
			previewNodeTransform->DeepCopy(CalculationHelper::GetNodeTransform(previousNode));
			this->GetDataStorage()->Remove(previousNode);
		}
		std::string stlPath = FileIO::CombinePath(path.string(), dicName.string()).string();
		PKARenderHelper::LoadSingleMitkFile(this->GetDataStorage(), stlPath, PKAData::m_FemurImplantNodeName.toStdString());
		m_FemoralImplant = new ChunLiXGImplant(this->GetDataStorage(), stlPath);

		if (previewNodeTransform)
		{
			PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_FemurImplantNodeName.toStdString().c_str(), previewNodeTransform);
			m_FemoralImplant->UpdateImplant(previewNodeTransform);
		}
	}
	else if (m_Controls.ProsthesisTypeComboBox->currentText() == "tuo")
	{
		mitk::DataNode::Pointer previousNode = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaTrayNodeName.toStdString());
		vtkSmartPointer<vtkMatrix4x4> previewNodeTransform = nullptr;
		/*auto axesActor = PKARenderHelper::GenerateAxesActor();
		PKARenderHelper::AddActor(GetRenderWindowPart(), axesActor);*/
		if (previousNode)
		{
			previewNodeTransform = vtkSmartPointer<vtkMatrix4x4>::New();
			previewNodeTransform->DeepCopy(CalculationHelper::GetNodeTransform(previousNode));
			this->GetDataStorage()->Remove(previousNode);
		}
		std::string stlPath = FileIO::CombinePath(path.string(), dicName.string()).string();
		std::cout << "stlPath: " << stlPath << std::endl;
		PKARenderHelper::LoadSingleMitkFile(this->GetDataStorage(), stlPath, PKAData::m_TibiaTrayNodeName.toStdString());
		m_TibiaTray = new ChunLiTray(this->GetDataStorage(), stlPath);

		if (previewNodeTransform)
		{
			PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_TibiaTrayNodeName.toStdString().c_str(), previewNodeTransform);
			if (PKARenderHelper::IsContainsNamedNode(this->GetDataStorage(), PKAData::m_TibiaInsertNodeName.toStdString()))
			{
				PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_TibiaInsertNodeName.toStdString().c_str(), previewNodeTransform);
			}
			m_TibiaTray->UpdateTray(previewNodeTransform);
		}
	}

	Render3D();
}

void CzxTest::InitFemurImplantPos()
{
	if (!ValidateRequiredNodes(PKAData::m_FemurKneeCenterNodeName.toStdString(), PKAData::m_FemurImplantNodeName.toStdString(),
		PKAData::m_LateralFemuralEpicondyleNodeName.toStdString(), PKAData::m_MedialFemuralEpicondyleNodeName.toStdString()))
		return;

	Eigen::Vector3d femurKneeCenter = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_FemurKneeCenterNodeName.toStdString());
	Eigen::Vector3d femurEpicondyle;
	Eigen::Vector2d center;
	double z = femurKneeCenter[2];

	switch (PKAData::m_MedioLateral)
	{
	case MedioLateral::Lateral:
		femurEpicondyle = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_LateralFemuralEpicondyleNodeName.toStdString());
		break;
	case MedioLateral::Medial:
		femurEpicondyle = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_MedialFemuralEpicondyleNodeName.toStdString());
		break;
	default:
		return;
	}

	// Calculate the center point between knee center and epicondyle
	center = Eigen::Vector2d((femurKneeCenter.x() + femurEpicondyle.x()) / 2,
		(femurKneeCenter.y() + femurEpicondyle.y()) / 2);

	// Construct the femur position vector
	Eigen::Vector3d femurPos(center[0], center[1], z - 10);

	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, femurPos, "Init Implant Pos");

	// Apply transformation
	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->Translate(femurPos.data());
	trans->RotateX(180);
	trans->RotateZ(90);
	//TransformAuxiliaryModel(trans->GetMatrix());
	PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_FemurImplantNodeName.toStdString().c_str(), trans->GetMatrix());
	PrintDataHelper::CoutMatrix("InitFemurImplantPos trans: ", trans->GetMatrix());
	auto implantImagMatrix = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
	PrintDataHelper::CoutMatrix("InitFemurImplantPos implant Matrix: ", implantImagMatrix);

	m_FemoralImplant->UpdateImplant(trans->GetMatrix());
	PrintDataHelper::CoutArray(m_FemoralImplant->GetDistalCut(), "m_FemoralImplant->GetDistalCut()");
}

void CzxTest::InitTibiaTrayPos()
{
	if (!ValidateRequiredNodes(PKAData::m_FemurKneeCenterNodeName.toStdString(), PKAData::m_FemurImplantNodeName.toStdString(),
		PKAData::m_LateralFemuralEpicondyleNodeName.toStdString(), PKAData::m_MedialFemuralEpicondyleNodeName.toStdString()))
		return;

	Eigen::Vector3d tibiaKneeCenter = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_TibiaKneeCenterNodeName.toStdString());
	Eigen::Vector3d tibiaXYPos;
	Eigen::Vector2d center;
	double z = tibiaKneeCenter[2];

	switch (PKAData::m_MedioLateral)
	{
	case MedioLateral::Lateral:
		tibiaXYPos = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_LateralMalleolusNodeName.toStdString());
		break;
	case MedioLateral::Medial:
		tibiaXYPos = PKARenderHelper::GetFirstPointFromDataStorage(this->GetDataStorage(), PKAData::m_MedialMalleolusNodeName.toStdString());
		break;
	default:
		return;
	}

	// Calculate the center point between knee center and epicondyle
	center = Eigen::Vector2d((tibiaKneeCenter.x() + tibiaXYPos.x()) / 2,
		(tibiaKneeCenter.y() + tibiaXYPos.y()) / 2);

	// Construct the femur position vector
	Eigen::Vector3d tibiaTratPos(center[0], center[1], z + 5);

	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, tibiaTratPos, "Init TibiaTray Pos");

	// Apply transformation
	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->Translate(tibiaTratPos.data());
	trans->RotateZ(90);
	PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_TibiaTrayNodeName.toStdString().c_str(), trans->GetMatrix());
	if (PKARenderHelper::IsContainsNamedNode(this->GetDataStorage(), PKAData::m_TibiaInsertNodeName.toStdString()))
	{
		PKARenderHelper::TransformModel(this->GetDataStorage(), PKAData::m_TibiaInsertNodeName.toStdString().c_str(), trans->GetMatrix());
	}
	m_TibiaTray->UpdateTray(trans->GetMatrix());
}

void CzxTest::CalculateProsBtnClicked()
{
	std::cout << "CalculateProsBtnClicked" << std::endl;
	QString nodeName = m_AngleCalculationTypeComboBoxSelectedKneeModel == KneeModel::Femur ? PKAData::m_FemurImplantNodeName : PKAData::m_TibiaTrayNodeName;
	auto dataNode = this->GetDataStorage()->GetNamedNode(nodeName.toStdString());
	if (!dataNode)
	{
		std::cout << "CalculateProsBtnClicked Node is nullptr" << std::endl;
		return;
	}

	auto geometry = dataNode->GetData()->GetGeometry();

	auto trans = geometry->GetVtkTransform();
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser, "Model Trans: ", trans->GetMatrix());
	if (m_FemoralImplant)
	{
		m_FemoralImplant->UpdateImplant(trans->GetMatrix());
		//TransformAuxiliaryModel(trans->GetMatrix());
		auto mechanical = m_FemoralImplant->GetMechanicalAxis();
		PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, mechanical, "Femur mechanical");
	}
	else if (m_TibiaBoneModel)
	{
		m_TibiaTray->UpdateTray(trans->GetMatrix());
		auto mechanical = m_TibiaBoneModel->GetMechanicalAxis();
		PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser, mechanical, "Tibia mechanical");
	}

	UpdateAngle(m_AngleCalculationTypeComboBoxSelectedKneeModel);
	ChangeFemurAngleUIDisplay(m_AngleCalculationTypeComboBoxSelectedKneeModel);
}

void CzxTest::ChangeFemurAngleUIDisplay(KneeModel kneeModel)
{
	if (kneeModel == KneeModel::Femur)
	{
		if (PKAData::FemurTilt == Tilt::BackTilt)
		{
			m_Controls.BackTiltRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.FrontTiltRadioBtn->setChecked(true);
		}
		m_Controls.TiltAngleLineEdit->setText(QString::number(PKAData::m_FemurTiltAngle));

		if (PKAData::FemurVa == Va::Valgus)
		{
			m_Controls.ValgusRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.VarusRadioBtn->setChecked(true);
		}
		m_Controls.ProsVaLineEdit->setText(QString::number(PKAData::m_FemurVaAngle));

		if (PKAData::FemurProsRotation == ProsRotation::ExternalRotation)
		{
			m_Controls.ExternalRotationRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.InternalRotationRadioBtn->setChecked(true);
		}
		m_Controls.ProsRotationLineEdit->setText(QString::number(PKAData::m_FemurProsRotationAngle));

		m_Controls.ProudDistanceLineEdit->setText(QString::number(PKAData::m_FemurProudDistance));
	}
	else
	{
		if (PKAData::TibiaTilt == Tilt::BackTilt)
		{
			m_Controls.BackTiltRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.FrontTiltRadioBtn->setChecked(true);
		}
		m_Controls.TiltAngleLineEdit->setText(QString::number(PKAData::m_TibiaTiltAngle));

		if (PKAData::TibiaVa == Va::Valgus)
		{
			m_Controls.ValgusRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.VarusRadioBtn->setChecked(true);
		}
		m_Controls.ProsVaLineEdit->setText(QString::number(PKAData::m_TibiaVaAngle));

		if (PKAData::FemurProsRotation == ProsRotation::ExternalRotation)
		{
			m_Controls.ExternalRotationRadioBtn->setChecked(true);
		}
		else
		{
			m_Controls.InternalRotationRadioBtn->setChecked(true);
		}
		m_Controls.ProsRotationLineEdit->setText(QString::number(PKAData::m_TibiaProsRotationAngle));

		m_Controls.ProudDistanceLineEdit->setText(QString::number(PKAData::m_TibiaProudDistance));
	}
}

void CzxTest::ApplyTiltAngleBtnClicked()
{
	double angle = m_Controls.TiltAngleLineEdit->text().toDouble();
	Tilt tilt = m_Controls.FrontTiltRadioBtn->isChecked() ? Tilt::FrontTilt : Tilt::BackTilt;

	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	angle = angle - PKAData::m_FemurTiltAngle;
	vtkMatrix4x4* result = m_AngleCalculationHelper->SetTiltAngle(tilt, angle, m_AngleCalculationTypeComboBoxSelectedKneeModel);
	matrix->DeepCopy(result);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser, "ApplyTiltAngleBtnClicked", matrix);
}

void CzxTest::ApplyRotationAngleBtnClicked()
{
	double angle = m_Controls.ProsRotationLineEdit->text().toDouble();
	ProsRotation prosRotation = m_Controls.InternalRotationRadioBtn->isChecked() ? ProsRotation::IntenalRotation : ProsRotation::ExternalRotation;
	angle -= PKAData::m_FemurProsRotationAngle;
	m_AngleCalculationHelper->SetProsRotationAngle(prosRotation, angle, m_AngleCalculationTypeComboBoxSelectedKneeModel);
}

void CzxTest::ApplyVaAngleBtnClicked()
{
	double angle = m_Controls.ProsVaLineEdit->text().toDouble();
	Va va = m_Controls.VarusRadioBtn->isChecked() ? Va::Varus : Va::Valgus;
	angle -= PKAData::m_FemurVaAngle;
	m_AngleCalculationHelper->SetVaAngle(va, angle, m_AngleCalculationTypeComboBoxSelectedKneeModel);
}

void CzxTest::ApplyProudDistanceBtnClicked()
{
	double distance = m_Controls.ProudDistanceLineEdit->text().toDouble();
	Eigen::Vector3d direction;
	if (PKAData::m_FemurProudDistance > 10000.0)
	{
		std::cout << "please reset implant or tray" << std::endl;
		return;
	}
	bool isPlus = distance > PKAData::m_FemurProudDistance;
	double moveMovement = std::abs(distance - PKAData::m_FemurProudDistance);
	moveMovement = isPlus ? moveMovement : moveMovement;
	mitk::DataNode* node;
	if (m_AngleCalculationTypeComboBoxSelectedKneeModel == KneeModel::Femur)
	{
		direction = m_FemoralImplant->GetProudDirection();
		node = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString());
	}
	else
	{
		direction = m_TibiaTray->GetProudDirection();
		node = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaTrayNodeName.toStdString());
	}
	if (!node)
		return;
	PKARenderHelper::TranslateModel(direction.data(), node, moveMovement);
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(node->GetData()->GetGeometry()->GetVtkMatrix());
	if (m_AngleCalculationTypeComboBoxSelectedKneeModel == KneeModel::Femur)
	{
		m_FemoralImplant->UpdateImplant(matrix);
	}
	else
	{
		m_TibiaTray->UpdateTray(matrix);
	}
}

void CzxTest::MoveModelByStepBtnClicked(bool isPlus)
{
	double length = 0.1;
	length = isPlus ? length : -length;

	Eigen::Vector3d direction;
	mitk::DataNode* node;
	if (m_AngleCalculationTypeComboBoxSelectedKneeModel == KneeModel::Femur)
	{
		direction = m_FemoralImplant->GetProudDirection();
		node = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString());
	}
	else
	{
		direction = m_TibiaTray->GetProudDirection();
		node = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaTrayNodeName.toStdString());
	}
	std::cout << "MoveModelByStepBtnClicked length: " << length << std::endl;
	PrintDataHelper::CoutArray(direction, "MoveModelByStepBtnClicked direction");
	if (!node)
		return;
	PKARenderHelper::TranslateModel(direction.data(), node, length);
	PKAData::m_FemurProudDistance += length;
	m_Controls.ProudDistanceLineEdit->setText(QString::number(PKAData::m_FemurProudDistance));
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(node->GetData()->GetGeometry()->GetVtkMatrix());
	if (m_AngleCalculationTypeComboBoxSelectedKneeModel == KneeModel::Femur)
	{
		m_FemoralImplant->UpdateImplant(matrix);
	}
	else
	{
		m_TibiaTray->UpdateTray(matrix);
	}
}

void CzxTest::OnAngleCalculationTypeComboBoxChanged()
{
	m_AngleCalculationTypeComboBoxSelectedKneeModel = m_Controls.AngleCalculationTypeComboBox->currentText() == "Femur" ? KneeModel::Femur : KneeModel::Tibia;
	CalculateProsBtnClicked();
}

void CzxTest::UpdateAngle(KneeModel kneeModel)
{
	if (kneeModel == KneeModel::Femur)
	{
		auto yz = m_AngleCalculationHelper->CalculateTilt(kneeModel);
		PKAData::FemurTilt = yz.first;
		PKAData::m_FemurTiltAngle = yz.second;
		//m_Controls.textBrowser->append("yz: " + QString::fromStdString(to_string(yz.first)) + QString::number(yz.second));
		//std::cout << "CalculateTilt Done" << std::endl;
		auto valgusOrVarus = m_AngleCalculationHelper->CalculateVa(kneeModel);// CalculateAngleInXZPlane(mechanical);
		PKAData::FemurVa = valgusOrVarus.first;
		PKAData::m_FemurVaAngle = valgusOrVarus.second;
		//m_Controls.textBrowser->append("xz : " + QString::fromStdString(to_string(valgusOrVarus.first)) + QString::number(valgusOrVarus.second));
		//std::cout << "CalculateVa Done" << std::endl;
		auto xy = m_AngleCalculationHelper->CalculateRotation(kneeModel);
		PKAData::FemurProsRotation = xy.first;
		PKAData::m_FemurProsRotationAngle = xy.second;
		//m_Controls.textBrowser->append("xy: " + QString::fromStdString(to_string(xy.first)) + QString::number(xy.second));
		//std::cout << "CalculateRotation Done" << std::endl;
		PKAData::m_FemurProudDistance = m_AngleCalculationHelper->CalculateProud(kneeModel);
	}
	else
	{
		auto yz = m_AngleCalculationHelper->CalculateTilt(kneeModel);
		PKAData::TibiaTilt = yz.first;
		PKAData::m_TibiaTiltAngle = yz.second;
		//m_Controls.textBrowser->append("yz: " + QString::fromStdString(to_string(yz.first)) + QString::number(yz.second));
		//std::cout << "CalculateTilt Done" << std::endl;
		auto valgusOrVarus = m_AngleCalculationHelper->CalculateVa(kneeModel);// CalculateAngleInXZPlane(mechanical);
		PKAData::TibiaVa = valgusOrVarus.first;
		PKAData::m_TibiaVaAngle = valgusOrVarus.second;
		//m_Controls.textBrowser->append("xz : " + QString::fromStdString(to_string(valgusOrVarus.first)) + QString::number(valgusOrVarus.second));
		//std::cout << "CalculateVa Done" << std::endl;
		auto xy = m_AngleCalculationHelper->CalculateRotation(kneeModel);
		PKAData::TibiaProsRotation = xy.first;
		PKAData::m_TibiaProsRotationAngle = xy.second;
		//m_Controls.textBrowser->append("xy: " + QString::fromStdString(to_string(xy.first)) + QString::number(xy.second));
		//std::cout << "CalculateRotation Done" << std::endl;
		PKAData::m_TibiaProudDistance = m_AngleCalculationHelper->CalculateProud(kneeModel);
	}
}

QString CzxTest::AdaptBoundingObjectName(const QString& name) const
{
	unsigned int counter = 2;
	QString newName = QString("%1 %2").arg(name).arg(counter);

	while (nullptr != this->GetDataStorage()->GetNode(mitk::NodePredicateFunction::New([&newName](const mitk::DataNode* node)
		{
			return 0 == node->GetName().compare(newName.toStdString());
		})))
	{
		newName = QString("%1 %2").arg(name).arg(++counter);
	}

		return newName;
}

void CzxTest::SelectFemurClippedPlanePointBtnClicked()
{
	m_BoundingBoxInteraction->DisplayBoundingBox("femurSurface", "femurSurfaceBoundingShape");
}

void CzxTest::CutFemurClippedPlaneBtnClicked()
{
	m_BoundingBoxInteraction->CutModel("FemurClippedSurface");
}

void CzxTest::SelectTibiaClippedPlanePointBtnClicked()
{
	m_BoundingBoxInteraction->DisplayBoundingBox("tibiaSurface", "tibiaSurfaceBoundingShape");
}

void CzxTest::CutTibiaClippedPlaneBtnClicked()
{
	m_BoundingBoxInteraction->CutModel("TibiaClippedSurface");
}

void CzxTest::StartIntraBtnClicked()
{
	m_Controls.ImageRegistrationTypeComboBox->setCurrentIndex(0);
	ReuseImageRegistrationBtnClicked();
	m_Controls.ImageRegistrationTypeComboBox->setCurrentIndex(1);
	ReuseImageRegistrationBtnClicked();
	UpdateAngle(KneeModel::Femur);
	UpdateAngle(KneeModel::Tibia);
	UpdateIntraAngleUIDisplay();
	m_ToolDisplayHelper->UsingFemurRegistration();
	//CalculateProsBtnClicked();
	if (!isStartIntra)
	{
		connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateBoneIntraDisplay()));
		m_Controls.StartIntraBtn->setText(QString::fromLocal8Bit("停止术中"));
		isStartIntra = !isStartIntra;
	}
	else
	{
		disconnect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateBoneIntraDisplay()));
		m_Controls.StartIntraBtn->setText(QString::fromLocal8Bit("开始术中"));
		isStartIntra = !isStartIntra;
	}
}

void CzxTest::SaveIntraDataBtnClicked()
{
	//获得图像坐标系下的 Implant 和 Tray Matrix
	auto TImage2Implant = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
	auto TImage2TrayMatrix = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaTrayNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TImage2Implant: ", TImage2Implant);
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TImage2TrayMatrix: ", TImage2TrayMatrix);
	//获得图像坐标系到骨头坐标系的关系
	vtkSmartPointer<vtkMatrix4x4> TTibia2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TFemur2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibia2Image->DeepCopy(this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix());
	TFemur2Image->DeepCopy(this->GetDataStorage()->GetNamedNode(PKAData::m_FemurClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix());
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TImage2Tibia: ", TTibia2Image);
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TImage2Femur: ", TFemur2Image);
	TTibia2Image->Invert();
	TFemur2Image->Invert();
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TTibia2Image: ", TTibia2Image);
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TFemur2Image: ", TFemur2Image);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->PreMultiply();

	vtkSmartPointer<vtkMatrix4x4> TTibia2Tray = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TFemur2Implant = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibia2Tray->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TTibia2Image, TImage2TrayMatrix));

	TFemur2Implant->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TFemur2Image, TImage2Implant));

	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TTibia2Tray: ", TTibia2Tray);
	PrintDataHelper::CoutMatrix("SaveIntraDataBtnClicked TFemur2Implant: ", TFemur2Implant);
	//最后得到  骨头到假体的两个矩阵
	FileIO::SaveMatrix2File(m_TFemur2FemurImplantSavePath, TFemur2Implant);
	FileIO::SaveMatrix2File(m_TTibia2TibiaTraySavePath, TTibia2Tray);
}

void CzxTest::OnIntraPlanViewTypeChanged(int index)
{
	QString selectedText = m_Controls.IntraPlanViewType->currentText();

	if (selectedText == to_string(ViewType::Axial))
	{
		m_IntraPlanViewType = ViewType::Axial;
		// 在横断面上的操作
	}
	else if (selectedText == to_string(ViewType::Sagittal))
	{
		m_IntraPlanViewType = ViewType::Sagittal;
	}
	else if (selectedText == to_string(ViewType::Coronal))
	{
		m_IntraPlanViewType = ViewType::Coronal;
	}
}

void CzxTest::TranslateIntraPros(Direction direction)
{
	Eigen::Vector3d moveDirection = GetTranslateMovementByViewType(m_IntraPlanViewType, direction);

	double length = 5.0;
	double directionLength = sqrt((pow(moveDirection[0], 2) + pow(moveDirection[1], 2) + pow(moveDirection[2], 2)));
	if (directionLength == 0)
		return;
	Eigen::Vector3d movementVector;
	movementVector[0] = length * moveDirection[0] / directionLength;
	movementVector[1] = length * moveDirection[1] / directionLength;
	movementVector[2] = length * moveDirection[2] / directionLength;

	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	if (m_Controls.IntraTibiaTrayRadioBtn->isChecked())
	{
		//假体中保存的是骨头坐标系到假体的位姿关系
		//因此假体移动是骨头坐标系下移动  即TTibiaBone2TibiaTray
		//获得假体的位姿
		vtkSmartPointer<vtkMatrix4x4> TTibiaBone2TibiaTray = vtkSmartPointer<vtkMatrix4x4>::New();
		TTibiaBone2TibiaTray->DeepCopy(m_TibiaTray->GetTrayMatrix());

		trans->SetMatrix(TTibiaBone2TibiaTray);

		trans->Translate(movementVector.data());
		vtkSmartPointer<vtkMatrix4x4> transformedMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		trans->GetMatrix(transformedMatrix);

		m_TibiaTray->UpdateTray(transformedMatrix);
	}
	else
	{
		vtkSmartPointer<vtkMatrix4x4> TFemurBone2FemurImplant = vtkSmartPointer<vtkMatrix4x4>::New();
		TFemurBone2FemurImplant->DeepCopy(m_FemoralImplant->GetImplantMatrix());

		trans->SetMatrix(TFemurBone2FemurImplant);

		trans->Translate(movementVector.data());
		vtkSmartPointer<vtkMatrix4x4> transformedMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		trans->GetMatrix(transformedMatrix);

		m_FemoralImplant->UpdateImplant(transformedMatrix);
		auto implantGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		implantGeo->SetIndexToWorldTransformByVtkMatrix(transformedMatrix);
	}
}

void CzxTest::RotatateIntraPros(bool up)
{
	double angle = 10;
	Eigen::Vector3d direction = GetRotationDirectionByViewType(m_IntraPlanViewType, up);

	double normalizedDirection[3];
	double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
	if (directionLength == 0)
		return;

	normalizedDirection[0] = direction[0] / directionLength;
	normalizedDirection[1] = direction[1] / directionLength;
	normalizedDirection[2] = direction[2] / directionLength;
	PrintDataHelper::CoutArray(normalizedDirection, 3, "RotatateIntraPros::normalizedDirection");
	vtkSmartPointer<vtkMatrix4x4> TBone2Pros = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> transformedMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	if (m_Controls.IntraTibiaTrayRadioBtn->isChecked())
	{
		TBone2Pros->DeepCopy(m_TibiaTray->GetTrayMatrix());
		transform->SetMatrix(TBone2Pros);
		transform->RotateWXYZ(angle, normalizedDirection[0], normalizedDirection[1], normalizedDirection[2]);
		transform->GetMatrix(transformedMatrix);
		m_TibiaTray->UpdateTray(transformedMatrix);
	}
	else
	{
		TBone2Pros->DeepCopy(m_FemoralImplant->GetImplantMatrix());
		transform->SetMatrix(TBone2Pros);
		transform->RotateWXYZ(angle, normalizedDirection[0], normalizedDirection[1], normalizedDirection[2]);
		transform->GetMatrix(transformedMatrix);
		m_FemoralImplant->UpdateImplant(transformedMatrix);
		auto femurImplantGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		femurImplantGeo->SetIndexToWorldTransformByVtkMatrix(transformedMatrix);
	}
}

void CzxTest::IntraVaAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel)
{
	vtkSmartPointer<vtkMatrix4x4> prosTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	double angle = isPlus ? +0.5 : -0.5;
	if (kneeModel == KneeModel::Femur)
	{
		prosTransform = m_AngleCalculationHelper->SetVaAngle(PKAData::FemurVa, angle, kneeModel);
		auto femurProsGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		femurProsGeo->SetIndexToWorldTransformByVtkMatrix(prosTransform);
		m_FemoralImplant->UpdateImplant(prosTransform);
		PKAData::m_FemurVaAngle += angle;
	}
	else
	{
		prosTransform = m_AngleCalculationHelper->SetVaAngle(PKAData::TibiaVa, angle, kneeModel);
		m_TibiaTray->UpdateTray(prosTransform);
		PKAData::m_TibiaVaAngle += angle;
	}
	UpdateIntraAngleUIDisplay();
}

void CzxTest::IntraTiltAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel)
{
	vtkSmartPointer<vtkMatrix4x4> prosTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	double angle = isPlus ? +0.5 : -0.5;;
	if (kneeModel == KneeModel::Femur)
	{
		prosTransform = m_AngleCalculationHelper->SetTiltAngle(PKAData::FemurTilt, angle, kneeModel);
		auto femurProsGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		femurProsGeo->SetIndexToWorldTransformByVtkMatrix(prosTransform);
		m_FemoralImplant->UpdateImplant(prosTransform);
		PKAData::m_FemurTiltAngle += angle;
	}
	else
	{
		prosTransform = m_AngleCalculationHelper->SetTiltAngle(PKAData::TibiaTilt, angle, kneeModel);
		m_TibiaTray->UpdateTray(prosTransform);
		PKAData::m_TibiaTiltAngle += angle;
	}
	UpdateIntraAngleUIDisplay();
}

void CzxTest::IntraRotationAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel)
{
	vtkSmartPointer<vtkMatrix4x4> prosTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	double angle = isPlus ? +0.5 : -0.5;;
	if (kneeModel == KneeModel::Femur)
	{
		prosTransform = m_AngleCalculationHelper->SetProsRotationAngle(PKAData::FemurProsRotation, angle, kneeModel);
		auto femurProsGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		femurProsGeo->SetIndexToWorldTransformByVtkMatrix(prosTransform);
		m_FemoralImplant->UpdateImplant(prosTransform);
		PKAData::m_FemurProsRotationAngle += angle;
	}
	else
	{
		prosTransform = m_AngleCalculationHelper->SetProsRotationAngle(PKAData::TibiaProsRotation, angle, kneeModel);
		m_TibiaTray->UpdateTray(prosTransform);
		PKAData::m_TibiaProsRotationAngle += angle;
	}
	UpdateIntraAngleUIDisplay();
}

void CzxTest::IntraProudChangeBtnClicked(bool isPlus, KneeModel kneeModel)
{
	double length = isPlus ? 0.1 : -0.1;
	std::cout << "IntraProudChangeBtnClicked length: " << length << std::endl;
	Eigen::Vector3d direction;
	mitk::BaseGeometry* geo;
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> prosMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	if (kneeModel == KneeModel::Femur)
	{
		prosMatrix->DeepCopy(m_FemoralImplant->GetImplantMatrix());
		transform->SetMatrix(prosMatrix);
		direction = m_FemoralImplant->GetProudDirection();
		direction.normalize();
		PrintDataHelper::CoutArray(direction, "IntraProudChangeBtnClicked direction");
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		if (directionLength == 0)
			return;
		Eigen::Vector3d movementVector;
		movementVector[0] = length * direction[0] / directionLength;
		movementVector[1] = length * direction[1] / directionLength;
		movementVector[2] = length * direction[2] / directionLength;
		PrintDataHelper::CoutArray(movementVector, "IntraProudChangeBtnClicked movementVector");
		transform->Translate(movementVector.data());

		geo = this->GetDataStorage()->GetNamedNode(PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry();
		geo->SetIndexToWorldTransformByVtkMatrix(transform->GetMatrix());
		m_FemoralImplant->UpdateImplant(transform->GetMatrix());
		PKAData::m_FemurProudDistance += length;
	}
	else
	{
		direction = m_TibiaTray->GetProudDirection();
		direction.normalize();
		prosMatrix->DeepCopy(m_TibiaTray->GetTrayMatrix());
		transform->SetMatrix(prosMatrix);
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		if (directionLength == 0)
			return;
		Eigen::Vector3d movementVector;
		movementVector[0] = length * direction[0] / directionLength;
		movementVector[1] = length * direction[1] / directionLength;
		movementVector[2] = length * direction[2] / directionLength;
		transform->Translate(movementVector.data());

		m_TibiaTray->UpdateTray(transform->GetMatrix());
		PKAData::m_TibiaProudDistance += length;
	}
	UpdateIntraAngleUIDisplay();
}

void CzxTest::UpdateIntraAngleUIDisplay()
{
	m_Controls.IntraFemurVaLabel->setText(QString::fromStdString(to_string(PKAData::FemurVa)));
	m_Controls.IntraFemurTiltLabel->setText(QString::fromStdString(to_string(PKAData::FemurTilt)));
	m_Controls.IntraFemurRotationLabel->setText(QString::fromStdString(to_string(PKAData::FemurProsRotation)));
	m_Controls.IntraFemurVaAngleSpinBox->setValue(PKAData::m_FemurVaAngle);
	m_Controls.IntraFemurTiltAngleSpinBox->setValue(PKAData::m_FemurTiltAngle);
	m_Controls.IntraFemurRotationAngleSpinBox->setValue(PKAData::m_FemurProsRotationAngle);
	m_Controls.IntraFemurProudSpinBox->setValue(PKAData::m_FemurProudDistance);

	m_Controls.IntraTibiaVaLabel->setText(QString::fromStdString(to_string(PKAData::TibiaVa)));
	m_Controls.IntraTibiaTiltLabel->setText(QString::fromStdString(to_string(PKAData::TibiaTilt)));
	m_Controls.IntraTibiaRotationLabel->setText(QString::fromStdString(to_string(PKAData::TibiaProsRotation)));
	m_Controls.IntraTibiaVaAngleSpinBox->setValue(PKAData::m_TibiaVaAngle);
	m_Controls.IntraTibiaTiltAngleSpinBox->setValue(PKAData::m_TibiaTiltAngle);
	m_Controls.IntraTibiaRotationAngleSpinBox->setValue(PKAData::m_TibiaProsRotationAngle);
	m_Controls.IntraTibiaProudSpinBox->setValue(PKAData::m_TibiaProudDistance);
}

void CzxTest::UpdateBoneIntraDisplay()
{
	vtkSmartPointer<vtkMatrix4x4> TFemurImage2TibiaImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TFemurImage2TibiaImage->DeepCopy(m_AngleCalculationHelper->CalculateTFemur2Tibia());

	auto tibiaSurfaceGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaClippedSurfaceNodeName.toStdString())->GetData()->GetGeometry();
	tibiaSurfaceGeo->SetIndexToWorldTransformByVtkMatrix(TFemurImage2TibiaImage);
	vtkSmartPointer<vtkMatrix4x4> tibiaCoordinate = vtkSmartPointer<vtkMatrix4x4>::New();
	tibiaCoordinate->DeepCopy(m_TibiaBoneModel->GetBoneModelCoordinate());
	vtkSmartPointer<vtkMatrix4x4> TfemurImage2TIbiaLocal = vtkSmartPointer<vtkMatrix4x4>::New();
	TfemurImage2TIbiaLocal->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TFemurImage2TibiaImage, tibiaCoordinate));

	m_TibiaBoneModel->UpdateBoneAxesActor(TfemurImage2TIbiaLocal);

	//计算托 
	auto tibiaTrayGeo = this->GetDataStorage()->GetNamedNode(PKAData::m_TibiaTrayNodeName.toStdString())->GetData()->GetGeometry();
	vtkSmartPointer<vtkMatrix4x4> TTibiaImage2TibiaTray = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibiaImage2TibiaTray->DeepCopy(m_TibiaTray->GetTrayMatrix());
	vtkSmartPointer<vtkMatrix4x4> TFemurImage2TibiaTray = vtkSmartPointer<vtkMatrix4x4>::New();

	TFemurImage2TibiaTray->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TFemurImage2TibiaImage, TTibiaImage2TibiaTray));
	tibiaTrayGeo->SetIndexToWorldTransformByVtkMatrix(TFemurImage2TibiaTray);
	auto flexion = m_AngleCalculationHelper->CalculateLimbFlexion();
	PKAData::m_LimbTiltAngle = flexion.second;
	PKAData::m_LimbTilt = flexion.first;
	m_Controls.IntraTiltDisplayLabel->setText(QString::fromStdString(to_string(PKAData::m_LimbTilt)));
	m_Controls.IntraTiltAngleDisplayLabel->setText(QString::number(PKAData::m_LimbTiltAngle));
	auto va = m_AngleCalculationHelper->CalculateLimbVarus();
	PKAData::m_LimbVaAngle = va.second;
	PKAData::m_LimbVa = va.first;
	m_Controls.IntraVaDisplayLabel->setText(QString::fromStdString(to_string(PKAData::m_LimbVa)));
	m_Controls.IntraVaAngleDisplayLabel->setText(QString::number(PKAData::m_LimbVaAngle));
	double distance = m_AngleCalculationHelper->CalculateCutPlaneDistance();
	m_Controls.IntraBoneCutDistanceDispalyLabel->setText(QString::number(distance));
}

void CzxTest::OnBoxModified(vtkObject* caller, long unsigned int evetnId, void* clientData, void* callData)
{
	// Retrieve the box widget from the caller
	vtkBoxWidget2* boxWidget = reinterpret_cast<vtkBoxWidget2*>(caller);

	// Get the transform generated by the box widget
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransform> boxTransform = vtkSmartPointer<vtkTransform>::New();
	vtkBoxRepresentation* boxRep = static_cast<vtkBoxRepresentation*>(boxWidget->GetRepresentation());
	boxRep->GetTransform(boxTransform);

	// Now you can apply the transform to your data or other objects
	// Example: applying the transform to a vtkPolyData object
	vtkPolyData* polyData = static_cast<vtkPolyData*>(clientData);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(polyData);
	transformFilter->SetTransform(boxTransform);
	transformFilter->Update();

	// If you have a renderer, you might want to render the updated scene
	vtkRenderer* renderer = static_cast<vtkRenderer*>(callData);
	if (renderer)
	{
		renderer->Render();
	}
}

void CzxTest::UpdateTCPAxesActor()
{
	m_PKADianaAimHardwareDevice->UpdateTCPAxesActor();
}

void CzxTest::InitalOsteotomyModelBtnClicked(std::string drillEndName, std::string prosNodeName, std::string boneNodeName)
{
	if (!ValidateRequiredNodes(drillEndName, prosNodeName, boneNodeName))
	{
		std::cout << "DrillEndTip, pros, bone is missing" << std::endl;
		return;
	}
	m_IntraOsteotomy->InitalOsteotomyModel(drillEndName, prosNodeName, boneNodeName);
}

void CzxTest::AddModelInterSection(std::string boneNodeName, std::string prosNodeName, std::string intersectionNodeName)
{
	vtkSmartPointer<vtkBooleanOperationPolyDataFilter> filter = vtkSmartPointer<vtkBooleanOperationPolyDataFilter>::New();
	if (!this->GetDataStorage()->GetNamedNode(boneNodeName) || !this->GetDataStorage()->GetNamedNode(prosNodeName))
		return;
	auto BoneSurface = (mitk::Surface*)this->GetDataStorage()->GetNamedNode(boneNodeName)->GetData();
	auto BonePolyData = BoneSurface->GetVtkPolyData();

	auto prosSurface = (mitk::Surface*)this->GetDataStorage()->GetNamedNode(prosNodeName)->GetData();
	auto prosMatrix = this->GetDataStorage()->GetNamedNode(prosNodeName)->GetData()->GetGeometry()->GetVtkMatrix();
	auto prosPolyData = prosSurface->GetVtkPolyData();
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transformFilter->SetInputData(prosPolyData);
	transform->SetMatrix(prosMatrix);
	transformFilter->SetTransform(transform);
	transformFilter->Update();
	vtkSmartPointer<vtkPolyData> transformedProsPolyData = vtkSmartPointer<vtkPolyData>::New();
	transformedProsPolyData->DeepCopy(transformFilter->GetOutput());

	filter->SetOperationToIntersection();
	filter->SetInputData(0, BonePolyData);
	filter->SetInputData(1, transformedProsPolyData);
	filter->Update();

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->DeepCopy(filter->GetOutput());
	if (polyData->GetNumberOfCells() <= 0)
	{
		std::cout << "AddModelInterSection Cells number <=0" << std::endl;
		return;
	}

	mitk::Surface::Pointer interSectionSurface = mitk::Surface::New();
	interSectionSurface->SetVtkPolyData(polyData);

	mitk::DataNode::Pointer interSectionDataNode = mitk::DataNode::New();
	interSectionDataNode->SetData(interSectionSurface);
	interSectionDataNode->SetColor(0, 1, 0);
	interSectionDataNode->SetName(intersectionNodeName);
	this->GetDataStorage()->Add(interSectionDataNode);
}

void CzxTest::AddFemurBoneModelInterSectionBtnClicked()
{
	AddModelInterSection(PKAData::m_FemurClippedSurfaceNodeName.toStdString(),
		PKAData::m_FemurImplantNodeName.toStdString(), "FemurModelIntersectionNode");
}

void CzxTest::AddTibiaBoneModelInterSectionBtnClicked()
{
	AddModelInterSection(PKAData::m_TibiaClippedSurfaceNodeName.toStdString(),
		PKAData::m_TibiaTrayNodeName.toStdString(), "TibiaModelIntersectionNode");
}

void CzxTest::Drill()
{
	if (!ValidateRequiredNodes("green", "buffer", "shell", "red", "DrillEndTip"))
	{
		MITK_WARN << "Cutting initialization is not ready";
		return;
	}

	m_IntraOsteotomy->Drill(m_IntraDrillPlane);
}

void CzxTest::RotateIntraDrillEndTipAndUpDateAxes(Eigen::Vector3d direction, mitk::DataNode* node, double angle)
{
	mitk::Point3D modelCenter = node->GetData()->GetGeometry()->GetCenter();
	double center[3] = { modelCenter[0], modelCenter[1], modelCenter[2] };
	PKARenderHelper::RotateModel(direction.data(), center, node, angle);
	m_DrillEndTipAxesActor->SetUserMatrix(node->GetData()->GetGeometry()->GetVtkMatrix());
}

void CzxTest::AddDrillEndTipModelBtnClicked()
{
	std::string drillEndFilePath = m_DrillEndTipDirectoryPath + "DrillEndTip1.STL";
	PKARenderHelper::LoadSingleMitkFile(this->GetDataStorage(), drillEndFilePath, PKAData::m_DrillEndTipNodeName.toStdString());

	m_DrillEndTipAxesActor = PKARenderHelper::GenerateAxesActor();
	PKARenderHelper::AddActor(GetRenderWindowPart(), m_DrillEndTipAxesActor);
}

void CzxTest::GoToPoteriorInitialPosBtnClicked()
{
	if (!this->GetDataStorage()->GetNamedNode("DrillEndTip"))
	{
		std::cout << "add drillEndTip before click this button" << std::endl;
		return;
	}
	Eigen::Vector3d posteriorPoint = m_FemoralImplant->GetPosteriorCut();
	Eigen::Vector3d posteriorDrection = CalculationHelper::CalculateDirection(posteriorPoint, m_FemoralImplant->GetPosteriorCutNormal());
	PrintDataHelper::CoutArray(posteriorPoint, "posteriorPoint");
	PrintDataHelper::CoutArray(posteriorDrection, "posteriorDrection");
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(m_IntraOsteotomy->CalculateFemurDrillEndTipPos(posteriorPoint, posteriorDrection));
	auto geometry = this->GetDataStorage()->GetNamedNode("DrillEndTip")->GetData()->GetGeometry();
	geometry->SetIndexToWorldTransformByVtkMatrix(matrix);
	m_DrillEndTipAxesActor->SetUserMatrix(matrix);

	std::string cutPath = FileIO::CombinePath(m_InitCutFilePath, m_FemoralImplant->GetPosterioCutFilePath()).string();
	PKARenderHelper::LoadSingleMitkFile(GetDataStorage(), cutPath, "SecurityBoundary");

	auto cutNode = this->GetDataStorage()->GetNamedNode("SecurityBoundary");
	auto implantGeoMatrix = this->GetDataStorage()->GetNamedNode(
		PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();

	cutNode->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(implantGeoMatrix);
	m_IntraDrillPlane = CutPlane::PosteriorCut;
}

void CzxTest::GoToDistalInitialPosBtnClicked()
{
	if (!this->GetDataStorage()->GetNamedNode("DrillEndTip"))
	{
		std::cout << "add drillEndTip before click this button" << std::endl;
		return;
	}
	Eigen::Vector3d distalPoint = m_FemoralImplant->GetDistalCut();
	Eigen::Vector3d distalDirection = m_FemoralImplant->GetMechanicalAxis();
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(m_IntraOsteotomy->CalculateFemurDrillEndTipPos(distalPoint, distalDirection));
	auto geometry = this->GetDataStorage()->GetNamedNode("DrillEndTip")->GetData()->GetGeometry();
	geometry->SetIndexToWorldTransformByVtkMatrix(matrix);
	m_DrillEndTipAxesActor->SetUserMatrix(matrix);

	std::string cutPath = FileIO::CombinePath(m_InitCutFilePath, m_FemoralImplant->GetDistalCutFilePath()).string();
	PKARenderHelper::LoadSingleMitkFile(GetDataStorage(), cutPath, "SecurityBoundary");

	auto cutNode = this->GetDataStorage()->GetNamedNode("SecurityBoundary");
	auto implantGeoMatrix = this->GetDataStorage()->GetNamedNode(
	PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();

	cutNode->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(implantGeoMatrix);
	m_IntraDrillPlane = CutPlane::DistalCut;
}

void CzxTest::GoToPosteriorChamferInitialPosBtnClicked()
{
	if (!this->GetDataStorage()->GetNamedNode("DrillEndTip"))
	{
		std::cout << "add drillEndTip before click this button" << std::endl;
		return;
	}
	Eigen::Vector3d posteriorChamferPoint = m_FemoralImplant->GetPosteriorChamferCut();
	Eigen::Vector3d posteriorChamferDrection = CalculationHelper::CalculateDirection(posteriorChamferPoint, m_FemoralImplant->GetPosteriorChamferCutNormal());

	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(m_IntraOsteotomy->CalculateFemurDrillEndTipPos(posteriorChamferPoint, posteriorChamferDrection));
	auto geometry = this->GetDataStorage()->GetNamedNode("DrillEndTip")->GetData()->GetGeometry();
	geometry->SetIndexToWorldTransformByVtkMatrix(matrix);
	m_DrillEndTipAxesActor->SetUserMatrix(matrix);
	std::string cutPath = FileIO::CombinePath(m_InitCutFilePath, m_FemoralImplant->GetPostriorChamferCutFilePath()).string();
	PKARenderHelper::LoadSingleMitkFile(GetDataStorage(), cutPath, "SecurityBoundary");

	auto cutNode = this->GetDataStorage()->GetNamedNode("SecurityBoundary");
	auto implantGeoMatrix = this->GetDataStorage()->GetNamedNode(
		PKAData::m_FemurImplantNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();

	cutNode->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(implantGeoMatrix);
	m_IntraDrillPlane = CutPlane::PosteriorChamferCut;
}

void CzxTest::GoToTibiaProximalInitialPosBtnClicked()
{
	if (!this->GetDataStorage()->GetNamedNode("DrillEndTip"))
	{
		std::cout << "add drillEndTip before click this button" << std::endl;
		return;
	}
	Eigen::Vector3d proximalPoint = m_TibiaTray->GetProximal();
	Eigen::Vector3d proximalDirection = m_TibiaTray->GetProximalDirection();
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(m_IntraOsteotomy->CalculateFemurDrillEndTipPos(proximalPoint, proximalDirection));
	auto geometry = this->GetDataStorage()->GetNamedNode("DrillEndTip")->GetData()->GetGeometry();
	geometry->SetIndexToWorldTransformByVtkMatrix(matrix);
	m_DrillEndTipAxesActor->SetUserMatrix(matrix);
	std::string cutPath = FileIO::CombinePath(m_InitCutFilePath, m_TibiaTray->GetProximalCutFilePath()).string();
	PKARenderHelper::LoadSingleMitkFile(GetDataStorage(), cutPath, "SecurityBoundary");
	auto cutNode = this->GetDataStorage()->GetNamedNode("SecurityBoundary");
	auto implantGeoMatrix = this->GetDataStorage()->GetNamedNode(
		PKAData::m_TibiaTrayNodeName.toStdString())->GetData()->GetGeometry()->GetVtkMatrix();
	
	cutNode->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(implantGeoMatrix);
	m_IntraDrillPlane = CutPlane::ProximalCut;
}

void CzxTest::RobotMoveToFemurDistalBtnClicked()
{
	//首先获得股骨图像坐标系中的   法向量与点
	auto distalPointInFemur = m_FemoralImplant->GetDistalCut();
	auto distalDirection = CalculationHelper::CalculateDirection(distalPointInFemur, m_FemoralImplant->GetDistalCutNormal());
	PrintDataHelper::CoutArray(distalPointInFemur, "distalPointInFemur: ");
	PrintDataHelper::CoutArray(distalDirection, "distalDirection: ");
	auto matrix = m_IntraOsteotomy->CalculateFemurDrillEndVerticalPlane(distalPointInFemur, distalDirection);

	m_PKADianaAimHardwareDevice->RobotTransformInTCP(matrix->GetData());
}

void CzxTest::RobotMoveToFemurPosteriorChamferBtnClicked()
{
	auto posteriorChamferPoint = m_FemoralImplant->GetPosteriorChamferCut();
	auto posteriorChamferDirection = CalculationHelper::CalculateDirection(posteriorChamferPoint, m_FemoralImplant->GetPosteriorChamferCutNormal());

	auto matrix = m_IntraOsteotomy->CalculateFemurDrillEndVerticalPlane(posteriorChamferPoint, posteriorChamferDirection);

	m_PKADianaAimHardwareDevice->RobotTransformInTCP(matrix->GetData());
}

void CzxTest::RobotMoveToFemurPosteriorBtnClicked()
{
	auto posteriorPoint = m_FemoralImplant->GetPosteriorCut();
	auto posteriorDirection = CalculationHelper::CalculateDirection(posteriorPoint, m_FemoralImplant->GetPosteriorCutNormal());

	auto matrix = m_IntraOsteotomy->CalculateFemurDrillEndVerticalPlane(posteriorPoint, posteriorDirection);

	m_PKADianaAimHardwareDevice->RobotTransformInTCP(matrix->GetData());
}

void CzxTest::RobotMoveToTibiaProximalBtnClicked()
{
	auto proximalPointInTibia = m_TibiaTray->GetProximal();
	auto proximalDirection = m_TibiaTray->GetProximalDirection();

	auto matrix = m_IntraOsteotomy->CalculateTibiaDrillEndVerticalPlane(proximalPointInTibia, proximalDirection);
	m_PKADianaAimHardwareDevice->RobotTransformInTCP(matrix->GetData());
}

void CzxTest::SwitchNodeNameBtnClicked()
{
	std::string nodeNameOne = m_Controls.SwitchNodeNameOneLineEdit->text().toStdString();
	std::string nodeNameTwo = m_Controls.SwitchNodeNameTwoLineEdit->text().toStdString();

	std::string tmp = "tmp";
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	auto dictories = FileIO::GetListSubdirectories(directory);
	mitk::SceneIO::Pointer sceneIO = mitk::SceneIO::New();
	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");

		for (auto file : mitkFiles)
		{
			auto filePath = FileIO::CombinePath(dic, file);
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());
			mitk::DataNode::Pointer nodeOne = this->GetDataStorage()->GetNamedNode(nodeNameOne);

			mitk::DataNode::Pointer nodeTwo = this->GetDataStorage()->GetNamedNode(nodeNameTwo);
			nodeOne->SetName(nodeNameTwo);
			nodeTwo->SetName(nodeNameOne);

			sceneIO->SaveScene(this->GetDataStorage()->GetAll(), this->GetDataStorage(), filePath.string());
		}
	}
}

void CzxTest::ReadConfigBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	auto dictories = FileIO::GetListSubdirectories(directory);
	std::ofstream outFile;
	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");

		for (auto file : mitkFiles)
		{
			auto filePath = FileIO::CombinePath(dic, file);
			std::filesystem::path textFilePath = FileIO::CombinePath(dic, "Config.txt");
			outFile.open(textFilePath, std::ios::trunc);
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());
			std::vector<std::string> nodeNames = { "PosteriorChamferCut" ,"DistalCut" ,"PosteriorCut" };
			std::string normalName = "Normal";
			// Helper function to print and remove points from a node
			auto processNode = [this, &outFile](const std::string& parentName, const std::string& childName) {
				auto parentNode = this->GetDataStorage()->GetNamedNode(parentName);
				if (!parentNode) {
					std::cout << "processNode ParentNode " << parentName << " is null" << std::endl;
					return;
				}

				mitk::DataNode::Pointer childNode = this->GetDataStorage()->GetNamedDerivedNode(childName.c_str(), parentNode);
				if (!childNode) {
					std::cout << "processNode Child node " << childName << " is null" << std::endl;
					return;
				}

				mitk::PointSet::Pointer points = dynamic_cast<mitk::PointSet*>(childNode->GetData());
				if (points.IsNull()) {
					std::cout << parentName << "processNode PointSet is null" << std::endl;
					return;
				}
				auto pointBegin = points->Begin();
				mitk::PointSet::PointType point0 = pointBegin->Value();
				std::cout << parentName << ": " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
				//outFile << parentName << ": " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
				auto pointEnd = std::prev(points->End());
				mitk::PointSet::PointType point1 = pointEnd->Value();
				std::cout << parentName << "Normal: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
				//outFile << parentName << "Normal: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;

				this->GetDataStorage()->Remove(childNode);
			};

			// Process each node in the list
			for (const auto& nodeName : nodeNames) {
				processNode(nodeName, "PointSet");
			}

			// Process the "PointSet" node separately
			for (int i = 0; i < 2; ++i)
			{
				auto node = this->GetDataStorage()->GetNamedNode("PointSet");
				if (!node) {
					std::cout << "single PointSet is null" << std::endl;
					continue;
				}

				mitk::PointSet::Pointer points = dynamic_cast<mitk::PointSet*>(node->GetData());
				if (points.IsNull()) {
					std::cout << "Single PointSet Points is null" << std::endl;
					continue;
				}

				if (points->GetSize() == 2)
				{
					std::cout << "points->GetSize(): " << points->GetSize() << std::endl;
					auto pointBegin = points->Begin();
					mitk::PointSet::PointType point0 = pointBegin->Value();
					std::cout << "orientationFront: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
					//outFile << "orientationFront: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;

					auto pointEnd = std::prev(points->End());
					mitk::PointSet::PointType point1 = pointEnd->Value();
					std::cout << "orientationBack: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
					//outFile << "orientationBack: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
				}
				else
				{
					std::cout << "points->GetSize(): " << points->GetSize() << std::endl;
					auto pointBegin = points->Begin();
					mitk::PointSet::PointType point0 = pointBegin->Value();
					std::cout << "ProudPoint: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
					//outFile << "orientationFront: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;

					auto pointMiddle = points->GetPoint(1);
					mitk::PointSet::PointType point1 = pointMiddle.data();
					std::cout << "smallHole: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
					auto pointEnd = std::prev(points->End());
					mitk::PointSet::PointType point2 = pointEnd->Value();
					std::cout << "bigHole: " << point2[0] << " " << point2[1] << " " << point2[2] << std::endl;
					//outFile << "orientationBack: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
				}
				this->GetDataStorage()->Remove(node);
			}
			outFile.close();
		}
	}
}

void CzxTest::WriteJsonBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	std::filesystem::path fileNamePath = directory;
	std::string folderName = "ChunLi" + fileNamePath.parent_path().filename().string();
	auto dictories = FileIO::GetListSubdirectories(directory);
	std::vector<Prosthesis> prosthesisList;

	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");

		for (auto file : mitkFiles)
		{
			auto filePath = FileIO::CombinePath(dic, file);
			//std::filesystem::path textFilePath = CombinePath(dic, "Config.txt");
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());
			std::vector<std::string> nodeNames = { "PosteriorChamferCut" ,"DistalCut" ,"PosteriorCut" };
			std::string normalName = "Normal";
			Eigen::Vector3d distalCut;
			Eigen::Vector3d distalCutNormal;
			Eigen::Vector3d posteriorCut;
			Eigen::Vector3d posteriorCutNormal;
			Eigen::Vector3d posteriorChamferCutNormal;
			Eigen::Vector3d posteriorChamferCut;
			Eigen::Vector3d orientationFront;
			Eigen::Vector3d orientationBack;
			Eigen::Vector3d proudPoint;
			Eigen::Vector3d smallHolePoint;
			Eigen::Vector3d bigHolePoint;
			std::vector<double> contractPointVector;
			// Helper function to print and remove points from a node
			auto processNode = [this](const std::string& parentName, const std::string& childName, Eigen::Vector3d& p1, Eigen::Vector3d& p2) {
				auto parentNode = this->GetDataStorage()->GetNamedNode(parentName);
				if (!parentNode) {
					std::cout << "ParentNode " << parentName << " is null" << std::endl;
					return;
				}

				mitk::DataNode::Pointer childNode = this->GetDataStorage()->GetNamedDerivedNode(childName.c_str(), parentNode);
				if (!childNode) {
					std::cout << "Child node " << childName << " is null" << std::endl;
					return;
				}

				mitk::PointSet::Pointer points = dynamic_cast<mitk::PointSet*>(childNode->GetData());
				if (points.IsNull()) {
					std::cout << parentName << " PointSet is null" << std::endl;
					return;
				}
				auto pointBegin = points->Begin();
				mitk::PointSet::PointType point0 = pointBegin->Value();
				p1[0] = point0[0];
				p1[1] = point0[1];
				p1[2] = point0[2];
				std::cout << parentName << ": " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
				auto pointEnd = std::prev(points->End());
				mitk::PointSet::PointType point1 = pointEnd->Value();
				std::cout << parentName << "Normal: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
				p2[0] = point1[0];
				p2[1] = point1[1];
				p2[2] = point1[2];

				this->GetDataStorage()->Remove(childNode);
			};

			processNode(nodeNames[0], "PointSet", posteriorChamferCut, posteriorChamferCutNormal);
			processNode(nodeNames[1], "PointSet", distalCut, distalCutNormal);
			processNode(nodeNames[2], "PointSet", posteriorCut, posteriorCutNormal);

			auto contractNode = this->GetDataStorage()->GetNamedNode("ContactPoints");
			if (!contractNode)
			{
				std::cout << "ContactPoints is nullptr" << std::endl;
				return;
			}
			mitk::PointSet::Pointer contractPointSet = dynamic_cast<mitk::PointSet*>(contractNode->GetData());

			for (auto it = contractPointSet->Begin(); it != contractPointSet->End(); ++it)
			{
				mitk::PointSet::PointType point = it->Value();
				contractPointVector.push_back(point[0]);
				contractPointVector.push_back(point[1]);
				contractPointVector.push_back(point[2]);
				std::cout << "ContactPoints" << point[0] << " " << point[1] << " " << point[2] << std::endl;
			}
			// Process the "PointSet" node separately
			for (int i = 0; i < 2; ++i)
			{
				auto node = this->GetDataStorage()->GetNamedNode("PointSet");
				if (!node) {
					std::cout << "single PointSet is null" << std::endl;
					continue;
				}

				mitk::PointSet::Pointer points = dynamic_cast<mitk::PointSet*>(node->GetData());
				if (points.IsNull()) {
					std::cout << "Single PointSet Points is null" << std::endl;
					continue;
				}

				if (points->GetSize() == 2)
				{
					std::cout << "points->GetSize(): " << points->GetSize() << std::endl;
					auto pointBegin = points->Begin();
					mitk::PointSet::PointType point0 = pointBegin->Value();
					std::cout << "orientationFront: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
					orientationFront = Eigen::Vector3d(point0[0], point0[1], point0[2]);

					auto pointEnd = std::prev(points->End());
					mitk::PointSet::PointType point1 = pointEnd->Value();
					std::cout << "orientationBack: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
					orientationBack = Eigen::Vector3d(point1[0], point1[1], point1[2]);
				}
				else
				{
					std::cout << "points->GetSize(): " << points->GetSize() << std::endl;
					auto pointBegin = points->Begin();
					mitk::PointSet::PointType point0 = pointBegin->Value();
					std::cout << "ProudPoint: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
					proudPoint = Eigen::Vector3d(point0[0], point0[1], point0[2]);

					auto pointMiddle = points->GetPoint(1);
					mitk::PointSet::PointType point1 = pointMiddle.data();
					std::cout << "smallHole: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
					smallHolePoint = Eigen::Vector3d(point1[0], point1[1], point1[2]);
					auto pointEnd = std::prev(points->End());
					mitk::PointSet::PointType point2 = pointEnd->Value();
					std::cout << "bigHole: " << point2[0] << " " << point2[1] << " " << point2[2] << std::endl;
					bigHolePoint = Eigen::Vector3d(point2[0], point2[1], point2[2]);
				}
				this->GetDataStorage()->Remove(node);
			}

			Data data;
			std::string number = FileIO::getFileNameWithoutExtension(filePath.string());
			std::string stlPath = number + ".STL";
			data.stl = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), stlPath).string();
			std::string distalPath = FileIO::CombinePath(dic, "DistalCut.stl").string();
			std::string posteriorPath = FileIO::CombinePath(dic, "PosteriorCut.stl").string();
			std::string posteriorChamferPath = FileIO::CombinePath(dic, "PosteriorChamferCut.stl").string();

			if (!std::filesystem::exists(distalPath))
			{
				std::cout << "No " << distalPath << std::endl;
				return;
			}
			if (!std::filesystem::exists(posteriorPath))
			{
				std::cout << "No " << posteriorPath << std::endl;
				return;
			}
			if (!std::filesystem::exists(posteriorChamferPath))
			{
				std::cout << "No " << posteriorChamferPath << std::endl;
				return;
			}
			distalPath = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), "DistalCut.stl").string();
			posteriorPath = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), "PosteriorCut.stl").string();
			posteriorChamferPath = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), "PosteriorChamferCut.stl").string();
			data.PosteriorChamferCut = Cut(posteriorChamferCut, posteriorChamferCutNormal);
			data.DistalCut = Cut(distalCut, distalCutNormal);
			data.PosteriorCut = Cut(posteriorCut, posteriorCutNormal);
			data.OrientationFront = Point(orientationFront);
			data.OrientationBack = Point(orientationBack);
			data.ProudPoint = Point(proudPoint);
			data.SmallHolePoint = Point(smallHolePoint);
			data.BigHolePoint = Point(bigHolePoint);
			data.ContactPoints = contractPointVector;
			data.DistalCutFilePath = distalPath;
			data.PosteriorCutFilePath = posteriorPath;
			data.PosteriorChamferCutFilePath = posteriorChamferPath;

			Prosthesis pros(number, data);
			prosthesisList.push_back(pros);
		}
	}
	Brand brand(prosthesisList);
	nlohmann::json json;
	std::vector<std::pair<std::string, Brand>> pairVec;

	pairVec.push_back({ folderName, brand });
	Type type(pairVec);
	to_json(json, type);
	std::cout << json.dump(4) << std::endl;
	std::filesystem::path outputPath = m_Controls.WriteJsonPahtLineEdit->text().toStdString();
	outputPath /= "config.json";
	std::cout << outputPath << std::endl;
	std::ofstream outputFile(outputPath);
	if (outputFile.is_open())
	{
		outputFile << json.dump(4) << std::endl;
		outputFile.close();
	}
	else
	{
		std::cout << "cannot write json" << std::endl;
	}
}

void CzxTest::ReadJsonBtnClicked()
{
	std::cout << "ReadJsonBtnClicked" << std::endl;
	std::string jsonPath = m_Controls.WriteJsonPahtLineEdit->text().toStdString();
	std::filesystem::path m_jsonPath = FileIO::CombinePath(jsonPath, "config.json");

	FileIO::ReadPKAImplantJson(m_jsonPath);
}

void CzxTest::SelectJsonPathBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	qDebug() << "The selected folder address :" << filename;
	m_Controls.WriteJsonPahtLineEdit->setText(filename);
}

void CzxTest::WriteTuoJsonBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	std::filesystem::path fileNamePath = directory;
	std::string folderName = "ChunLi" + fileNamePath.parent_path().filename().string();
	auto dictories = FileIO::GetListSubdirectories(directory);
	std::vector<TibiaTray> tibiaTrayList;

	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");

		for (auto file : mitkFiles)
		{
			auto filePath = FileIO::CombinePath(dic, file);
			//std::filesystem::path textFilePath = CombinePath(dic, "Config.txt");
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());
			std::string normalName = "Normal";
			Eigen::Vector3d proximal;
			Eigen::Vector3d proximalNormal;
			Eigen::Vector3d orientationFront;
			Eigen::Vector3d orientationBack;

			// Process the "PointSet" node separately
			auto node = this->GetDataStorage()->GetNamedNode("PointSet");
			if (!node) {
				std::cout << "single PointSet is null" << std::endl;
				continue;
			}

			mitk::PointSet::Pointer points = dynamic_cast<mitk::PointSet*>(node->GetData());
			if (points.IsNull()) {
				std::cout << "Single PointSet Points is null" << std::endl;
				continue;
			}

			std::cout << "points->GetSize(): " << points->GetSize() << std::endl;
			auto pointBegin = points->Begin();
			mitk::PointSet::PointType point0 = pointBegin->Value();
			std::cout << "orientationFront: " << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
			orientationFront = Eigen::Vector3d(point0[0], point0[1], point0[2]);

			auto pointEnd = std::prev(points->End());
			mitk::PointSet::PointType point1 = pointEnd->Value();
			std::cout << "orientationBack: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
			orientationBack = Eigen::Vector3d(point1[0], point1[1], point1[2]);
			this->GetDataStorage()->Remove(node);

			TibiaData data;
			std::string number = FileIO::getFileNameWithoutExtension(filePath.string());
			std::string stlPath = number + ".STL";
			data.stl = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), stlPath).string();
			std::string proximalPath = FileIO::CombinePath(dic, "ProximalCut.stl").string();
			if (!std::filesystem::exists(proximalPath))
			{
				std::cout << "No " << proximalPath << std::endl;
				return;
			}

			proximalPath = FileIO::CombinePath(FileIO::GetLastNParts(filePath.parent_path().string(), 4), "ProxiamlCut.stl").string();
			std::regex re(R"(([-+]?\d*\.\d+|\d+)(?!.*\d))");
			std::smatch match;
			std::string lastNumber;
			if (std::regex_search(number, match, re)) {
				lastNumber = match.str(0);
				std::cout << "最后一串数字是: " << lastNumber << std::endl;
			}
			else {
				std::cout << "未找到数字。" << std::endl;
			}

			
			data.OrientationFront = Point(orientationFront);
			data.OrientationBack = Point(orientationBack);
			data.thickness = lastNumber;
			//
			Eigen::Vector3d proximalCut = Eigen::Vector3d(0, 0, 0);
			Eigen::Vector3d proximalCutNormal = Eigen::Vector3d(0, 0, -1);
			data.ProximalCut = Cut(proximalCut, proximalCutNormal);
			data.ProximalCutFilePath = proximalPath;
			TibiaTray pros(number, data);
			tibiaTrayList.push_back(pros);
		}
	}
	TrayBrand brand(tibiaTrayList);
	nlohmann::json json;
	std::vector<std::pair<std::string, TrayBrand>> pairVec;

	pairVec.push_back({ folderName, brand });
	TibiaType type(pairVec);
	to_json(json, type);
	std::cout << json.dump(4) << std::endl;
	std::filesystem::path outputPath = m_Controls.WriteJsonPahtLineEdit->text().toStdString();
	outputPath /= "Trayconfig.json";
	std::cout << outputPath << std::endl;
	std::ofstream outputFile(outputPath);
	if (outputFile.is_open())
	{
		outputFile << json.dump(4) << std::endl;
		outputFile.close();
	}
	else
	{
		std::cout << "cannot write json" << std::endl;
	}
}

void CzxTest::GenerativeImplantSecurityBoundaryBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	std::filesystem::path fileNamePath = directory;
	//std::string folderName = "ChunLi" + fileNamePath.parent_path().filename().string();
	auto dictories = FileIO::GetListSubdirectories(directory);
	std::vector<std::string> cutsName = { "DistalCut", "PosteriorChamferCut" ,"PosteriorCut" };
	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");
		for (auto file : mitkFiles)
		{
			std::cout << "mitk File Name" << file << std::endl;
			auto filePath = FileIO::CombinePath(dic, file);
			//std::filesystem::path textFilePath = CombinePath(dic, "Config.txt");
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());

			//auto  = mitk:

			for (auto cutName : cutsName)
			{
				auto node = this->GetDataStorage()->GetNamedNode(cutName);
				if (!node)
				{
					std::cout << dic << "lack " << cutName << std::endl;
					return;
				}

				auto savePath = FileIO::CombinePath(dic, cutName + ".stl");
				std::cout << "savePath: " << savePath << std::endl;

				FileIO::SaveDataNodeAsSTL(node, savePath.string());
			}
		}
	}
}

void CzxTest::GenerativeTraySecurityBoundaryBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;

	qDebug() << "The selected folder address :" << filename;
	std::string directory = filename.toStdString();
	std::filesystem::path fileNamePath = directory;
	//std::string folderName = "ChunLi" + fileNamePath.parent_path().filename().string();
	auto dictories = FileIO::GetListSubdirectories(directory);
	//std::vector<std::string> cutsName = { "DistalCut", "PosteriorChamferCut" ,"PosteriorCut" };
	
	for (auto dic : dictories)
	{
		auto mitkFiles = FileIO::GetPathFilesWithFileType(dic, ".mitk");
		for (auto file : mitkFiles)
		{
			std::cout << "mitk File Name" << file << std::endl;
			auto filePath = FileIO::CombinePath(dic, file);
			PKARenderHelper::RemoveAllNode(this->GetDataStorage());
			LoadMITKFile(filePath.string());

			std::filesystem::path cutName(file);
			cutName = cutName.stem();
			std::string cutNameStr = cutName.string();
			std::string cutNameStr1 = cutNameStr.substr(0, cutNameStr.size()-3) + "-02";
			std::string cutNameStr2 = cutNameStr.substr(0, cutNameStr.size() - 3) + "02";
			auto node1 = this->GetDataStorage()->GetNamedNode(cutNameStr1);
			auto node2 = this->GetDataStorage()->GetNamedNode(cutNameStr2);
			if (!node1 && !node2)
			{
				std::cout << "cannot find cut!!!!!!!!!!" << std::endl;
				std::cout << cutNameStr1 << std::endl;
				std::cout << cutNameStr2 << std::endl;
				return;
			}

			auto savePath = FileIO::CombinePath(dic, "ProximalCut.stl");
			std::cout << "savePath: " << savePath << std::endl;
			if (node1)
			{
				FileIO::SaveDataNodeAsSTL(node1, savePath.string());
			}
			else
			{
				FileIO::SaveDataNodeAsSTL(node2, savePath.string());
			}
		}
	}
}

void CzxTest::VerifyProbeBtn()
{
	m_PreoPreparation->VerifyProbe(m_Controls.VerifyProbeBtn, m_Controls.ProbeProgressBar, m_Controls.ProbeVerifyRMSLabel);
}

void CzxTest::OnRotateTibia()
{
	mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode("tibiaSurface");
	Eigen::Vector3d condyleCenter = (PKAData::m_MedialTibialEpicondyle + PKAData::m_LateralTibialEpicondyle) / 2;
	double direction[3] = { 1.0, 0.0,0.0 };
	PKARenderHelper::RotateModel(direction, condyleCenter.data(), node, 10);
	//PKARenderHelper::RotateModel(direction, CondyleCenterPos.data(), node, m_RotateAngle);
}

void CzxTest::RenderViewAndWidget()
{
	if (m_PKAHardwareDevice)
	{
		m_PKAHardwareDevice->UpdateHardware();
	}
	m_Controls.m_StatusWidgetVegaToolToShow->Refresh();
	m_Controls.m_StatusWidgetKukaToolToShow->Refresh();
	this->RequestRenderWindowUpdate();
	//std::cout<<"Render" << std::endl;
}