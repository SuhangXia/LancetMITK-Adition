#include "RobotArmRegistrationTab.h"/// <summary>

/// Robots the arm registration tab.
/// </summary>
/// <param name="ui">The UI.</param>
/// <param name="aDataStorage">a data storage.</param>
/// <param name="aRobot">a robot.</param>
/// <param name="aCamera">a camera.</param>
/// <param name="parent">The parent.</param>
/// <returns></returns>
RobotArmRegistrationTab::RobotArmRegistrationTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent)
{
	m_ui = ui;
	m_Robot = dynamic_cast<LancetJakaRobot*>(aRobot);
	m_Camera = aCamera;
	m_dataStorage = aDataStorage;
	if (!m_dataStorage)
	{
		std::cout << "m_dataStorage is nullptr" << std::endl;
	}
	//ÒÆ¶¯
	m_LancetRobotRegistration = new LancetRobotRegistration(aRobot, aCamera);

	connect(m_ui.pushButton_xp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->xp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_yp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->yp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_zp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->zp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_xm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->xm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_ym_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->ym();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_zm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->zm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rxp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rxp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_ryp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->ryp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rzp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rzp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rxm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rxm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rym_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rym();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rzm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rzm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	//Åä×¼
	connect(m_ui.pushButton_setTCPToFlange_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->setTCPToFlange();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_setInitialPoint_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->recordInitialPos();
		m_ui.textBrowser_hans->append(QString("RecordInitialPos"));
		});
	connect(m_ui.pushButton_goToInitial_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->goToInitialPos();
		m_ui.textBrowser_hans->append(QString("GoToInitialPos"));
		});
	connect(m_ui.pushButton_captureRobot_2, &QPushButton::clicked, this, [this]() {
		m_ui.textBrowser_hans->append(QString("captureRobot"));
		m_LancetRobotRegistration->captureRobot();
		});
	connect(m_ui.pushButton_replaceRegistration_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->replaceRegistration();
		m_ui.textBrowser_hans->append(QString("Replace Registration"));
		m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(0));
		});
	connect(m_ui.pushButton_saveArmMatrix_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->saveArmMatrix();
		m_ui.textBrowser_hans->append(QString("Save ArmMatrix"));
		});
	connect(m_ui.pushButton_reuseArmMatrix_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->reuseArmMatrix();
		m_ui.textBrowser_hans->append(QString("Reuse ArmMatrix"));
		m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(10));
		});
	connect(m_ui.pushButton_AutoMoveJ_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->autoCollection();
		m_ui.textBrowser_hans->append(QString("Auto Collection"));
		});
	connect(m_LancetRobotRegistration, &LancetRobotRegistration::countPose,
		this, &RobotArmRegistrationTab::updateUiCaputreCount);
}

void RobotArmRegistrationTab::updateUiCaputreCount(int cnt)
{ 
	m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(cnt));
}





