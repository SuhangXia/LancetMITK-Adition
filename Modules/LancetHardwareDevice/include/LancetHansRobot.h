#pragma once
#ifndef LANCETHANSROBOT_h
#define LANCETHANSROBOT_h

#include "AbstractRobot.h"
#include <HR_Pro.h>
#include <vtkMath.h>
#include "MitkLancetHardwareDeviceExports.h"
#include "PrintDataHelper.h"
#include <Qtimer.h>
#include <qdatetime.h>

class MITKLANCETHARDWAREDEVICE_EXPORT LancetHansRobot : public AbstractRobot
{
	//Q_OBJECT
public:
	LancetHansRobot();
	void Connect() override;

	void Disconnect() override;

	void Stop() override;

	void PowerOn() override;

	void PowerOff() override;

	void Reset()  override;
	void SetFreeDrag()  override;
	void StopFreeDrag()  override;
	void SetToolMotion() ;
	void SetBaseMotion() ;
	void SetForceFreeDrive() ;
	void StopForceFreeDrive() ;
	void SetOverride(double PercentageofSpeed);

	void Translate(double x, double y, double z) override;

	void Translate(double* aDirection, double aLength) override;

	void Rotate(double* aDirection, double aAngle) override;

	void RecordInitialPos() override;

	void GoToInitialPos() override;

	void SetTCPToFlange() override;

	void SetTCP(vtkMatrix4x4* aMatrix, std::string TCP_NAME = "") override;

	void ConfigTCP(vtkMatrix4x4* aMatrix, std::string TCP_NAME);

	std::vector<double> GetJointAngles() override;

	void SetJointAngles(std::vector<double> aJointAngles) override;

	vtkSmartPointer<vtkMatrix4x4> GetBaseToTCP() override;

	vtkSmartPointer<vtkMatrix4x4> GetFlangeToTCP() override;

	vtkSmartPointer<vtkMatrix4x4> GetBaseToFlange() override;

	std::vector<double> CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle) override;
	std::vector<double> CalculateForward(std::vector<double> aJointAngles) override;

	void RobotTransformInBase(double* aMatrix) override;

	void RobotTransformInTCP(double* aMatrix) override;

	std::vector<double> GetCartStiffParams() override;

	bool SetCartStiffParams(std::vector<double> aStiff) override;

	std::vector<double> GetCartDampParams() override;

	bool SetCartDampParams(std::vector<double> aDamp) override;

	std::vector<std::vector<double>> GetJointAngleLimits() override;

	bool SetVelocity(double aVelocity) override;

	bool SetAcceleration(double aVelocity) override;
	bool SetRadius(double aVelocity) override;

	void WaitMove() override;

	void Sleep(int msec) override;

	inline std::string to_string(int error_code) override;
  
private:
	Eigen::Matrix3d GetRotationMatrixByEuler(double rx, double ry, double rz);
	vtkSmartPointer<vtkMatrix4x4> GetMatrixByRotationAndTranslation(Eigen::Matrix3d aRotation, Eigen::Vector3d aTranslation);
	Eigen::Vector3d GetEulerByMatrix(vtkMatrix4x4* m);

	Eigen::Matrix3d GetRotationPartByMatrix(vtkMatrix4x4* m);
	Eigen::Vector3d GetTranslationPartByMatrix(vtkMatrix4x4* m);
	Eigen::Vector3d CalculateZYXEulerByRotation(Eigen::Matrix3d m);

private:
	vtkSmartPointer<vtkMatrix4x4> m_InitialPos;
private:
	// 定义工具坐标变量
	std::string sTcpName = "TCP";
	// 定义用户坐标变量
	std::string sUcsName = "Base";
	// 定义运动速度
	double dVelocity = 30;
	// 定义运动加速度
	double dAcc = 50;
	// 定义过渡半径
	double dRadius = 50;
	// 定义是否使用关节角度
	int nIsUseJoint = 1;
	// 定义是否使用检测 DI 停止
	int nIsSeek = 0;
	// 定义检测的 DI 索引
	int nIOBit = 0;
	// 定义检测的 DI 状态
	int nIOState = 0;
	// 定义路点 ID 
	string strCmdID = "0";

	int nMoveType = 0;

	int ToolMotion = 0;

	const unsigned int boxID = 0;
	const unsigned int rbtID = 0;
};

#endif