#pragma once
#include "AbstractRobot.h"
#include <robotapi_Staubli.h>
class LancetStaubilRobot : public AbstractRobot
{
	// this robot is using angle not radius
	virtual void Connect() override;
	virtual void Disconnect() override;
	virtual void PowerOn() override;
	virtual void PowerOff() override;

	virtual void Translate(double x, double y, double z) override;
	virtual void Translate(double* aDirection, double aLength) override;
	virtual void Rotate(double* aDirection, double aAngle) override;

	virtual void RecordInitialPos() override;
	virtual void GoToInitialPos() override;
	virtual void SetTCPToFlange() override;
	virtual void SetTCP(vtkMatrix4x4* aMatrix, std::string TCP_NAME = "") override;

	virtual std::vector<double> GetJointAngles() override;
	virtual void SetJointAngles(std::vector<double> aJointAngles) override;

	virtual vtkSmartPointer<vtkMatrix4x4> GetBaseToTCP() override;
	virtual vtkSmartPointer<vtkMatrix4x4> GetFlangeToTCP() override;
	virtual vtkSmartPointer<vtkMatrix4x4> GetBaseToFlange() override;

	virtual void RobotTransformInBase(double* aMatrix) override;
	virtual void RobotTransformInTCP(double* aMatrix) override;

	virtual std::vector<double> GetCartDampParams() override;
	virtual bool SetCartDampParams(std::vector<double> aDampParams) override;
	virtual std::vector<double> GetCartStiffParams() override;
	virtual bool SetCartStiffParams(std::vector<double> aStiffParams) override;

	virtual bool SetVelocity(double aVelocity) override;

	virtual std::vector<std::vector<double>> GetJointAngleLimits() override;

	virtual void WaitMove() override;

private: 
	Eigen::Vector3d GetXYZEulerByMatrix(vtkMatrix4x4* aMatrix);
	Eigen::Vector3d GetTranslationByMatrix(vtkMatrix4x4* aMatrix);
	std::pair<Eigen::Vector3d, Eigen::Vector3d> GetEulerAndTranslationByMatrix(vtkMatrix4x4* aMatrix);
	vtkSmartPointer<vtkMatrix4x4> GetMatrixByEulerAndTranslation(Eigen::Vector3d euler, Eigen::Vector3d trans);
private:
	RobotApi m_Robot;
};

