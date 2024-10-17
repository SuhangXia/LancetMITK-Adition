#pragma once
#ifndef AIMCAMERA_h
#define AIMCAMERA_h
#include "MitkLancetHardwareDeviceExports.h"
#include "AbstractCamera.h"

#include "AimPositionAPI.h"

class MITKLANCETHARDWAREDEVICE_EXPORT AimCamera : public AbstractCamera
{
	Q_OBJECT
public:
	void Connect() override;
	void Disconnect() override;
	void Start() override;
	void Stop() override;
	Eigen::Vector3d GetToolTipByName(std::string aToolName) override;

	vtkSmartPointer<vtkMatrix4x4> GetToolMatrixByName(std::string aToolName) override;

	void InitToolsName(std::vector<std::string> aToolsName) override;

private:
	T_AimToolDataResult* GetNewToolData();
	bool UpdateCameraToToolMatrix(T_AimToolDataResult* aToolData, const std::string aName);

protected slots:
	void UpdateData() override;
private:
	AimHandle m_AimHandle = NULL;
	E_Interface m_EI;
	T_MarkerInfo markerSt;
	T_AimPosStatusInfo statusSt;
	E_ReturnValue rlt;
	
};
#endif
