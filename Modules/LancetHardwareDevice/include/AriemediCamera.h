#pragma once
#include "AbstractCamera.h"
#include <MitkLancetHardwareDeviceExports.h>
#include <ARMDCombinedAPI.h>
#include <DeviceScan.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
class MITKLANCETHARDWAREDEVICE_EXPORT AriemediCamera : public AbstractCamera
{
	Q_OBJECT
public:
	AriemediCamera();
	void Connect() override;

	void Disconnect() override;

	void Start() override;

	void Stop() override;

	Eigen::Vector3d GetToolTipByName(std::string aToolName) override;

	vtkSmartPointer<vtkMatrix4x4> GetToolMatrixByName(std::string aToolName) override;

	void InitToolsName(std::vector<std::string> aToolsName) override;

	void DisplayArea(short width, short height, short leftX, short leftY, short rightX, short
		rightY);
	
	std::pair<const char*, const char*> GetImageData();
	void HideArea();
	std::pair<double, double> GetImageSize();

	void ConvertRomToARom(std::string out, std::string in);
	//vtkSmartPointer<vtkPolyData> Get
public slots:
	void UpdateData() override;

private:
	void UpdateImageData(/*std::unique_ptr<char[]>& aLeftImage, std::unique_ptr<char[]>& aRightImage*/);

	bool UpdateCameraToToolMatrix(ToolTrackingData aToolTrackingData);
private:
	ARMDCombinedAPI m_Tracker;
	std::shared_ptr<char[]> m_LeftImage;
	std::shared_ptr<char[]> m_RightImage;
};

