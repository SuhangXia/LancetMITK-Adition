#pragma once
#include "AbstractCamera.h"
#include <MitkLancetHardwareDeviceExports.h>
#include <ARMDCombinedAPI.h>
#include <DeviceScan.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <qthread.h>
#include <mitkDataNode.h>
#include <qfuture.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <ReconstructPointCloud.h>
#include <mitkPointSet.h>
class MITKLANCETHARDWAREDEVICE_EXPORT AriemediCamera : public AbstractCamera
{
	Q_OBJECT
public:
	AriemediCamera();
	~AriemediCamera();
	void Connect() override;

	void Disconnect() override;

	void Start() override;

	void Stop() override;

	Eigen::Vector3d GetToolTipByName(std::string aToolName) override;

	vtkSmartPointer<vtkMatrix4x4> GetToolMatrixByName(std::string aToolName) override;

	void InitToolsName(std::vector<std::string> aToolsName) override;

	void DisplayArea(short width, short height, short leftX, short leftY, short rightX, short
		rightY);
	
	std::pair<char*, char*> GetImageData();
	void HideArea();
	std::pair<double, double> GetImageSize();

	void ConvertRomToARom(std::string out, std::string in);

	void SetAreaDisplay(short leftX, short leftY, short rightX, short rightY, short width = 50, short height = 40);

	char* GetRightImage();

	mitk::DataNode::Pointer GetPointCloud();

public slots:
	void UpdateData() override;

signals:
	void ImageUpdateClock(char* leftImage, char* rightImage, double row,double col);

protected:
	void run() override;

private:
	void UpdateImageData(/*std::unique_ptr<char[]>& aLeftImage, std::unique_ptr<char[]>& aRightImage*/);

	bool UpdateCameraToToolMatrix(ToolTrackingData aToolTrackingData);

	mitk::DataNode::Pointer GetCameraPointCloudDataNode(std::vector<std::vector<float>> aPointCloud);

	void RequestUpdateImage();
	void RequestUpdateTracking();
private:
	std::shared_ptr<char[]> m_LeftImage;
	std::shared_ptr<char[]> m_RightImage;
	size_t m_PreviousImageSize = 0;
	QTimer* m_ImageUpdateTimer;
	ARMDCombinedAPI* m_Tracker;
	bool m_IsStart = false;

	bool m_IsGeneratePointCloud = false;

	std::vector<std::vector<float>> m_PointCloud;
};

