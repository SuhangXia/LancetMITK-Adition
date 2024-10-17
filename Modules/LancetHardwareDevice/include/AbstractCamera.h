#pragma once
#ifndef ABSTRACTCAMERA_h
#define ABSTRACTCAMERA_h

#include <QFileDialog>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <map>
#include <vector>
#include <string>
#include <qtimer.h>
#include <qlabel.h>
#include <eigen3/Eigen/Dense>
#include <QObject>
#include "MitkLancetHardwareDeviceExports.h"
class MITKLANCETHARDWAREDEVICE_EXPORT AbstractCamera : public QObject
{
	Q_OBJECT
public:
	AbstractCamera() : m_CameraUpdateTimer(new QTimer())
	{
	}
	virtual void Connect() = 0;
	virtual void Disconnect() = 0;
	virtual void Start() = 0;

	virtual void Stop() = 0;

	virtual Eigen::Vector3d GetToolTipByName(std::string aToolName) = 0;

	virtual vtkSmartPointer<vtkMatrix4x4> GetToolMatrixByName(std::string aToolName) = 0;

	virtual void InitToolsName(std::vector<std::string> aToolsName) = 0;

	void ResetTools()
	{
		m_ToolMatrixMap.clear();
		m_ToolTipMap.clear();
		m_ToolLabelMap.clear();
	}
public slots:
	virtual void UpdateData() = 0;
signals:
	void CameraUpdateClock();
protected:
	vtkSmartPointer<vtkMatrix4x4> GetMatrixByRotationAndTranslation(Eigen::Matrix3d rotation, Eigen::Vector3d translation)
	{
		vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
		matrix->Identity();
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
			{
				matrix->SetElement(row, col, rotation(row, col));
			}
		}
		matrix->SetElement(0, 3, translation[0]);
		matrix->SetElement(1, 3, translation[1]);
		matrix->SetElement(2, 3, translation[2]);
		return matrix;
	}


protected:
	QTimer* m_CameraUpdateTimer;
	std::map<std::string, vtkSmartPointer<vtkMatrix4x4>> m_ToolMatrixMap;
	std::map<std::string, Eigen::Vector3d> m_ToolTipMap;
	std::map<std::string, QLabel*> m_ToolLabelMap;
	QString m_GreenLabelStyleSheet = "QLabel { color : green; }";
	QString m_RedLabelStyleSheet = "QLabel { color : red; }";
};
#endif