#pragma once
#ifndef CHUNLITRAY
#define CHUNLITRAY

#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <mitkDataStorage.h>
#include <vtkLineSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyData.h>
#include "CalculationHelper.h"
#include "PrintDataHelper.h"
#include "FileIO.h"
namespace lancetAlgorithm
{
	class ChunLiTray
	{
	public:
		ChunLiTray(mitk::DataStorage* dataStorage, std::string path);

		Eigen::Vector3d GetOrientation();
		void UpdateTray(vtkMatrix4x4* matrix);
		Eigen::Vector3d GetProximalDirection();
		Eigen::Vector3d GetProximal();
		Eigen::Vector3d GetProudDirection();
		Eigen::Vector3d GetProudPoint();
		Eigen::Vector3d GetFrontOrientationPoint();
		Eigen::Vector3d GetBackOrientationPoint();
		Eigen::Vector3d GetProximalNormal();

		//胫骨图像坐标系到胫骨假体的位姿变换
		vtkSmartPointer<vtkMatrix4x4> GetTrayMatrix();
		std::string GetProximalCutFilePath();
	private:
		void SerializerFemoralImplant(std::string path);
		
	private:
		mitk::DataStorage* m_dataStorage;
		//Eigen::Vector3d m_MechanicalAxisPoint;
		Eigen::Vector3d m_Proximal;
		Eigen::Vector3d m_ProximalNormal;
		Eigen::Vector3d m_ModelOrientation;
		Eigen::Vector3d m_ProudPoint;
		Eigen::Vector3d m_ProudDirection;
		Eigen::Vector3d m_FrontOrientationPoint;
		Eigen::Vector3d m_BackOrientationPoint;
		std::string m_JsonPath = "E:\\PKAModelData\\CHUNLI-danke\\config.json";//"E:\\PKAModelData\\Model\\ChunLi\\config.json";//"E:\\PKAModelData\\data\\prosthesisLibrary\\prosthesis.json";
		vtkSmartPointer<vtkMatrix4x4> m_Matrix;
		vtkSmartPointer<vtkLineSource> m_MechanicalLine;
		vtkSmartPointer<vtkPolyData> m_OriginalMechanicalPolyData;
		vtkSmartPointer<vtkPolyData> m_OriginOrientationPolyData;
		vtkSmartPointer<vtkTransformPolyDataFilter> m_TransformPolyDataFilter;
		std::string m_ProximalCutFilePath;
		double m_TibiaTraySize;
		double thickness = 0.0;
	};
}
#endif // !CHUNLITRAY




