#pragma once
#ifndef PKAFEMORLALImplant
#define PKAFEMORLALImplant

#include <iostream>
#include <qobject.h>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <qtimer.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <mitkDataStorage.h>
#include <vtkLineSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyData.h>
#include "CalculationHelper.h"
#include "PrintDataHelper.h"
#include "FileIO.h"
#include "PKAData.h"
namespace lancetAlgorithm
{
	class ChunLiXGImplant : public QObject
	{
		Q_OBJECT
	public:
		ChunLiXGImplant(mitk::DataStorage* dataStorage,std::string path);
		Eigen::Vector3d GetDistalCut();
		Eigen::Vector3d GetDistalCutNormal();
		Eigen::Vector3d GetPosteriorChamferCut();
		Eigen::Vector3d GetPosteriorChamferCutNormal();
		Eigen::Vector3d GetPosteriorCut();
		Eigen::Vector3d GetPosteriorCutNormal();
		Eigen::Vector3d GetOrientation();
		void UpdateImplant(vtkMatrix4x4* matrix);
		vtkSmartPointer<vtkMatrix4x4> GetImplantMatrix();
		Eigen::Vector3d GetMechanicalAxis();
		Eigen::Vector3d GetProudDirection();
		Eigen::Vector3d GetProudPoint();
		Eigen::Vector3d GetFrontOrientationPoint();
		Eigen::Vector3d GetBackOrientationPoint();
		std::vector<Eigen::Vector3d> GetGapPoints();
		double GetDistalCutWithPosteriorChamferCutAngle();
		double GetChamferCutWithPosterioCutAngle();
		std::string GetDistalCutFilePath();
		std::string GetPosterioCutFilePath();
		std::string GetPostriorChamferCutFilePath();
	private:
		void SerializerFemoralImplant(std::string path);
		void UpdateMechanicalAxis();
		void UpdateDistalCut();
		void UpdateAnterior();
		void UpdatePosterior();
		void UpdateOrientation();
		void UpdateProud();
		Eigen::Vector3d GetSmallHolePoint();
		Eigen::Vector3d GetBigHolePoint();
		//void Calculation
	private:
		mitk::DataStorage* m_dataStorage;
		//Eigen::Vector3d m_MechanicalAxisPoint;
		Eigen::Vector3d m_DistalCut;
		Eigen::Vector3d m_PosteriorChamferCut;
		Eigen::Vector3d m_PosteriorCut;
		Eigen::Vector3d m_MechanicalAxisDirection;
		Eigen::Vector3d m_DistalCutNormal;
		Eigen::Vector3d m_PosteriorChamferCutNormal;
		Eigen::Vector3d m_PosteriorCutNormal;
		Eigen::Vector3d m_ModelOrientation;
		Eigen::Vector3d m_ProudPoint;
		Eigen::Vector3d m_ProudTangentLineDirection;
		Eigen::Vector3d m_ProudBigHole;
		Eigen::Vector3d m_ProudSmallHole;
		Eigen::Vector3d m_FrontOrientationPoint;
		Eigen::Vector3d m_BackOrientationPoint;
		std::vector<Eigen::Vector3d> m_ContactPoints;
		std::string m_JsonPath = "E:\\PKAModelData\\CHUNLI-danke\\config.json";//"E:\\PKAModelData\\Model\\ChunLi\\config.json";//"E:\\PKAModelData\\data\\prosthesisLibrary\\prosthesis.json";
		vtkSmartPointer<vtkMatrix4x4> m_Matrix;
		vtkSmartPointer<vtkLineSource> m_MechanicalLine;
		vtkSmartPointer<vtkPolyData> m_OriginalMechanicalPolyData;
		vtkSmartPointer<vtkPolyData> m_OriginOrientationPolyData;
		vtkSmartPointer<vtkTransformPolyDataFilter> m_TransformPolyDataFilter;
		double m_FemurImplantSize;
		double m_TibiaInsertSize;
		QTimer* m_IntraTransformTimer = nullptr;
		std::string m_DistalCutFilePath;
		std::string m_PosteriorCutFilePath;
		std::string m_PosteriorChamferFilePath;
	};
}
#endif // !PKAFEMORLAImplant

