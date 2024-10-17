#include "ChunLiXGImplant.h"
using namespace lancetAlgorithm;
ChunLiXGImplant::ChunLiXGImplant(mitk::DataStorage* dataSrotage, std::string path)
{
	m_Matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	SerializerFemoralImplant(path);
	m_dataStorage = dataSrotage;

	m_TransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	m_OriginalMechanicalPolyData = vtkSmartPointer<vtkPolyData>::New();
	m_MechanicalLine = vtkSmartPointer<vtkLineSource>::New();

	Eigen::Vector3d direction = CalculationHelper::CalculateDirection(m_DistalCut, m_DistalCutNormal);
	direction.normalize();
	m_MechanicalLine->SetPoint1(m_DistalCut[0], m_DistalCut[1], m_DistalCut[2]);
	m_MechanicalLine->SetPoint2(m_DistalCutNormal[0], m_DistalCutNormal[1], m_DistalCutNormal[2]);

	m_MechanicalLine->Update();
	m_OriginalMechanicalPolyData->DeepCopy(m_MechanicalLine->GetOutput());
}

Eigen::Vector3d ChunLiXGImplant::GetDistalCut()
{
	return  CalculationHelper::TransformByMatrix(m_DistalCut,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetDistalCutNormal()
{
	return CalculationHelper::TransformByMatrix(m_DistalCutNormal,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetPosteriorChamferCut()
{
	return CalculationHelper::TransformByMatrix(m_PosteriorChamferCut,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetPosteriorChamferCutNormal()
{
	return CalculationHelper::TransformByMatrix(m_PosteriorChamferCutNormal,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetPosteriorCut()
{
	return CalculationHelper::TransformByMatrix(m_PosteriorCut,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetPosteriorCutNormal()
{
	return CalculationHelper::TransformByMatrix(m_PosteriorCutNormal,m_Matrix);
}

Eigen::Vector3d ChunLiXGImplant::GetOrientation()
{
	return CalculationHelper::CalculateDirection(this->GetBackOrientationPoint(), this->GetFrontOrientationPoint());
}

void ChunLiXGImplant::UpdateImplant(vtkMatrix4x4* matrix)
{
	std::cout << "*******************************************************************************" << std::endl;
	m_Matrix->DeepCopy(matrix);
	m_Matrix->Modified();
	std::cout << "UpdateImplant" << std::endl;
	PrintDataHelper::CoutMatrix("UpdateImplant: ", m_Matrix);
	std::cout << "*******************************************************************************" << std::endl;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::ChunLiXGImplant::GetImplantMatrix()
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(m_Matrix);
	return matrix;
}

Eigen::Vector3d ChunLiXGImplant::GetMechanicalAxis()
{
	PrintDataHelper::CoutArray(CalculationHelper::CalculateDirection(this->GetDistalCut(), this->GetDistalCutNormal()), "GetMechanicalAxis previous Direction");
	return CalculationHelper::CalculateDirection(this->GetDistalCut(), this->GetDistalCutNormal());
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetProudDirection()
{
	return CalculationHelper::ComputePerpendicular(this->GetProudPoint(), this->GetSmallHolePoint(), this->GetBigHolePoint());
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetProudPoint()
{
	return CalculationHelper::TransformByMatrix(m_ProudPoint,m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetFrontOrientationPoint()
{
	return CalculationHelper::TransformByMatrix(m_FrontOrientationPoint, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetBackOrientationPoint()
{
	return CalculationHelper::TransformByMatrix(m_BackOrientationPoint, m_Matrix);
}

std::vector<Eigen::Vector3d> lancetAlgorithm::ChunLiXGImplant::GetGapPoints()
{
	std::vector<Eigen::Vector3d> gapPointsInFemur;
	for (auto point : m_ContactPoints)
	{
		gapPointsInFemur.push_back(CalculationHelper::TransformByMatrix(point, m_Matrix));
	}
	return gapPointsInFemur;
}

double lancetAlgorithm::ChunLiXGImplant::GetDistalCutWithPosteriorChamferCutAngle()
{
	Eigen::Vector3d posteriorChamferNormalDirection = CalculationHelper::CalculateDirection
	(this->GetPosteriorChamferCut(), this->GetPosteriorChamferCutNormal());
	return CalculationHelper::CalculateAngle(this->GetMechanicalAxis(), posteriorChamferNormalDirection);
}

double lancetAlgorithm::ChunLiXGImplant::GetChamferCutWithPosterioCutAngle()
{
	Eigen::Vector3d posteriorChamferNormalDirection = CalculationHelper::CalculateDirection
	(this->GetPosteriorChamferCut(), this->GetPosteriorChamferCutNormal());
	Eigen::Vector3d posteriorNormalDirection = CalculationHelper::CalculateDirection
	(this->GetPosteriorCut(), this->GetPosteriorCutNormal());
	return CalculationHelper::CalculateAngle(posteriorChamferNormalDirection, posteriorNormalDirection);
}

std::string lancetAlgorithm::ChunLiXGImplant::GetDistalCutFilePath()
{
	return m_DistalCutFilePath;
}

std::string lancetAlgorithm::ChunLiXGImplant::GetPosterioCutFilePath()
{
	return m_PosteriorCutFilePath;
}

std::string lancetAlgorithm::ChunLiXGImplant::GetPostriorChamferCutFilePath()
{
	return m_PosteriorChamferFilePath;
}

void ChunLiXGImplant::SerializerFemoralImplant(std::string path)
{
	std::ifstream ifs(m_JsonPath);
	if (!ifs.is_open()) {
		std::cerr << "Error: Failed to open JSON file: " << m_JsonPath << std::endl;
		return;
	}

	nlohmann::json jsonData;
	try {
		ifs >> jsonData;
	}
	catch (nlohmann::json::parse_error& e) {
		std::cerr << "JSON parse error: " << e.what() << std::endl;
		return;
	}

	// Check if 'stl' path matches
	if (!jsonData.contains("FemurImplant")) {
		return;
	}

	for (auto& type : jsonData["FemurImplant"])
	{
		if (!type.contains("ChunLiXG") || !type.contains("ChunLiXK"))
		{
			std::cout << "Not Contains Brand" << std::endl;
			continue;
		}
		for (auto& brand : { "ChunLiXG", "ChunLiXK" })
		{
			for (auto& xg : type[brand])
			{
				std::filesystem::path stlPath = xg["datas"]["stl"];
				//std::cout << "stlPath: " << stlPath << std::endl;
				//std::cout <<"path:"<< path << std::endl;
				int length = stlPath.string().length();
				/*std::cout << "length: " << length << std::endl;*/
				std::filesystem::path currentPath = /*'.' +*/ (FileIO::GetLastCharaters(path, length));
				//std::cout << "currentPath: " << currentPath << std::endl;
				//std::cout << "stlPath: " << stlPath << std::endl;
				if (stlPath != currentPath) {
					
					continue;
				}
				else
				{
					std::cout << "find path" << std::endl;
					std::cout << "currentPaht: " << currentPath << std::endl;
					std::cout << "stlPath: " << stlPath << std::endl;
				}

				auto data = xg["datas"];
				{
					auto load = [](const nlohmann::json& obj, const std::string& name, Eigen::Vector3d& vectorXYZ, Eigen::Vector3d& vectorNXYZ)
					{
						if (obj.contains("x") && obj.contains("y") && obj.contains("z"))
						{
							Eigen::Vector3d vecXYZ(
								std::stod(obj["x"].get<std::string>()),
								std::stod(obj["y"].get<std::string>()),
								std::stod(obj["z"].get<std::string>())
							);
							vectorXYZ = vecXYZ;
							PrintDataHelper::CoutArray(vecXYZ, (name + "XYZ").c_str());
						}
						if (obj.contains("nx") && obj.contains("ny") && obj.contains("nz"))
						{
							Eigen::Vector3d vecNXYZ(
								std::stod(obj["nx"].get<std::string>()),
								std::stod(obj["ny"].get<std::string>()),
								std::stod(obj["nz"].get<std::string>())
							);
							vectorNXYZ = vecNXYZ;
							PrintDataHelper::CoutArray(vecNXYZ, (name + "NXYZ").c_str());
						}
					};
					
					load(data["PosteriorChamferCut"], "PosteriorChamferCut", m_PosteriorChamferCut, m_PosteriorChamferCutNormal);
					load(data["DistalCut"], "DistalCut", m_DistalCut, m_DistalCutNormal);
					load(data["PosteriorCut"], "PosteriorCut", m_PosteriorCut, m_PosteriorCutNormal);
					Eigen::Vector3d tmp;
					load(data["OrientationBack"], "OrientationBack", m_FrontOrientationPoint, tmp);
					load(data["OrientationFront"], "OrientationFront", m_BackOrientationPoint, tmp);
					load(data["ProudPoint"], "ProudPoint", m_ProudPoint, tmp);
					load(data["SmallHolePoint"], "SmallHolePoint", m_ProudSmallHole, tmp);
					load(data["BigHolePoint"], "BigHolePoint", m_ProudBigHole, tmp);
					m_DistalCutFilePath = data["DistalCutFilePath"];
					
					m_PosteriorCutFilePath = data["PosteriorCutFilePath"];
					m_PosteriorChamferFilePath = data["PosteriorChamferCutFilePath"];
					std::cout << "m_DistalCutFilePath: " << m_DistalCutFilePath << std::endl;
					std::cout << "m_PosteriorCutFilePath: " << m_PosteriorCutFilePath << std::endl;
					std::cout << "m_PosteriorChamferFilePath: " << m_PosteriorChamferFilePath << std::endl;
					m_ModelOrientation = CalculationHelper::CalculateDirection(m_FrontOrientationPoint, m_BackOrientationPoint);

					std::vector<double> points = data["ContactPoints"].get<std::vector<double>>();
					for (int i = 0; i < points.size(); i += 3)
					{
						Eigen::Vector3d vec(points[i], points[i + 1], points[i + 2]);
						PrintDataHelper::CoutArray(vec, "ContactPoints");
						m_ContactPoints.push_back(vec);
					}
					return;
				}
			}
		}
	}
#if false
	for (auto& type : jsonData["type"]) {
	    if (!type.contains("JNJ")) {
	        continue;
	    }
	    for (auto& jnj : type["JNJ"]) {
	        if (!jnj.contains("datas")) {
	            continue;
	        }

	        for (auto& side : { "left", "right" }) {
	            if (!jnj["datas"].contains(side) || !jnj["datas"][side].contains("stl")) {
	                continue;
	            }

	            std::filesystem::path stlPath = jnj["datas"][side]["stl"];
	            //std::cout << stlPath << std::endl;
	            int length = stlPath.string().length() - 1;
	            std::filesystem::path currentPath = '.' + (CalculationHelper::GetLastCharaters(path, length));
	            //std::cout << currentPath << std::endl;
	            if (stlPath != currentPath) {
	                continue;
	            }

	            // Print all information for this layer
	            std::cout << "Match found for path: " << stlPath << std::endl;

	            auto load = [](const nlohmann::json& obj, const std::string& name, Eigen::Vector3d& vectorXYZ,
	                Eigen::Vector3d& vectorNXYZ) 
	            {
	                if (obj.contains("x") && obj.contains("y") && obj.contains("z")) {
	                    Eigen::Vector3d vecXYZ(
	                        std::stod(obj["x"].get<std::string>()),
	                        std::stod(obj["y"].get<std::string>()),
	                        std::stod(obj["z"].get<std::string>())
	                    );
	                    vectorXYZ = vecXYZ;
	                    PrintDataHelper::CoutArray(vecXYZ, (name+"XYZ").c_str());
	                }
	                if (obj.contains("nx") && obj.contains("ny") && obj.contains("nz")) {
	                    Eigen::Vector3d vecNXYZ(
	                        std::stod(obj["nx"].get<std::string>()),
	                        std::stod(obj["ny"].get<std::string>()),
	                        std::stod(obj["nz"].get<std::string>())
	                    );
	                    vectorNXYZ = vecNXYZ;
	                    PrintDataHelper::CoutArray(vecNXYZ, (name + "NXYZ").c_str());
	                }
	            };

	            load(jnj["datas"][side]["AnteriorChamferCut"], "AnteriorChamferCut", m_AnteriorChamferCut, m_AnteriorChamferCutNormal);
	            load(jnj["datas"][side]["DistalCut"], "DistalCut", m_DistalCut, m_DistalCutNormal);
	            //load(jnj["datas"][side]["MechanicalAxisPoint"], "MechanicalAxisPoint", m_MechanicalAxisPoint, Eigen::Vector3d());
	            load(jnj["datas"][side]["PosteriorCut"], "PosteriorCut", m_PosteriorChamferCut, m_PosteriorChamferCutNormal);
	            Eigen::Vector3d medialHole;
	            Eigen::Vector3d lateralHole;
	            load(jnj["datas"][side]["PostHoleMedial"], "PostHoleMedial", medialHole, Eigen::Vector3d());
	            load(jnj["datas"][side]["PostHoleLateral"], "PostHoleLateral", lateralHole, Eigen::Vector3d());
	            m_ModelOrientation = CalculationHelper::CalculateDirection(medialHole, lateralHole);
	        }
	    }
	}
#endif
}

void ChunLiXGImplant::UpdateMechanicalAxis()
{
	//m_MechanicalAxisDirection = CalculationHelper::TransformByMatrix(m_MechanicalAxisDirection, m_Matrix);
	//m_MechanicalAxisPoint = CalculationHelper::TransformByMatrix(m_MechanicalAxisPoint, m_Matrix);

	//PrintDataHelper::CoutArray(m_MechanicalAxisDirection, "m_MechanicalAxisDirection");
	//PrintDataHelper::CoutArray(m_MechanicalAxisPoint, "m_MechanicalAxisPoint");
}

void ChunLiXGImplant::UpdateDistalCut()
{
	//m_DistalCut = CalculationHelper::TransformByMatrix(m_DistalCut, m_Matrix);
	//m_DistalCutNormal = CalculationHelper::TransformByMatrix(m_DistalCutNormal, m_Matrix);

	//PrintDataHelper::CoutArray(m_DistalCut, "m_DistalCut");
	//PrintDataHelper::CoutArray(m_DistalCutNormal, "m_DistalCutNormal");
}

void ChunLiXGImplant::UpdateAnterior()
{
	m_PosteriorChamferCut = CalculationHelper::TransformByMatrix(m_PosteriorChamferCut, m_Matrix);
	m_PosteriorChamferCutNormal = CalculationHelper::TransformByMatrix(m_PosteriorChamferCutNormal, m_Matrix);

	PrintDataHelper::CoutArray(m_PosteriorChamferCut, "m_AnteriorChamferCut");
	PrintDataHelper::CoutArray(m_PosteriorChamferCutNormal, "m_AnteriorChamferCutNormal");
}

void ChunLiXGImplant::UpdatePosterior()
{
	m_PosteriorCut = CalculationHelper::TransformByMatrix(m_PosteriorCut, m_Matrix);
	m_PosteriorChamferCutNormal = CalculationHelper::TransformByMatrix(m_PosteriorChamferCutNormal, m_Matrix);

	PrintDataHelper::CoutArray(m_PosteriorCut, "m_PosteriorChamferCut");
	PrintDataHelper::CoutArray(m_PosteriorChamferCutNormal, "m_AnteriorChamferCutNormal");
}

void ChunLiXGImplant::UpdateOrientation()
{
	m_ModelOrientation = CalculationHelper::TransformByMatrix(m_ModelOrientation, m_Matrix);
	PrintDataHelper::CoutArray(m_ModelOrientation, "m_ModelOrientation");
}

void lancetAlgorithm::ChunLiXGImplant::UpdateProud()
{
	m_ProudPoint = CalculationHelper::TransformByMatrix(m_ProudPoint, m_Matrix);
	m_ProudTangentLineDirection = CalculationHelper::TransformByMatrix(m_ProudTangentLineDirection, m_Matrix);
	m_ProudBigHole = CalculationHelper::TransformByMatrix(m_ProudBigHole, m_Matrix);
	m_ProudSmallHole = CalculationHelper::TransformByMatrix(m_ProudSmallHole, m_Matrix);
	PrintDataHelper::CoutArray(m_ProudPoint, "m_ProudPoint");
	PrintDataHelper::CoutArray(m_ProudTangentLineDirection, "m_ProudTangentLineDirection");
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetSmallHolePoint()
{
	return CalculationHelper::TransformByMatrix(m_ProudSmallHole, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiXGImplant::GetBigHolePoint()
{
	return CalculationHelper::TransformByMatrix(m_ProudBigHole, m_Matrix);
}
