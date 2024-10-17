#include "ChunLITray.h"

lancetAlgorithm::ChunLiTray::ChunLiTray(mitk::DataStorage* dataStorage, std::string path)
{
	m_Matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	SerializerFemoralImplant(path);
	m_dataStorage = dataStorage;
}

void lancetAlgorithm::ChunLiTray::SerializerFemoralImplant(std::string path)
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
	if (!jsonData.contains("TibiaTray")) {
		return;
	}

	for (auto& type : jsonData["TibiaTray"])
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
				int length = stlPath.string().length();
				std::filesystem::path currentPath = (FileIO::GetLastCharaters(path, length));

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
					load(data["ProximalCut"], "ProximalCut", m_Proximal, m_ProximalNormal);
					Eigen::Vector3d orientationNormal;
					load(data["OrientationBack"], "OrientationBack", m_FrontOrientationPoint, orientationNormal);
					load(data["OrientationFront"], "OrientationFront", m_BackOrientationPoint, orientationNormal);

					//计算水平方向，用于计算内外旋
					m_ModelOrientation = CalculationHelper::CalculateDirection(m_FrontOrientationPoint, m_BackOrientationPoint);
					m_ProximalCutFilePath = data["ProximalCutFilePath"];
					std::cout << "m_ProximalCutFilePath: " << m_ProximalCutFilePath << std::endl;
					std::string thicknessStr = data["thickness"];
					thickness = std::stod(thicknessStr);
					Eigen::Vector3d origin;
					origin.Zero();
					//Proud初始方向为proximal 的反方向
					m_ProudDirection = -m_ProximalNormal;
					PrintDataHelper::CoutArray(m_ProudDirection, "m_ProudDirection: ");
					//proud 初始位置为原点减去厚度
					m_ProudPoint = origin + (thickness) * m_ProudDirection;
					PrintDataHelper::CoutArray(m_ProudPoint, "Tibia m_ProudPoint");
					std::cout << "thickness: " << thickness << std::endl;
					return;
				}
			}
		}
	}
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetOrientation()
{
	return CalculationHelper::CalculateDirection(this->GetFrontOrientationPoint(), this->GetBackOrientationPoint());
}

void lancetAlgorithm::ChunLiTray::UpdateTray(vtkMatrix4x4* matrix)
{
	m_Matrix->DeepCopy(matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetProximalDirection()
{
	return CalculationHelper::CalculateDirection(this->GetProximal(), this->GetProximalNormal());
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetProximal()
{
	return CalculationHelper::TransformByMatrix(m_Proximal, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetProudDirection()
{
	return CalculationHelper::TransformByMatrix(m_ProudDirection, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetProudPoint()
{
	return CalculationHelper::TransformByMatrix(m_ProudPoint, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetProximalNormal()
{
	return  CalculationHelper::TransformByMatrix(m_ProximalNormal, m_Matrix);
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::ChunLiTray::GetTrayMatrix()
{
	return m_Matrix;
}

std::string lancetAlgorithm::ChunLiTray::GetProximalCutFilePath()
{
	return m_ProximalCutFilePath;
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetFrontOrientationPoint()
{
	return CalculationHelper::TransformByMatrix(m_FrontOrientationPoint, m_Matrix);
}

Eigen::Vector3d lancetAlgorithm::ChunLiTray::GetBackOrientationPoint()
{
	return CalculationHelper::TransformByMatrix(m_BackOrientationPoint, m_Matrix);
}
