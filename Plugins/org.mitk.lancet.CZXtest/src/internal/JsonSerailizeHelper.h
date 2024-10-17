#pragma once
#ifndef PKAJSONSERIALIZE
#define PKAJSONSERIALIZE

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
namespace lancetAlgorithm
{
	class Point
	{
	public:
		std::string x, y, z;
		Point(const Eigen::Vector3d Pos = Eigen::Vector3d::Zero())
		{
			x = std::to_string(Pos[0]);
			y = std::to_string(Pos[1]);
			z = std::to_string(Pos[2]);
		}
	};

	inline void to_json(nlohmann::json& json, const Point& p)
	{
		json = nlohmann::json{ {"x", p.x}, {"y", p.y}, {"z", p.z} };
	}

	class Cut {
	public:
		std::string x, y, z, nx, ny, nz/*, stl*/;
		Cut(const Eigen::Vector3d Pos = Eigen::Vector3d::Zero(), const Eigen::Vector3d direction = Eigen::Vector3d::Zero())
		{
			x = std::to_string(Pos[0]);
			y = std::to_string(Pos[1]);
			z = std::to_string(Pos[2]);
			nx = std::to_string(direction[0]);
			ny = std::to_string(direction[1]);
			nz = std::to_string(direction[2]);
		}
	};

	inline void to_json(nlohmann::json& json, const Cut& cut)
	{
		json = nlohmann::json{ {"x", cut.x}, {"y", cut.y}, {"z", cut.z}, {"nx", cut.nx}, {"ny", cut.ny}, {"nz", cut.nz} };
	}

	class Data
	{
	public:
		//stl相对路径
		std::string stl;
		Cut DistalCut;
		Cut PosteriorChamferCut;
		Cut PosteriorCut;
		Point OrientationFront;
		Point OrientationBack;
		Point ProudPoint;
		Point SmallHolePoint;
		Point BigHolePoint;
		std::string CurvePlane;
		std::vector<double> ContactPoints;
		std::string DistalCutFilePath;
		std::string PosteriorCutFilePath;
		std::string PosteriorChamferCutFilePath;
		Data(const std::string& stl = "",
			const Cut& DistalCut = Cut(),
			const Cut& AnteriorChamferCut = Cut(),
			const Cut& PosteriorChamferCut = Cut(),
			const Point& OrientationFront = Point(),
			const Point& OrientationBack = Point(),
			const Point& ProudPoint = Point(),
			const Point& SmallHolePoint = Point(),
			const Point& BigHolePoint = Point(),
			const std::string& CurvePlane = "", const std::vector<double> ContactPoints = std::vector<double>(),
			const std::string& DistalCutFilePath = "", const std::string& PosteriorCutFilePath = "",
			const std::string& PosteriorChamferCutFilePath = "")
			: stl(stl), DistalCut(DistalCut), PosteriorChamferCut(AnteriorChamferCut), PosteriorCut(PosteriorChamferCut),
			OrientationFront(OrientationFront), OrientationBack(OrientationBack), ProudPoint(ProudPoint),
			SmallHolePoint(SmallHolePoint), BigHolePoint(BigHolePoint), ContactPoints(ContactPoints), DistalCutFilePath(DistalCutFilePath),
			PosteriorCutFilePath(PosteriorCutFilePath), PosteriorChamferCutFilePath(PosteriorChamferCutFilePath){}
	};

	inline void to_json(nlohmann::json& json, const Data& data)
	{
		json = nlohmann::json{
			{"stl", data.stl},
			{"DistalCut", data.DistalCut},
			{"PosteriorChamferCut", data.PosteriorChamferCut},
			{"PosteriorCut", data.PosteriorCut},
			{"OrientationFront", data.OrientationFront},
			{"OrientationBack", data.OrientationBack},
			{"ProudPoint", data.ProudPoint},
			{"SmallHolePoint", data.SmallHolePoint},
			{"BigHolePoint", data.BigHolePoint},
			{"ContactPoints", data.ContactPoints},
			{"CurvePlane", data.CurvePlane},
			{"DistalCutFilePath", data.DistalCutFilePath},
			{"PosteriorCutFilePath", data.PosteriorCutFilePath},
			{"PosteriorChamferCutFilePath", data.PosteriorChamferCutFilePath}
		};
	}

	//inline void to_json(nlohmann::json& json, const std::vector<Data>& dataList)
	//{
	//	json = nlohmann::json::array();
	//	for (const auto& data : dataList) {
	//		json.push_back(data);
	//	}
	//}

	//TypoName ChunLiXG ChunLiXK
	class Prosthesis
	{
	public:
		std::string Number;
		Data data;
		Prosthesis(const std::string& number, const Data& data)
			: Number(number), data(data) {}
	};

	inline void to_json(nlohmann::json& json, const Prosthesis& pros)
	{
		json = nlohmann::json{ {"number", pros.Number} ,{"datas", pros.data} };
	}

	inline void to_json(nlohmann::json& json, const std::vector<Prosthesis> prosList)
	{
		json = nlohmann::json::array();
		for (const auto& pros : prosList) {
			json.push_back(pros);
		}
	}

	class TibiaData
	{
	public:
		std::string stl;
		std::string thickness;
		Cut ProximalCut;
		Point OrientationFront;
		Point OrientationBack;
		std::string UnderSurface;
		std::string VerticalSurface;
		std::string ProximalCutFilePath;
		TibiaData(const std::string& stl = "",const std::string& thickness ="0", const Cut& ProximalCut = Cut(), const Point& OrientationFront = Point(),
			const Point& OrientationBack = Point(),
			const std::string& UnderSurface = "",
			const std::string& VerticalSurface = "", const std::string& ProximalCutFilePath = "") :stl(stl), thickness(thickness), ProximalCut(ProximalCut),
			OrientationFront(OrientationFront), OrientationBack(OrientationBack),
			UnderSurface(UnderSurface), VerticalSurface(VerticalSurface), ProximalCutFilePath(ProximalCutFilePath){}
	};

	inline void to_json(nlohmann::json& json, const TibiaData& tibiaData)
	{
		json = nlohmann::json{
			{"stl",tibiaData.stl},
			{"thickness", tibiaData.thickness},
			{"ProximalCut", tibiaData.ProximalCut},
			{"OrientationFront", tibiaData.OrientationFront},
			{"OrientationBack", tibiaData.OrientationBack},
			{"UnderSurface", tibiaData.UnderSurface},
			{"VerticalSurface", tibiaData.VerticalSurface},
			{"ProximalCutFilePath", tibiaData.ProximalCutFilePath}
		};
	}

	class TibiaTray
	{
	public:
		std::string Number;
		TibiaData data;
		TibiaTray(const std::string& number, const TibiaData& data)
			: Number(number), data(data) {}
	};

	inline void to_json(nlohmann::json& json, const TibiaTray& tibiaTray)
	{
		json = nlohmann::json{ {"number", tibiaTray.Number} ,{"datas", tibiaTray.data} };
	}

	inline void to_json(nlohmann::json& json, const std::vector<TibiaTray>& tibiaTrayList)
	{
		json = nlohmann::json::array();
		for (const auto& tibiaTray : tibiaTrayList) {
			json.push_back(tibiaTray);
		}
	}

	//品牌
	class Brand {
	public:
		//brand Name
		std::vector<Prosthesis> Prostheses;

		Brand(const std::vector<Prosthesis>& prostheses) : Prostheses(prostheses) {}
	};

	inline void to_json(nlohmann::json& json, const Brand& brand, const std::string& brandName)
	{
		json = nlohmann::json
		{
			{brandName, brand.Prostheses}
		};
	}

	class TrayBrand
	{
	public:
		std::vector<TibiaTray> TrayList;
		TrayBrand(const std::vector<TibiaTray>& trayList) :TrayList(trayList) {}
	};

	inline void to_json(nlohmann::json& json, const TrayBrand& brand, const std::string& brandName)
	{
		json = nlohmann::json
		{
			{brandName, brand.TrayList}
		};
	}

	class Type
	{
	public:
		std::vector<std::pair<std::string, Brand>> Brands;
		Type(const std::vector<std::pair<std::string, Brand>> Brands) : Brands(Brands) {}
	};

	// 将 Type 转换为 JSON
	inline void to_json(nlohmann::json& j, const Type& type) {
		nlohmann::json brandsJson = nlohmann::json::array();
		for (const auto& brand : type.Brands) {
			nlohmann::json brandJson;
			to_json(brandJson, brand.second, brand.first);
			brandsJson.push_back(brandJson);
		}

		j = nlohmann::json{
			{"FemurImplant", brandsJson}
		};
	}

	class TibiaType
	{
	public: 
		std::vector<std::pair<std::string, TrayBrand>> Trays;
		TibiaType(const std::vector<std::pair<std::string, TrayBrand>> Trays) : Trays(Trays) {}
	};

	inline void to_json(nlohmann::json& j, const TibiaType& type) {
		nlohmann::json traysJson = nlohmann::json::array();
		for (const auto& tray : type.Trays) {
			nlohmann::json trayJson;
			to_json(trayJson, tray.second, tray.first);
			traysJson.push_back(trayJson);
		}

		j = nlohmann::json{
			{"TibiaTray", traysJson}
		};
	}
}
#endif // !PKAJSONSERIALIZE

