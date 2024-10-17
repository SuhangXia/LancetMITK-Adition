#pragma once
#include <string>
#include "PKAData.h"
#ifndef PKAENUM
#define PKAENUM

namespace lancetAlgorithm
{
	enum class CutPlane
	{
		DistalCut,
		PosteriorCut,
		PosteriorChamferCut,
		ProximalCut
	};
	enum class KneeModel
	{
		Femur,        //股骨
		Tibia         //胫骨
	};

	enum class SurgicalType
	{
		LM,
		RM,
		LL,
		RL
	};
	enum class MedioLateral
	{
		Medial,
		Lateral
	};
	enum TCPRegistraionMethod
	{
		FourPoints,
		ThreePoints
	};
	enum class MalleolusPoint
	{
		MalleolusMedialis,  //外踝
		LateralMalleolus    //内踝
	};

	enum class ViewType
	{
		Axial,
		Sagittal,
		Coronal
	};

	inline const char* to_string(ViewType type)
	{
		switch (type)
		{
		case ViewType::Axial: return "Axial";
		case ViewType::Sagittal: return "Sagittal";
		case ViewType::Coronal: return "Coronal";
		default: return "unknown";
		}
	}

	enum class Direction
	{
		Up,
		Down,
		Left,
		Right
	};

	enum class Tilt
	{
		FrontTilt,
		BackTilt
	};

	inline std::string to_string(Tilt t)
	{
		switch (t)
		{
		case Tilt::FrontTilt:
			return "FrontTilt";
		case Tilt::BackTilt:
			return "BackTilt";
		default:
			return "unknow";
		}
	}

	enum class Va
	{
		Valgus, //外翻
		Varus   //内翻
	};

	inline std::string to_string(Va va)
	{
		switch (va)
		{
		case Va::Valgus:
			return "Valgus";
		case Va::Varus:
			return "Varus";
		default:
			return "unknow";
		}
	}

	enum class ProsRotation
	{
		IntenalRotation,
		ExternalRotation
	};

	inline std::string to_string(ProsRotation rotate)
	{
		switch (rotate)
		{
		case ProsRotation::IntenalRotation:
			return "Internal";
		case ProsRotation::ExternalRotation:
			return "External";
		default:
			return "unknow";
		}
	}

	enum class PKASurgicalSide
	{
		Left,
		Right
	};

	inline const char* to_string(PKASurgicalSide e)
	{
		switch (e)
		{
		case PKASurgicalSide::Right: return "right";
		case PKASurgicalSide::Left: return "left";
		default: return "unknown";
		}
	}

	enum class PKALandmarks
	{
		/** \brief [UserInput] 股骨内上髁 femur medial epicondyle*/
		FemurMedialEpicondyle = 0,
		/** \brief [UserInput] 股骨外上髁 femur lateral epicondyle*/
		FemurLateralEpicondyle,
		/** \brief [UserInput] 股骨内侧后髁 femur posterior medial*/
		FemurMedialPosterior,
		/** \brief [UserInput] 股骨外侧后髁  femur posterior lateral*/
		FemurLateralPosterior,
		/** \brief [UserInput] 股骨内侧远端平面点(手动输入) femur medial distal point*/
		f_MDP,
		/** \brief [UserInput] 股骨外侧远端平面点(手动输入) femur lateral distal point*/
		f_LDP,
		/** \brief [UserInput] 股骨内侧后端平面点(手动输入) femur posterior medial point*/
		f_MPP,
		/** \brief [UserInput] 股骨外侧后端平面点(手动输入) femur posterior lateral point*/
		f_LPP,
		/** \brief [UserInput] 髋关节中心  hip center*/
		HipCenter,
		/** \brief [UserInput] 股骨膝关节中心  femur knee center*/
		FemurKneeCenter,
		/** \brief [UserInput] 后交叉韧带中心	 posterior cruciate ligament center*/
		PCL_CENTER,
		/** \brief [UserInput] 胫骨膝关节中心  tibia knee center*/
		TibiaKneeCenter,
		/** \brief [UserInput] 胫骨踝关节中心  tibia ankle joint center*/
		TibiaAngleJointCenter,
		/** \brief [UserInput] 胫骨结节内侧1/3处	 Medial 1/3 of the tubercle*/
		TUBERCLE,
		/** \brief [UserInput] 胫骨平台内侧 tibia proximal medial*/
		t_PM,
		/** \brief [UserInput] 胫骨平台外侧 tibia proximal lateral*/
		t_PL,
		/** \brief [UserInput] 股骨假体前端法线起点 femur implant anterior cut start*/
		fi_ANTERIORSTART,
		/** \brief [UserInput] 股骨假体前端法线终点 femur implant anterior cut end*/
		fi_ANTERIOREND,
		/** \brief [UserInput] 股骨假体前倒角法线起点 femur implant anterior chamfer cut start*/
		fi_ANTERIORCHAMSTART,
		/** \brief [UserInput] 股骨假体前倒角法线终点 femur implant anterior chamfer cut end*/
		fi_ANTERIORCHAMEND,
		/** \brief [UserInput] 股骨假体远端法线起点 femur implant distal cut start*/
		fi_DISTALSTART,
		/** \brief [UserInput] 股骨假体远端法线终点 femur implant distal cut end*/
		fi_DISTALEND,
		/** \brief [UserInput] 股骨假体后倒角法线起点 femur implant posterior chamfer cut start*/
		fi_POSTERIORCHAMSTART,
		/** \brief [UserInput] 股骨假体后倒角法线终点 femur implant posterior chamfer cut end*/
		fi_POSTERIORCHAMEND,
		/** \brief [UserInput] 股骨假体后端法线起点 femur implant posterior cut start*/
		fi_POSTERIORSTART,
		/** \brief [UserInput] 股骨假体后端法线终点 femur implant posterior cut end*/
		fi_POSTERIOREND,
		/** \brief [UserInput] 股骨假体截骨线内侧点 femur implant resection line medial*/
		fi_ResectionMedial,
		/** \brief [UserInput] 股骨假体截骨线外侧点 femur implant resection line lateral*/
		fi_ResectionLateral,
		/** \brief [UserInput] 胫骨假体近端法线起点 tibia implant proximal cut start*/
		ti_PROXIMALSTART,
		/** \brief [UserInput] 胫骨假体近端法线终点 tibia implant proximal cut end*/
		ti_PROXIMALEND,
		/** \brief [UserInput] 胫骨假体对称轴起点 tibia implant symmetry axis start*/
		ti_SYMMETRYSTART,
		/** \brief [UserInput] 胫骨假体对称轴终点 tibia implant symmetry axis end*/
		ti_SYMMETRYEND,
		/** \brief [ModelOutput] 伸直位股骨截骨平面内侧点(用于计算关节间隙)*/
		fi_FLEXIONMEDIAL,
		/** \brief [ModelOutput] 伸直位股骨截骨平面外侧点(用于计算关节间隙)*/
		fi_FLEXIONLATERAL,
		/** \brief [ModelOutput] 屈曲位股骨截骨平面内侧点(用于计算关节间隙)*/
		fi_EXTENSIONMEDIAL,
		/** \brief [ModelOutput] 屈曲位股骨截骨平面外侧点(用于计算关节间隙)*/
		fi_EXTENSIONLATERAL,
		/** \brief [ModelOutput] 伸直位胫骨截骨平面内侧点(用于计算关节间隙)*/
		ti_FLEXIONMEDIAL,
		/** \brief [ModelOutput] 伸直位胫骨截骨平面外侧点(用于计算关节间隙)*/
		ti_FLEXIONLATERAL,
		/** \brief [ModelOutput] 屈曲位胫骨截骨平面内侧点(用于计算关节间隙)*/
		ti_EXTENSIONMEDIAL,
		/** \brief [ModelOutput] 屈曲位胫骨截骨平面外侧点(用于计算关节间隙)*/
		ti_EXTENSIONLATERAL,
		///** \brief [UserInput] 股骨假体远端内侧平面点1*/
		//fi_DISTALMEDIAL_1,
		///** \brief [UserInput] 股骨假体远端内侧平面点2*/
		//fi_DISTALMEDIAL_2,
		///** \brief [UserInput] 股骨假体远端内侧平面点3*/
		//fi_DISTALMEDIAL_3,
		///** \brief [UserInput] 股骨假体远端内侧平面点4*/
		//fi_DISTALMEDIAL_4,
		///** \brief [UserInput] 股骨假体远端外侧平面点1*/
		//fi_DISTALLATERAL_1,
		///** \brief [UserInput] 股骨假体远端内侧平面点2*/
		//fi_DISTALLATERAL_2,
		///** \brief [UserInput] 股骨假体远端内侧平面点3*/
		//fi_DISTALLATERAL_3,
		///** \brief [UserInput] 股骨假体远端内侧平面点4*/
		//fi_DISTALLATERAL_4,
	};

	inline const char* to_string(PKALandmarks t)
	{
		switch (t)
		{
		case PKALandmarks::FemurMedialEpicondyle: return "f_ME";
		case PKALandmarks::FemurLateralEpicondyle: return "f_LE";
		case PKALandmarks::FemurMedialPosterior: return "f_PM";
		case PKALandmarks::FemurLateralPosterior: return "f_PL";
		case PKALandmarks::f_MDP: return "f_MDP";
		case PKALandmarks::f_LDP: return "f_LDP";
		case PKALandmarks::f_MPP: return "f_MPP";
		case PKALandmarks::f_LPP: return "f_LPP";
		case PKALandmarks::HipCenter: return "HIP_CENTER";
		case PKALandmarks::PCL_CENTER: return "PCL_CENTER";
		case PKALandmarks::FemurKneeCenter: return "fKNEE_CENTER";
		case PKALandmarks::TibiaKneeCenter: return "tKNEE_CENTER";
		case PKALandmarks::TibiaAngleJointCenter: return "tANKLE_CENTER";
		case PKALandmarks::TUBERCLE: return "TUBERCLE";
		case PKALandmarks::t_PM: return "t_PM";
		case PKALandmarks::t_PL: return "t_PL";
		case PKALandmarks::fi_ANTERIORSTART: return "fi_ANTERIORSTART";
		case PKALandmarks::fi_ANTERIOREND: return "fi_ANTERIOREND";
		case PKALandmarks::fi_ANTERIORCHAMSTART: return "fi_ANTERIORCHAMSTART";
		case PKALandmarks::fi_ANTERIORCHAMEND: return "fi_ANTERIORCHAMEND";
		case PKALandmarks::fi_DISTALSTART: return "fi_DISTALSTART";
		case PKALandmarks::fi_DISTALEND: return "fi_DISTALEND";
		case PKALandmarks::fi_POSTERIORCHAMSTART: return "fi_POSTERIORCHAMSTART";
		case PKALandmarks::fi_POSTERIORCHAMEND: return "fi_POSTERIORCHAMEND";
		case PKALandmarks::fi_POSTERIORSTART: return "fi_POSTERIORSTART";
		case PKALandmarks::fi_POSTERIOREND: return "fi_POSTERIOREND";
		case PKALandmarks::fi_ResectionMedial: return "fi_ResectionMedial";
		case PKALandmarks::fi_ResectionLateral: return "fi_ResectionLateral";
		case PKALandmarks::ti_PROXIMALSTART: return "ti_PROXIMALSTART";
		case PKALandmarks::ti_PROXIMALEND: return "ti_PROXIMALEND";
		case PKALandmarks::ti_SYMMETRYSTART: return "ti_SYMMETRYSTART";
		case PKALandmarks::ti_SYMMETRYEND: return "ti_SYMMETRYEND";
		case PKALandmarks::fi_FLEXIONMEDIAL: return "fi_FLEXIONMEDIAL";
		case PKALandmarks::fi_FLEXIONLATERAL: return "fi_FLEXIONLATERAL";
		case PKALandmarks::fi_EXTENSIONMEDIAL: return "fi_EXTENSIONMEDIAL";
		case PKALandmarks::fi_EXTENSIONLATERAL: return "fi_EXTENSIONLATERAL";
		case PKALandmarks::ti_FLEXIONMEDIAL: return "ti_FLEXIONMEDIAL";
		case PKALandmarks::ti_FLEXIONLATERAL: return "ti_FLEXIONLATERAL";
		case PKALandmarks::ti_EXTENSIONMEDIAL: return "ti_EXTENSIONMEDIAL";
		case PKALandmarks::ti_EXTENSIONLATERAL: return "ti_EXTENSIONLATERAL";
		default: return "unknown";
		}
	}

	enum class PKAAxes
	{
		/** \brief [UserInput] 股骨解剖轴 femur anatomical axis*/
		femurAnatomicalAxis = 0,
		/** \brief [UserInput] 股骨机械轴 mechanical axis of femur*/
		FemurMechanicalAxis,
		/** \brief [UserInput] 股骨内外髁轴 femur TE axis*/
		f_TEA,
		/** \brief [UserInput] 股骨后髁轴 femur PC axis*/
		FemurPosteriorCondyleAxis,
		/** \brief [UserInput] 胫骨机械轴 mechanical axis of tibia*/
		tibiaMechanicalAxis,
		/** \brief [UserInput] 胫骨前后轴 tibia AP axis*/
		t_APA,
		/** \brief [UserInput] 股骨假体轴线 femur implant axis*/
		fi_A,
		/** \brief [UserInput] 股骨假体截骨线 femur implant bone resection line*/
		fi_BRL,
		/** \brief [UserInput] 胫骨假体轴线 tibia implant axis*/
		ti_A,
		/** \brief [UserInput] 胫骨假体平台对称轴线 tibia implant symmetry axis*/
		ti_SA,
		/** \brief [UserInput] 股骨冠状面法线 矢叉乘横 femur coronal plane normal*/
		FemurCoronalNormal,
		/** \brief [UserInput] 股骨横截面法线 从上到下 femur transverse plane normal*/
		FemurTransverseNormal,
		/** \brief [UserInput] 股骨矢状面法线 从外到内 femur sagittal plane normal */
		femurSagittalNormal,
		/** \brief [UserInput] 胫骨冠状面法线 从后向前tibia coronal plane normal*/
		TibiaCoronalNormal,
		/** \brief [UserInput] 胫骨横截面法线 从上到下 tibia transverse plane normal*/
		TibiaTransverseNormal,
		/** \brief [UserInput] 胫骨矢状面法线 冠叉乘横 tibia sagittal plane normal*/
		TibiaSagittalNormal,
	};

	inline const char* to_string(PKAAxes t)
	{
		switch (t)
		{
		case PKAAxes::femurAnatomicalAxis: return "f_AA";
		case PKAAxes::FemurMechanicalAxis: return "f_MA";
		case PKAAxes::f_TEA: return "f_TEA";
		case PKAAxes::FemurPosteriorCondyleAxis: return "f_PCA";
		case PKAAxes::tibiaMechanicalAxis: return "t_MA";
		case PKAAxes::t_APA: return "t_APA";
		case PKAAxes::fi_A: return "fi_A";
		case PKAAxes::fi_BRL: return "fi_BRL";
		case PKAAxes::ti_A: return "ti_A";
		case PKAAxes::ti_SA: return "ti_PSA";
		case PKAAxes::FemurCoronalNormal: return "f_coronal";
		case PKAAxes::FemurTransverseNormal: return "f_transverse";
		case PKAAxes::femurSagittalNormal: return "f_sagittal";
		case PKAAxes::TibiaCoronalNormal: return "t_coronal";
		case PKAAxes::TibiaTransverseNormal: return "t_transverse";
		case PKAAxes::TibiaSagittalNormal: return "t_sagittal";
		default: return "unknown";
		}
	}

	enum class PKAPlanes
	{
		/** \brief [UserInput] 股骨前端截骨面 femur anterior planar cut*/
		FEMURANTERIOR,
		/** \brief [UserInput] 股骨前倒角截骨面 femur anterior chamfer planar cut*/
		FEMURANTERIORCHAM,
		/** \brief [UserInput] 股骨远端截骨面 femur distal planar cut*/
		FEMURDISTAL,
		/** \brief [UserInput] 股骨后倒角截骨面 femur posterior planar cut*/
		FEMURPOSTERIORCHAM,
		/** \brief [UserInput] 股骨后端截骨面 femur posterior planar cut*/
		FEMURPOSTERIOR,
		/** \brief [UserInput] 胫骨近端截骨面 tibia proximal planar cut*/
		TIBIAPROXIMAL,
	};

	inline const char* to_string(PKAPlanes t)
	{
		switch (t)
		{
		case PKAPlanes::FEMURANTERIOR: return "FEMURANTERIOR";
		case PKAPlanes::FEMURANTERIORCHAM: return "FEMURANTERIORCHAM";
		case PKAPlanes::FEMURDISTAL: return "FEMURDISTAL";
		case PKAPlanes::FEMURPOSTERIORCHAM: return "FEMURPOSTERIORCHAM";
		case PKAPlanes::FEMURPOSTERIOR: return "FEMURPOSTERIOR";
		case PKAPlanes::TIBIAPROXIMAL: return "TIBIAPROXIMAL";
		default: return "unknown";
		}
	}

	enum class PKAResult
	{
		/** \brief [ModelOutput] 股骨内翻角 股骨假体轴线与股骨机械轴的夹角*/
		f_Varus,
		/** \brief [ModelOutput] 股骨外旋角 股骨假体截骨线与股骨后髁轴的夹角*/
		f_TEAExternal,
		/** \brief [ModelOutput] 股骨内旋角 股骨假体截骨线与股骨内外髁轴的夹角*/
		f_PCAExternal,
		/** \brief [ModelOutput] 股骨弯曲度 股骨假体轴线与股骨机械轴的夹角*/
		f_flexion,
		/** \brief [ModelOutput] 股骨远端内侧截骨深度 medial resection Depth of the distal femur*/
		f_Distal_MRD,
		/** \brief [ModelOutput] 股骨远端外侧截骨深度 lateral resection Depth of the distal femur*/
		f_Distal_LRD,
		/** \brief [ModelOutput] 股骨后端内侧截骨深度 medial resection Depth of the posterior femur*/
		f_Posterior_MRD,
		/** \brief [ModelOutput] 股骨后端外侧截骨深度 lateral resection Depth of the posterior femur*/
		f_Posterior_LRD,
		/** \brief [ModelOutput] 胫骨内翻角 胫骨截骨线的出垂线与下肢机械轴的夹角*/
		t_Varus,
		/** \brief [ModelOutput] 胫骨外倾角 胫骨假体轴线与胫骨机械轴的夹角*/
		t_External,
		/** \brief [ModelOutput] 胫骨后倾角 胫骨假体轴线与胫骨机械轴的夹角*/
		t_PSlope,
		/** \brief [ModelOutput] 胫骨近端内侧截骨深度 medial resection depth of the proximal tibia*/
		t_Proximal_MRD,
		/** \brief [ModelOutput] 胫骨近端外侧截骨深度 lateral resection depth of the proximal tibia*/
		t_Proximal_LRD,
		/** \brief [ModelOutput] 伸直位关节内侧间隙*/
		ExtensionMedialGap,
		/** \brief [ModelOutput] 伸直位关节外侧间隙*/
		ExtensionLateralGap,
		/** \brief [ModelOutput] 屈曲位关节内侧间隙*/
		flexionMedialGap,
		/** \brief [ModelOutput] 屈曲位关节外侧间隙*/
		flexionLateralGap,
		/** \brief [ModelOutput] 下肢屈曲度*/
		Limb_Flexion,
		/** \brief [ModelOutput] 下肢内外翻*/
		Limb_Varus,
		/** \brief [ModelOutput] 规划内外翻*/
		Planned_Varus,
		/** \brief [ModelOutput] 伸直间隙*/
		Extension,
		/** \brief [ModelOutput] 屈曲间隙*/
		Flexion,
	};

	inline const char* to_string(PKAResult t)
	{
		switch (t)
		{
		case PKAResult::f_Varus: return "f_Varus";
		case PKAResult::f_TEAExternal: return "f_External";
		case PKAResult::f_PCAExternal: return "f_Internal";
		case PKAResult::f_flexion: return "f_flexion";
		case PKAResult::f_Distal_MRD: return "f_Distal_MRD";
		case PKAResult::f_Distal_LRD: return "f_Distal_LRD";
		case PKAResult::f_Posterior_MRD: return "f_Posterior_MRD";
		case PKAResult::f_Posterior_LRD: return "f_Posterior_LRD";
		case PKAResult::t_Varus: return "t_Varus";
		case PKAResult::t_External: return "t_External";
		case PKAResult::t_PSlope: return "t_PSlope";
		case PKAResult::t_Proximal_MRD: return "t_Proximal_MRD";
		case PKAResult::t_Proximal_LRD: return "t_Proximal_LRD";
		case PKAResult::ExtensionMedialGap: return "ExtensionMedialGap";
		case PKAResult::ExtensionLateralGap: return "ExtensionLateralGap";
		case PKAResult::flexionMedialGap: return "flexionMedialGap";
		case PKAResult::flexionLateralGap: return "flexionLateralGap";
		case PKAResult::Limb_Flexion: return "Limb_Flexion";
		case PKAResult::Limb_Varus: return "Limb_Varus";
		case PKAResult::Planned_Varus: return "Planned_Varus";
		case PKAResult::Extension: return "Extension";
		case PKAResult::Flexion: return "Flexion";
		default: return "unknown";
		}
	}

	enum class PKAMarker
	{
		PKARobotBaseRF,
		PKARobotEndRF,
		PKAFemurRF, 
		PKAProbe,
		PKATibiaRF,
		PKADrill,
		PKABluntProbe
	};

	inline const char* to_string(PKAMarker p)
	{
		switch (p)
		{
		case lancetAlgorithm::PKAMarker::PKARobotBaseRF:
			return "PKARobotBaseRF";
		case lancetAlgorithm::PKAMarker::PKARobotEndRF:
			return "PKARobotEndRF";
		case lancetAlgorithm::PKAMarker::PKAFemurRF:
			return "PKAFemurRF";
		case lancetAlgorithm::PKAMarker::PKAProbe:
			return "PKAProbe";
		case lancetAlgorithm::PKAMarker::PKATibiaRF:
			return "PKATibiaRF";
		case lancetAlgorithm::PKAMarker::PKADrill:
			return "PKADrill";
		case lancetAlgorithm::PKAMarker::PKABluntProbe:
			return "PKABluntProbe";
		default:
			return "unknow";
		}
	}
	
	inline std::string to_path(PKAMarker p)
	{
		std::string path = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\";
		switch (p)
		{
		case lancetAlgorithm::PKAMarker::PKARobotBaseRF:
			return "unknow";
		case lancetAlgorithm::PKAMarker::PKARobotEndRF:
			return "unknow";
		case lancetAlgorithm::PKAMarker::PKAFemurRF:
			return "unknow";
		case lancetAlgorithm::PKAMarker::PKAProbe:
			return path + to_string(p) + ".stl";
		case lancetAlgorithm::PKAMarker::PKATibiaRF:
			return "unknow";
		case lancetAlgorithm::PKAMarker::PKADrill:
			return path + to_string(p)+".stl";
		case lancetAlgorithm::PKAMarker::PKABluntProbe:
			return path + to_string(p) + ".stl";
		default:
			return "unknow";
		}
	}
}
#endif // !PKAENUM
