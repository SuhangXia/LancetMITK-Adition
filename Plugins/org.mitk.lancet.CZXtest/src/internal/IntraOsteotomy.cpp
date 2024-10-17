#include "IntraOsteotomy.h"

lancetAlgorithm::IntraOsteotomy::IntraOsteotomy(mitk::DataStorage* dataStorage, PKADianaAimHardwareDevice* pkaDianaAimHardwareDevice,
	ChunLiXGImplant* aChunLiXGImplant, ChunLiTray* aChunLITray)
{
	m_DataStorage = dataStorage;
	m_PKADianaAimHardwareDevice = pkaDianaAimHardwareDevice;
	m_LastRoundMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	m_ChunLiTray = aChunLITray;
	m_ChunLiXGImplant = aChunLiXGImplant;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateTDrillEnd2FemurImage(/*Eigen::Vector3d pointInImage*/)
{
	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

	//从机械臂出获得当前末端的位姿
	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2Base = vtkSmartPointer<vtkMatrix4x4>::New();  //末端tcp的位置
	TDrillEnd2Base->DeepCopy(CalculateTBase2DrillEnd());
	TDrillEnd2Base->Invert();
	
	transform->PreMultiply();
	vtkSmartPointer<vtkMatrix4x4> TBase2BaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2BaseRF->DeepCopy(PKAData::m_TBaseRF2Base);
	TBase2BaseRF->Invert();
	vtkSmartPointer<vtkMatrix4x4> TBaseRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBaseRF2Camera->DeepCopy(PKAData::m_TCamera2BaseRF);
	TBaseRF2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCamera2FemurRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2FemurRF->DeepCopy(PKAData::m_TCamera2FemurRF);

	vtkSmartPointer<vtkMatrix4x4> TFemurRF2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TFemurRF2FemurImage->DeepCopy(PKAData::m_TFemurRF2FemurImage);
	transform->SetMatrix(TDrillEnd2Base);
	transform->Concatenate(TBase2BaseRF);
	transform->Concatenate(TBaseRF2Camera);
	transform->Concatenate(TCamera2FemurRF);
	transform->Concatenate(TFemurRF2FemurImage);

	transform->GetMatrix(TDrillEnd2FemurImage);
	return TDrillEnd2FemurImage;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateTDrillEnd2FemurImage2()
{
	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2Drill = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrillEnd2Drill->Identity();
	TDrillEnd2Drill->SetElement(0, 3, 230.492);
	TDrillEnd2Drill->SetElement(1, 3, -85);
	TDrillEnd2Drill->SetElement(2, 3, -49.800);
	TDrillEnd2Drill->Invert();
	vtkSmartPointer<vtkMatrix4x4> TDrill2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrill2Camera->DeepCopy(PKAData::m_TCamera2Drill);
	TDrill2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCamera2FemurRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2FemurRF->DeepCopy(PKAData::m_TCamera2FemurRF);

	vtkSmartPointer<vtkMatrix4x4> TFemurRF2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TFemurRF2FemurImage->DeepCopy(PKAData::m_TFemurRF2FemurImage);

	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrillEnd2FemurImage->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TDrillEnd2Drill, TDrill2Camera, TCamera2FemurRF, TFemurRF2FemurImage));

	return TDrillEnd2FemurImage;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateTDrillEnd2TibiaImage2()
{
	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2Drill = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrillEnd2Drill->Identity();
	TDrillEnd2Drill->SetElement(0, 3, 230.492);
	TDrillEnd2Drill->SetElement(1, 3, -85);
	TDrillEnd2Drill->SetElement(2, 3, -49.800);
	TDrillEnd2Drill->Invert();
	vtkSmartPointer<vtkMatrix4x4> TDrill2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrill2Camera->DeepCopy(PKAData::m_TCamera2Drill);
	TDrill2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCamera2TibiaRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2TibiaRF->DeepCopy(PKAData::m_TCamera2TibiaRF);

	vtkSmartPointer<vtkMatrix4x4> TTibia2TibiaImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibia2TibiaImage->DeepCopy(PKAData::m_TTibiaRF2TibiaImage);

	vtkSmartPointer<vtkMatrix4x4> TDrillEnd2TibiaImage = vtkSmartPointer<vtkMatrix4x4>::New();
	TDrillEnd2TibiaImage->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TDrillEnd2Drill, TDrill2Camera, TCamera2TibiaRF, TTibia2TibiaImage));
	
	return TDrillEnd2TibiaImage;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateTBase2DrillEnd()
{
	vtkSmartPointer<vtkMatrix4x4> TBase2BaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2BaseRF->DeepCopy(PKAData::m_TBaseRF2Base);
	TBase2BaseRF->Invert();
	vtkSmartPointer<vtkMatrix4x4> TBaseRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBaseRF2Camera->DeepCopy(PKAData::m_TCamera2BaseRF);
	TBaseRF2Camera->Invert();

	//机械臂末端参考架与 机械臂配准参考架相同
	vtkSmartPointer<vtkMatrix4x4> TCamera2EndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2EndRF->DeepCopy(PKAData::m_TCamera2Drill);

	vtkSmartPointer<vtkMatrix4x4> TEndRF2DrillEnd = vtkSmartPointer<vtkMatrix4x4>::New();
	TEndRF2DrillEnd->Identity();
	TEndRF2DrillEnd->SetElement(0, 3, 230.492);
	TEndRF2DrillEnd->SetElement(1, 3, -85);
	TEndRF2DrillEnd->SetElement(2, 3, -49.800);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

	transform->PreMultiply();
	transform->SetMatrix(TBase2BaseRF);
	transform->Concatenate(TBaseRF2Camera);
	transform->Concatenate(TCamera2EndRF);
	transform->Concatenate(TEndRF2DrillEnd);
	transform->Update();
	
	vtkSmartPointer<vtkMatrix4x4> TBase2DrillEnd = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(TBase2DrillEnd);
	return TBase2DrillEnd;
}

/// <summary>
/// 
/// </summary>
/// <param name="pointInPlane">股骨图像中的点的位置 </param>
/// <param name="planeNormal">股骨图像中平面的法向量</param>
vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateDrillEndHorizontalPlane(Eigen::Vector3d pointPlaneInImage, Eigen::Vector3d planeNormalInImage)
{
	vtkSmartPointer<vtkMatrix4x4> tDrillEnd2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	tDrillEnd2FemurImage->DeepCopy(this->CalculateTDrillEnd2FemurImage());

	Eigen::Vector3d pointPlaneInDrill = CalculationHelper::TransformByMatrix(pointPlaneInImage, tDrillEnd2FemurImage);
	Eigen::Vector3d planeNormalInDrill = CalculationHelper::TransformByMatrix(planeNormalInImage, tDrillEnd2FemurImage);

	planeNormalInDrill.normalize();

	//获得当前tcp的 xyz rx ry rz
	vtkSmartPointer<vtkMatrix4x4> TBase2DrillEnd = vtkSmartPointer<vtkMatrix4x4>::New();

	Eigen::Vector3d originZAxis(0, 0, 1);
	Eigen::Vector3d rotationAxis = originZAxis.cross(planeNormalInDrill);
	double angle = std::acos(originZAxis.dot(planeNormalInDrill));

	if (rotationAxis.norm() < 1e-6)
	{
		return vtkSmartPointer<vtkMatrix4x4>::New();
	}

	rotationAxis.normalize();

	Eigen::AngleAxisd rotation(angle, rotationAxis);
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
	pointPlaneInDrill[0] = pointPlaneInDrill[0] - 20;
	pointPlaneInDrill[1] = pointPlaneInDrill[1] - 20;
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.linear() = rotationMatrix;
	transform.translation() = pointPlaneInDrill;
	Eigen::Matrix4d m = transform.matrix();
	return CalculationHelper::ConvertEigen2vtkMatrix(m);
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateFemurDrillEndVerticalPlane(Eigen::Vector3d pointInPlane, Eigen::Vector3d planeNormal)
{
	vtkSmartPointer<vtkMatrix4x4> tDrillEnd2FemurImage = vtkSmartPointer<vtkMatrix4x4>::New();
	tDrillEnd2FemurImage->DeepCopy(this->CalculateTDrillEnd2FemurImage2());
	PrintDataHelper::CoutMatrix("tDrillEnd2FemurImage:", tDrillEnd2FemurImage);
	Eigen::Vector3d pointPlaneInDrill = CalculationHelper::TransformByMatrix(pointInPlane, tDrillEnd2FemurImage);
	Eigen::Vector3d planeNormalInDrill = CalculationHelper::CalculateInCoordinate(planeNormal, tDrillEnd2FemurImage);
	planeNormalInDrill.normalize();
	
	Eigen::Vector3d ZAxis(0, 0, 1);
	Eigen::Vector3d YAxis = ZAxis.cross(planeNormalInDrill);
	Eigen::Vector3d ZNewAxis = planeNormalInDrill.cross(YAxis);
	YAxis.normalize();
	ZNewAxis.normalize();
	planeNormalInDrill.normalize();

	Eigen::Vector3d t = pointPlaneInDrill - 50 * planeNormalInDrill;

	vtkSmartPointer<vtkMatrix4x4> tcpToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	CalculationHelper::SetMatrixXAxis(planeNormalInDrill, tcpToTarget);
	CalculationHelper::SetMatrixYAxis(YAxis, tcpToTarget);
	CalculationHelper::SetMatrixZAxis(ZNewAxis, tcpToTarget);
	CalculationHelper::SetMatrixTranslation(t, tcpToTarget);

	PrintDataHelper::CoutMatrix("tcpToTarget", tcpToTarget);
	return tcpToTarget;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateTibiaDrillEndVerticalPlane(Eigen::Vector3d pointInPlane, Eigen::Vector3d planeNormal)
{
	vtkSmartPointer<vtkMatrix4x4> tDrillEnd2TibiaImage = vtkSmartPointer<vtkMatrix4x4>::New();
	tDrillEnd2TibiaImage->DeepCopy(this->CalculateTDrillEnd2TibiaImage2());
	PrintDataHelper::CoutMatrix("tDrillEnd2TibiaImage:", tDrillEnd2TibiaImage);
	Eigen::Vector3d pointPlaneInDrill = CalculationHelper::TransformByMatrix(pointInPlane, tDrillEnd2TibiaImage);
	Eigen::Vector3d planeNormalInDrill = CalculationHelper::CalculateInCoordinate(planeNormal, tDrillEnd2TibiaImage);
	planeNormalInDrill.normalize();
	
	Eigen::Vector3d ZAxis(0, 0, 1);
	Eigen::Vector3d YAxis = ZAxis.cross(planeNormalInDrill);
	Eigen::Vector3d ZNewAxis = planeNormalInDrill.cross(YAxis);
	YAxis.normalize();
	ZNewAxis.normalize();
	planeNormalInDrill.normalize();

	Eigen::Vector3d t = pointPlaneInDrill - 50 * planeNormalInDrill;

	vtkSmartPointer<vtkMatrix4x4> tcpToTarget = vtkSmartPointer<vtkMatrix4x4>::New();
	CalculationHelper::SetMatrixXAxis(planeNormalInDrill, tcpToTarget);
	CalculationHelper::SetMatrixYAxis(YAxis, tcpToTarget);
	CalculationHelper::SetMatrixZAxis(ZNewAxis, tcpToTarget);
	CalculationHelper::SetMatrixTranslation(t, tcpToTarget);

	PrintDataHelper::CoutMatrix("tcpToTarget", tcpToTarget);
	return tcpToTarget;
}

/// <summary>
/// 输入股骨图像上的点和法向量
/// </summary>
/// <param name="pointInPlane"></param>
/// <param name="planeNormal"></param>
/// <returns></returns>
vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::IntraOsteotomy::CalculateFemurDrillEndTipPos( 
	Eigen::Vector3d pointInPlane, Eigen::Vector3d planeNormal)
{
	// 将法向量归一化以得到 X 轴方向
	Eigen::Vector3d xAxis = planeNormal.normalized();

	// 计算局部坐标系原点，向法向量负方向移动 50 毫米
	Eigen::Vector3d origin = pointInPlane - xAxis * 50.0; // 50 毫米 = 0.05 米

	// 确定 Y 轴方向：选择一个与法向量不平行的向量进行叉乘
	Eigen::Vector3d arbitraryVector(0.0, 1.0, 0.0); // 假设一个任意向量 Y 轴
	if (fabs(xAxis.dot(arbitraryVector)) > 0.99) {
		// 如果假定向量与法向量接近平行，选择另一个向量
		arbitraryVector = Eigen::Vector3d(1.0, 0.0, 0.0);
	}

	// 计算 Y 轴：选择一个不平行向量叉乘得到一个候选 Z 轴
	Eigen::Vector3d zAxis = xAxis.cross(arbitraryVector).normalized();

	// 计算 Z 轴
	Eigen::Vector3d yAxis = zAxis.cross(xAxis).normalized(); // 通过与 X 轴叉乘得到正交 Y 轴

	// 组装局部坐标系的旋转矩阵和变换矩阵
	vtkSmartPointer<vtkMatrix4x4> localCoordinateSystem = vtkSmartPointer<vtkMatrix4x4>::New();
	localCoordinateSystem->Identity(); // 初始化为单位矩阵

	// 设置旋转矩阵部分
	CalculationHelper::SetMatrixXAxis(xAxis, localCoordinateSystem);
	CalculationHelper::SetMatrixYAxis(yAxis, localCoordinateSystem);
	CalculationHelper::SetMatrixZAxis(zAxis, localCoordinateSystem);
	CalculationHelper::SetMatrixTranslation(origin, localCoordinateSystem);

	return localCoordinateSystem;
}

void lancetAlgorithm::IntraOsteotomy::CalculateBoolean(std::string nodeNameA, MR::Mesh& MeshA,
	MR::Mesh& MeshB, MR::BooleanOperation operationOne, std::string nodeNameB, MR::Mesh& MeshC,
	MR::Mesh& MeshD, MR::BooleanOperation operationTwo, MR::BooleanOperation operationThree)
{
	if (m_DataStorage->GetNamedObject<mitk::Surface>(nodeNameA)->GetVtkPolyData()->GetPoints()->GetNumberOfPoints() <= 0)
		return;

	auto intersectResult = (MR::boolean(MeshA, MeshB, operationOne));

	MR::Mesh intersectMesh = *intersectResult;
	std::cout << "CalculateBoolean:" << intersectMesh.points.vec_.size() << std::endl;
	if (intersectMesh.points.vec_.size() > 0)
	{
		auto diffResult = (MR::boolean(MeshC, MeshD, operationTwo));
		MR::Mesh testMesh = *diffResult;

		if (testMesh.points.vec_.size() > 0)
		{
			auto cutResult = (MR::boolean(MeshC, MeshD, operationThree));
			MR::Mesh shellMesh = *cutResult;
			MeshC = shellMesh;
			MeshC.invalidateCaches();
			MeshD.invalidateCaches();

			vtkSmartPointer<vtkPolyData> ShellPolyData = vtkSmartPointer<vtkPolyData>::New();
			TurnMRMeshIntoPolyData(MeshC, ShellPolyData);
			m_DataStorage->GetNamedObject<mitk::Surface>(nodeNameB)->ReleaseData();
			m_DataStorage->GetNamedObject<mitk::Surface>(nodeNameB)->SetVtkPolyData(ShellPolyData);
		}
	}
}

void lancetAlgorithm::IntraOsteotomy::CalculateBoolean(std::string nodeName, MR::Mesh& MeshA,
	MR::Mesh& MeshB, MR::BooleanOperation operationOne, MR::BooleanOperation operationTwo, int compareSize)
{
	if (m_DataStorage->GetNamedObject<mitk::Surface>(nodeName)->GetVtkPolyData()->GetPoints()->GetNumberOfPoints() <= 0)
		return;

	auto intersectResult = (MR::boolean(MeshA, MeshB, operationOne));

	MR::Mesh intersectMesh = *intersectResult;
	std::cout << "CalculateBoolean:" << intersectMesh.points.vec_.size() << std::endl;
	if (intersectMesh.points.vec_.size() > 0)
	{
		auto diffResult = (MR::boolean(MeshA, MeshB, operationTwo));

		MR::Mesh diffMesh = *diffResult;
		std::cout << "diffMesh points: " << diffMesh.points.vec_.size() << " MeshA points: " << MeshA.points.vec_.size() << std::endl;
		if (diffMesh.points.vec_.size() <= compareSize * MeshA.points.vec_.size())
		{
			std::cout << "diffMesh.points.vec_.size() <= " << compareSize << " * MeshA.points.vec_.size()" << std::endl;
			MeshA = diffMesh;
			MeshA.invalidateCaches();
			MeshB.invalidateCaches();
			vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
			TurnMRMeshIntoPolyData(MeshA, polyData);
			m_DataStorage->GetNamedObject<mitk::Surface>(nodeName)->ReleaseData();
			m_DataStorage->GetNamedObject<mitk::Surface>(nodeName)->SetVtkPolyData(polyData);
		}
	}
}

void lancetAlgorithm::IntraOsteotomy::TurnMRMeshIntoPolyData(MR::Mesh MRMesh, vtkSmartPointer<vtkPolyData> PolyData)
{
	clock_t t_start = clock();

	// all vertices of valid triangles
	const std::vector<std::array<MR::VertId, 3>> triangles = MRMesh.topology.getAllTriVerts();

	size_t cellNum = triangles.size();

	// all point coordinates
	const std::vector<MR::Vector3f>& mrPoints = MRMesh.points.vec_;

	const float* pointsXYZtripples = reinterpret_cast<const float*>(mrPoints.data());

	size_t ptNum = mrPoints.size();

	// triangle vertices as tripples of ints (pointing to elements in points vector)
	const int* vertexIDtripples = reinterpret_cast<const int*>(triangles.data());

	// Step 1: Create points
	vtkNew<vtkPoints> vtkPoints;
	vtkPoints->SetNumberOfPoints(ptNum);

	for (int i{ 0 }; i < ptNum; i++)
	{
		vtkPoints->SetPoint(i, pointsXYZtripples[3 * i], pointsXYZtripples[3 * i + 1], pointsXYZtripples[3 * i + 2]); // fastest
	}

	// faster than above
	vtkNew<vtkCellArray> vtkTriangles;
	vtkNew<vtkIdTypeArray> connectivity;

	// Each triangle has 4 entries: 3 point ids and 1 entry for the number of points in the cell (3)
	connectivity->SetNumberOfValues(cellNum * 4);
	vtkIdType* ptr = connectivity->GetPointer(0);

	for (int i{ 0 }; i < cellNum; i++)
	{
		*ptr++ = 3; // Number of points in this cell
		*ptr++ = vertexIDtripples[i * 3];
		*ptr++ = vertexIDtripples[i * 3 + 1];
		*ptr++ = vertexIDtripples[i * 3 + 2];
	}

	vtkTriangles->SetCells(cellNum, connectivity);

	//Create a polydata object and add the points and triangles to it
	vtkNew<vtkPolyData> newPolyData;
	newPolyData->SetPoints(vtkPoints);
	newPolyData->SetPolys(vtkTriangles);

	vtkNew<vtkPolyDataNormals> normals;
	normals->SetInputData(newPolyData);
	normals->Update();

	PolyData->DeepCopy(normals->GetOutput());

	clock_t t_end = clock();

	std::cout<<"TurnMRMeshIntoPolyData cost: " + QString::number(t_end - t_start) + " ms"<<std::endl;
}

void lancetAlgorithm::IntraOsteotomy::SaveIntialNodePolyDataWithTransform(std::string nodeName,
	std::string path, vtkTransform* transform)
{
	auto node =m_DataStorage->GetNamedNode(nodeName);
	if (!node)
	{
		std::cout << "Cannot find node: " << nodeName << std::endl;
		return;
	}
	vtkSmartPointer<vtkPolyData> transformedPolyData = vtkSmartPointer<vtkPolyData>::New();
	auto surface = dynamic_cast<mitk::Surface*>(node->GetData());
	auto polyData = surface->GetVtkPolyData();
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->DeepCopy(surface->GetGeometry()->GetVtkMatrix());
	transformedPolyData->DeepCopy(CalculationHelper::TransformPolyData(polyData, matrix));

	if (transform)
	{
		vtkSmartPointer<vtkPolyData> finalTransformedPolyData = vtkSmartPointer<vtkPolyData>::New();
		finalTransformedPolyData->DeepCopy(CalculationHelper::TransformPolyData(transformedPolyData, transform));
		FileIO::WritePolyDataAsSTL(finalTransformedPolyData, path);
		return;
	}
	FileIO::WritePolyDataAsSTL(transformedPolyData, path);
}

void lancetAlgorithm::IntraOsteotomy::Drill(CutPlane aCutPlane)
{
	if (!ValidateRequiredNodes("green", "buffer", "shell", "red", "DrillEndTip"))
	{
		MITK_WARN << "Cutting initialization is not ready";
		return;
	}

	clock_t start = clock();

	//------------ Move m_Cutter_mesh with geometry obtained from the cutter surface ----------------
	auto cutterMatrix = m_DataStorage->GetNamedNode(PKAData::m_DrillEndTipNodeName.toStdString())
		->GetData()->GetGeometry()->GetVtkMatrix();

	double matrixDiff{ 0 };

	for (int i{ 0 }; i < 4; i++)
	{
		for (int j{ 3 }; j < 4; j++)
		{
			if (abs(m_LastRoundMatrix->GetElement(i, j) - cutterMatrix->GetElement(i, j)) > matrixDiff)
			{
				matrixDiff = abs(m_LastRoundMatrix->GetElement(i, j) - cutterMatrix->GetElement(i, j));
			}
		}
	}

	m_LastRoundMatrix->DeepCopy(cutterMatrix);
	if (matrixDiff < 0.01)
	{
		clock_t end = clock();
		std::cout << "Cutting time: " << end - start << std::endl;
		return;
	}

	MR::Vector3f x(cutterMatrix->GetElement(0, 0), cutterMatrix->GetElement(1, 0), cutterMatrix->GetElement(2, 0));
	MR::Vector3f y(cutterMatrix->GetElement(0, 1), cutterMatrix->GetElement(1, 1), cutterMatrix->GetElement(2, 1));
	MR::Vector3f z(cutterMatrix->GetElement(0, 2), cutterMatrix->GetElement(1, 2), cutterMatrix->GetElement(2, 2));
	MR::Vector3f t(cutterMatrix->GetElement(0, 3), cutterMatrix->GetElement(1, 3), cutterMatrix->GetElement(2, 3));
	MR::Matrix3f r(x, y, z);
	MR::AffineXf3f T(r, t);

	m_DrillTipMesh.transform(T);

	//------------ MeshLib Boolean -------------------
	//------------ Green -----------
	CalculateBoolean("green", m_GreenMesh, m_DrillTipMesh, MR::BooleanOperation::Intersection, MR::BooleanOperation::DifferenceAB, 2);

	//------------ buffer -----------
	CalculateBoolean("buffer", m_BufferMesh, m_DrillTipMesh, MR::BooleanOperation::Intersection, MR::BooleanOperation::DifferenceAB, 2);

	//------------- Shell and Red should be considered together ----------------------
	CalculateBoolean("red", m_RedMesh, m_DrillTipMesh, MR::BooleanOperation::Intersection, MR::BooleanOperation::DifferenceAB, 5);
	CalculateBoolean("red", m_RedMesh, m_DrillTipMesh, MR::BooleanOperation::Intersection,
		"shell", m_ShellMesh, m_DrillTipMesh, MR::BooleanOperation::InsideA, MR::BooleanOperation::OutsideA);
	//------------- Move the cutter back to initial --------------
	MR::AffineXf3f T_invert = T.inverse();
	m_DrillTipMesh.transform(T_invert);

	clock_t end = clock();
	std::cout << "Cutting time: " << end - start << std::endl;
	IsDrillInSecurityBoundary(aCutPlane);
	IsDrillInSecurityDepth(aCutPlane);
}

double lancetAlgorithm::IntraOsteotomy::GetDrillTip2FemurDrillPlaneDistance(Eigen::Vector3d planePoint, Eigen::Vector3d planeNormal, vtkMatrix4x4* TFemur2Tibia)
{
	Eigen::Vector3d tipPos = m_PKADianaAimHardwareDevice->GetDrillEndInCamera();
	vtkSmartPointer<vtkMatrix4x4> TCamera2Drill = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TCamera2DrillTip = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2Drill->DeepCopy(PKAData::m_TCamera2Drill);
	auto drillTipRotation = CalculationHelper::GetRotationFromMatrix4x4(TCamera2Drill);
	TCamera2DrillTip->DeepCopy(CalculationHelper::GetMatrixByRotationAndTranslation(drillTipRotation, tipPos));

	vtkSmartPointer<vtkMatrix4x4> TFemur2DrillTip = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TFemur2FemurRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TFemurRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TFemur2FemurRF->DeepCopy(PKAData::m_TFemurRF2FemurImage);
	TFemur2FemurRF->Invert();
	TFemurRF2Camera->DeepCopy(PKAData::m_TCamera2FemurRF);
	TFemurRF2Camera->Invert();

	TFemur2DrillTip->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TFemur2FemurRF, TFemurRF2Camera, TCamera2DrillTip));
	//转换到股骨图像坐标系下
	Eigen::Vector3d tipPosInFemurImage = CalculationHelper::GetTranslationFromMatrix4x4(TFemur2DrillTip);
	Eigen::Vector3d tipPosDirectionInFemurImage = CalculationHelper::GetXAxisFromVTKMatrix(TFemur2DrillTip);
	Eigen::Vector3d planePointInFemur = planePoint;
	Eigen::Vector3d planeNormalInFemur = planeNormal;
	if (TFemur2Tibia)
	{
		planePointInFemur = CalculationHelper::TransformByMatrix(planePointInFemur, TFemur2Tibia);
		planeNormalInFemur = CalculationHelper::CalculateInCoordinate(planeNormalInFemur, TFemur2Tibia);
	}
	double distance = -CalculationHelper::DistanceFromPointToPlane(tipPosInFemurImage, planeNormalInFemur, planePointInFemur);

	return distance;
}

double lancetAlgorithm::IntraOsteotomy::GetDrillTip2TibiaDrillPlaneDistance(Eigen::Vector3d planePoint, Eigen::Vector3d planeNormal)
{
	Eigen::Vector3d tipPos = m_PKADianaAimHardwareDevice->GetDrillEndInCamera();
	vtkSmartPointer<vtkMatrix4x4> TCamera2Drill = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TCamera2DrillTip = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2Drill->DeepCopy(PKAData::m_TCamera2Drill);
	auto drillTipRotation = CalculationHelper::GetRotationFromMatrix4x4(TCamera2Drill);
	TCamera2DrillTip->DeepCopy(CalculationHelper::GetMatrixByRotationAndTranslation(drillTipRotation, tipPos));

	vtkSmartPointer<vtkMatrix4x4> TFemur2DrillTip = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TTibia2TibiaRF = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TTibiaRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TTibia2TibiaRF->DeepCopy(PKAData::m_TTibiaRF2TibiaImage);
	TTibia2TibiaRF->Invert();
	TTibiaRF2Camera->DeepCopy(PKAData::m_TCamera2TibiaRF);
	TTibiaRF2Camera->Invert();

	TFemur2DrillTip->DeepCopy(CalculationHelper::PreConcatenateMatrixs(TTibia2TibiaRF, TTibiaRF2Camera, TCamera2DrillTip));
	//转换到股骨图像坐标系下
	Eigen::Vector3d tipPosInTibiaImage = CalculationHelper::GetTranslationFromMatrix4x4(TFemur2DrillTip);
	Eigen::Vector3d tipPosDirectionInTibiaImage = CalculationHelper::GetXAxisFromVTKMatrix(TFemur2DrillTip);
	Eigen::Vector3d planePointInFemur = planePoint;
	Eigen::Vector3d planeNormalInFemur = planeNormal;

	double distance = -CalculationHelper::DistanceFromPointToPlane(tipPosInTibiaImage, planeNormalInFemur, planePointInFemur);

	return distance;
}

bool lancetAlgorithm::IntraOsteotomy::IsDrillInSecurityBoundary(CutPlane aCutPlane)
{
	Eigen::Vector3d planeNormal;
	Eigen::Vector3d planePoint;
	auto drillEndMatrix = m_DataStorage->GetNamedNode(PKAData::m_DrillEndTipNodeName.toStdString())
		->GetData()->GetGeometry()->GetVtkMatrix()->GetData();;
	Eigen::Vector3d drillPos(drillEndMatrix[3], drillEndMatrix[7], drillEndMatrix[11]);
	auto securityBoundaryData = m_DataStorage->GetNamedNode("SecurityBoundary")->GetData();
	auto cutPlanePolyData = dynamic_cast<mitk::Surface*>(m_DataStorage->GetNamedNode("SecurityBoundary")->GetData())->GetVtkPolyData();
	vtkSmartPointer<vtkTransformPolyDataFilter> transFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	if (!cutPlanePolyData)
	{
		std::cout << "cutPlane is nullptr" << std::endl;
		return false;
	}
	auto securityBoundaryMatrix = securityBoundaryData->GetGeometry()->GetVtkMatrix();
	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->SetMatrix(securityBoundaryMatrix);
	transFilter->SetInputData(cutPlanePolyData);
	transFilter->SetTransform(trans);
	transFilter->Update();
	vtkSmartPointer<vtkPolyData> transformedPolyData = vtkSmartPointer<vtkPolyData>::New();
	transformedPolyData->DeepCopy(transFilter->GetOutput());
	vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
	cellLocator->SetDataSet(transformedPolyData);
	cellLocator->BuildLocator();

	switch (aCutPlane)
	{
	case lancetAlgorithm::CutPlane::DistalCut:
	{
		planePoint = m_ChunLiXGImplant->GetDistalCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetDistalCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::PosteriorCut:
	{
		planePoint = m_ChunLiXGImplant->GetPosteriorCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetPosteriorCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::PosteriorChamferCut:
	{
		planePoint = m_ChunLiXGImplant->GetPosteriorChamferCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetPosteriorChamferCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::ProximalCut:
	{
		planePoint = m_ChunLiTray->GetProximal();
		planeNormal = m_ChunLiTray->GetProximalDirection();
		break;
	}
	default:
		return false;
	}
	Eigen::Vector3d projectedPoint = CalculationHelper::ProjectPointOntoPlane(drillPos, planePoint, planeNormal);

	PrintDataHelper::CoutArray(projectedPoint, "projectedPoint");

	double closestPoint[3];  // 最近点
	double dist2;             // 距离平方
	vtkIdType cellId;         // 单元 ID
	int subId;                // 子单元 ID
	cellLocator->FindClosestPoint(projectedPoint.data(), closestPoint, cellId, subId, dist2);
	const double epsilon = 1e-2;
	if (dist2 < epsilon) {
		std::cout <<std::setprecision(6) << "IsDrillInSecurityBoundary True dist2: "<< dist2 << std::endl;
		PrintDataHelper::CoutArray(closestPoint, 3, "closestPoint");
		return true;
	}
	else {
		std::cout << std::setprecision(6) << "IsDrillInSecurityBoundary True dist2: " << dist2 << std::endl;
		PrintDataHelper::CoutArray(closestPoint, 3, "closestPoint");
		return false;
	}
}

bool lancetAlgorithm::IntraOsteotomy::IsDrillInSecurityDepth(CutPlane aCutPlane)
{
	Eigen::Vector3d planeNormal;
	Eigen::Vector3d planePoint;
	auto drillEndMatrix = m_DataStorage->GetNamedNode(PKAData::m_DrillEndTipNodeName.toStdString())
		->GetData()->GetGeometry()->GetVtkMatrix()->GetData();;
	Eigen::Vector3d drillPos(drillEndMatrix[3], drillEndMatrix[7], drillEndMatrix[11]);
	switch (aCutPlane)
	{
	case lancetAlgorithm::CutPlane::DistalCut:
	{
		planePoint = m_ChunLiXGImplant->GetDistalCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetDistalCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::PosteriorCut:
	{
		planePoint = m_ChunLiXGImplant->GetPosteriorCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetPosteriorCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::PosteriorChamferCut:
	{
		planePoint = m_ChunLiXGImplant->GetPosteriorChamferCut();
		planeNormal = planePoint - m_ChunLiXGImplant->GetPosteriorChamferCutNormal();
		break;
	}
	case lancetAlgorithm::CutPlane::ProximalCut:
	{
		planePoint = m_ChunLiTray->GetProximal();
		planeNormal = m_ChunLiTray->GetProximalDirection();
		break;
	}
	default:
		return false;
	}

	double normalLength = planeNormal.norm();
	double distance = CalculationHelper::DistanceFromPointToPlane(drillPos, planeNormal, planePoint);
	if (distance < -1.0)
	{
		std::cout << "IsDrillInSecurityDepth FALSE: "<<distance << std::endl;
		return false;
	}
	else
	{
		std::cout << "IsDrillInSecurityDepth TRUE: " << distance << std::endl;
		return true;
	}
}

void lancetAlgorithm::IntraOsteotomy::InitalOsteotomyModel(std::string drillEndName, std::string prosNodeName, std::string boneNodeName)
{
		if (!ValidateRequiredNodes(drillEndName, prosNodeName, boneNodeName))
		{
			std::cout << "DrillEndTip, pros, bone is missing" << std::endl;
			return;
		}

		PKARenderHelper::RemoveNodeIsExist(m_DataStorage, "red");
		PKARenderHelper::RemoveNodeIsExist(m_DataStorage, "buffer");
		PKARenderHelper::RemoveNodeIsExist(m_DataStorage, "green");
		PKARenderHelper::RemoveNodeIsExist(m_DataStorage, "shell");

		vtkSmartPointer<vtkPolyData> transformedPolyData = vtkSmartPointer<vtkPolyData>::New();
		std::string drillTipFilePath = m_DesktopPKAIntraOsteotomyFilePath + drillEndName + ".stl";
		std::string prosFilePath = m_DesktopPKAIntraOsteotomyFilePath + prosNodeName + ".stl";
		std::string prosPlusPath = m_DesktopPKAIntraOsteotomyFilePath + prosNodeName + "Plus.stl";
		std::string bonePath = m_DesktopPKAIntraOsteotomyFilePath + boneNodeName + ".stl";

		//保存磨钻头
		auto drillEndTipSurface = dynamic_cast<mitk::Surface*>(m_DataStorage->GetNamedNode(drillEndName)->GetData());
		auto drillEndTipPolyData = drillEndTipSurface->GetVtkPolyData();
		FileIO::WritePolyDataAsSTL(drillEndTipPolyData, drillTipFilePath);

		//保存假体
		SaveIntialNodePolyDataWithTransform(prosNodeName, prosFilePath);

		//保存放大后的假体
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Scale(1.1, 1.1, 1.1);
		transform->Update();
		SaveIntialNodePolyDataWithTransform(prosNodeName, prosPlusPath, transform);

		//保存股骨模型
		SaveIntialNodePolyDataWithTransform(boneNodeName, bonePath);

		//读取为MR::Mesh
		MR::Mesh boneMesh = MR::MeshLoad::fromAnySupportedFormat(bonePath).value();
		MR::Mesh prosMesh = MR::MeshLoad::fromAnySupportedFormat(prosFilePath).value();
		MR::Mesh prosPlusMesh = MR::MeshLoad::fromAnySupportedFormat(prosPlusPath).value();
		m_DrillTipMesh = MR::MeshLoad::fromAnySupportedFormat(drillTipFilePath).value();

		//预处理
		m_GreenMesh = *(MR::boolean(boneMesh, prosMesh, MR::BooleanOperation::Intersection)); //骨头与假体相交部分
		m_RedMesh = *(MR::boolean(boneMesh, prosPlusMesh, MR::BooleanOperation::DifferenceAB)); //A-B
		m_ShellMesh = *(MR::boolean(boneMesh, prosPlusMesh, MR::BooleanOperation::OutsideA));   //骨头与外层模型  相交 不包括骨头的部分
		MR::Mesh tmp_mesh = *(MR::boolean(boneMesh, prosPlusMesh, MR::BooleanOperation::Intersection)); //骨头与外层模型相交的部分
		m_BufferMesh = *(MR::boolean(tmp_mesh, prosMesh, MR::BooleanOperation::DifferenceAB));   //A-B

		vtkSmartPointer<vtkPolyData> greenPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPolyData> redPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPolyData> tmpShellPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPolyData> shellPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPolyData> bufferPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPolyData> drillEndPolyData = vtkSmartPointer<vtkPolyData>::New();

		TurnMRMeshIntoPolyData(m_GreenMesh, greenPolyData);
		TurnMRMeshIntoPolyData(m_RedMesh, redPolyData);
		TurnMRMeshIntoPolyData(m_ShellMesh, tmpShellPolyData);
		TurnMRMeshIntoPolyData(m_DrillTipMesh, drillEndPolyData);

		vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
		normals->SetInputData(tmpShellPolyData);
		normals->SplittingOff();
		normals->Update();

		// Warp using the normals
		vtkSmartPointer<vtkWarpVector> warper = vtkSmartPointer<vtkWarpVector>::New();
		warper->SetInputData(normals->GetOutput());
		warper->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS,
			vtkDataSetAttributes::NORMALS);
		warper->SetScaleFactor(0.07);
		warper->Update();

		vtkNew<vtkPolyDataNormals> normals_;
		normals_->SetInputData(warper->GetPolyDataOutput());
		normals_->Update();
		shellPolyData->DeepCopy(normals->GetOutput());
		TurnMRMeshIntoPolyData(m_BufferMesh, bufferPolyData);
		
		PKARenderHelper::DisplaySingleNode(m_DataStorage, drillEndName);
		auto redNode = PKARenderHelper::AddPolyData2DataStorage(m_DataStorage, "red", redPolyData, 1, 0, 0);
		auto bufferNode = PKARenderHelper::AddPolyData2DataStorage(m_DataStorage, "buffer", bufferPolyData);
		auto greenNode = PKARenderHelper::AddPolyData2DataStorage(m_DataStorage, "green", greenPolyData, 0, 1, 0);
		auto shellNode = PKARenderHelper::AddPolyData2DataStorage(m_DataStorage, "shell", shellPolyData, 1, 1, 1, redNode);

		greenNode->SetIntProperty("layer", 5);
		bufferNode->SetIntProperty("layer", 4);
		shellNode->SetIntProperty("layer", 3);
		redNode->SetIntProperty("layer", 2);
}
