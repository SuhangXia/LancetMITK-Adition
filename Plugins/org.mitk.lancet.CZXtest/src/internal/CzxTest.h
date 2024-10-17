/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef CzxTest_h
#define CzxTest_h

#include <iostream>
#include <queue>
#include <berryISelectionListener.h>
#include <random>
#include <kukaRobotDevice.h>
#include <filesystem>
#include <regex>
// Qt
#include <QmitkAbstractView.h>
#include <QMessageBox>
#include <QtWidgets\qfiledialog.h>
#include <QProgressBar>
#include <QtConcurrent>
#include <QmitkSingleNodeSelectionWidget.h>
#include <QAction>
#include <QShortcut>
#include <QInputDialog>
#include <QmitkPointListWidget.h>
#include <QmitkRenderWindow.h>
#include <QFuture>
#include <QButtonGroup>
//vtk
#include <vtkQuaternion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
#include <vtkCubeSource.h>
#include <vtkLineSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAxesActor.h>
#include <vtkAppendPolyData.h>
#include <vtkTubeFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkDataSet.h>
#include <vtkCharArray.h>
#include <vtkPointData.h>
#include <vtkPlaneSource.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPropCollection.h>
#include <vtkRendererCollection.h>
#include <vtkSphereSource.h>
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkSTLWriter.h>
#include <vtkPolyDataNormals.h>
#include <vtkWarpVector.h>
#include <vtkDelaunay3D.h>
#include <vtkSphereSource.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkClipPolyData.h>
#include <vtkBox.h>
#include <vtkClipClosedSurface.h>
#include <vtkPlanes.h>
#include <vtkPlaneCollection.h>

#include <itkSpatialOrientation.h>
// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkTransferFunction.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <mitkTrackingDeviceSource.h>
#include <mitkNavigationToolStorageDeserializer.h>
#include <mitkNavigationToolStorageSerializer.h>
#include <mitkIGTIOException.h>
#include <mitkAffineTransform3D.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateProperty.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateDataType.h>
#include <mitkInteractionEventObserver.h>
#include <mitkInteractionPositionEvent.h>
#include <mitkInteractionEvent.h>
#include <mitkMousePressEvent.h>
#include <mitkInteractionEventHandler.h>
#include <mitkIRenderWindowPartListener.h>
#include <mitkRotationOperation.h>
#include <mitkInteractionConst.h>
#include <mitkGeometryData.h>
#include <mitkIOUtil.h>
#include <mitkLevelWindowProperty.h>
#include <mitkSceneIO.h>
#include <mitkPointSetVtkMapper3D.h>
#include <mitkBoundingShapeInteractor.h>
#include <mitkNodePredicateFunction.h>
#include <mitkSliceNavigationController.h>
//meshlib
#include <MRMesh/MRMesh.h>
#include <MRMesh/MRMeshLoad.h>
#include <MRMesh/MRUVSphere.h>
#include <MRMesh/MRMeshBoolean.h>
#include <MRMesh/MRMeshSave.h>
#include <MRMesh/MRMeshDecimate.h>

//lancet
#include <lancetVegaTrackingDevice.h>
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetTrackingDeviceSourceConfigurator.h"

#include "robotRegistration.h"
#include "ui_CzxTestControls.h"
#include "PrintDataHelper.h"
#include "CalculationHelper.h"
#include "PKAData.h"
#include "mitkGizmo.h"
#include "ChunLiXGImplant.h"
#include "ChunLiXKImplant.h"
#include "PKAEnum.h"
#include "PKARenderHelper.h"
#include "PKAPrePlanBoneModel.h"
#include "JsonSerailizeHelper.h"
#include "AngleCalculationHelper.h"
#include "FileIO.h"
#include "PKAHardwareDevice.h"
#include "PKADianaAimHardwareDevice.h"
#include "ChunLITray.h"
#include "ModelRegistration.h"
#include "ToolDisplayHelper.h"
#include "IntraOsteotomy.h"
#include "PreoPreparation.h"
#include "JointPartDescription.h"
#include "RobotJoint.h"
#include "RobotBase.h"
#include "RobotFrame.h"
#include "BoundingBoxInteraction.h"
#include "RobotFrame2.h"

//#include "mitkGizmoInteractor.h"
//#include "ObserveTransCommand.h"
//#include "ModelUserControl.h"
//#include "ModelInteractor.h"

/**
  \brief CzxTest

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
using namespace lancetAlgorithm;

class CzxTest : public QmitkAbstractView,public mitk::IRenderWindowPartListener
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;
  void RenderWindowPartActivated(mitk::IRenderWindowPart* renderWindowPart) override;
  void RenderWindowPartDeactivated(mitk::IRenderWindowPart* renderWindowPart) override;
  //Robot And NDI
  void ConnectRobotBtnClicked();
  void ConnectCameraClicked();
  void UpdateCameraBtnClicked();
  void PowerOffBtnClicked();
  void PowerOnBtnClicked();
  void SelfCheckBtnClicked();

 //Robot Movement
  bool Translate(const double axis[3]);
  bool Rotate(const double axis[3]);
  void RobotAutoRegistationBtnClicked();
  void automoveBtnClicked();
  void StopRobotMoveBtnClicked();
  void ClearRobotErrorInfoBtnClicked();
  //RobotRegistration
  void SetTcpToFlangeBtnClicked();
  void RecordInitPosBtnClicked();
  void GoToInitPosBtnClicked();
  void CaptureRobotBtnClicked();
  void ResetRobotRegistrationBtnClicked();
  void SaveRobotRegistrationBtnClicked();
  void ReuseRobotRegistationBtnClicked();
  void ReadRobotJointAnglesBtnClicked();
  void SetRobotJointAnglesBtnClicked();
  void GetRobotJointsLimitBtnClicked();
  void SetRobotPositionModeBtnClicked();
  void SetRobotJointsImpedanceModelBtnClicked();
  void SetRobotCartImpedanceModeBtnClicked();

  //TCP Registration
  void GetProbeEndPos(TCPRegistraionMethod method);
  void SetDrillEndTCPBtnClicked();
  void DisplayCameraToTCPAxesBtnClicked();
  void SetTCPByProbeTipBtnClicked();
  void ReadRobotImpedaBtnClicked();
  void SetRobotImpedaBtnClicked();

  //Image Registation Table
  void SaveImageRegistrationResultBtnClicked();
  void RegisterMalleolus(MalleolusPoint malleolusPoint);
  void SelectRegistrationPointBtnClicked();
  void RemoveLastRegistrationPointBtnClicked();
  void ClearRegistrationPointBtnClicked();
  void OnMitkNodeSelectWidget_landmark_srcChanged();
  void RegisterImageBtnClicked();
  void StartSelectPointBtnClicked();
  void UpdateRegistrationPointState(int index);
  void VerifyImageRegistrationAccuracyBtnClicked();
  void SaveImageRegistraionResultBtnClicked();
  void ReuseImageRegistrationBtnClicked();
  void OnPKAReUseRegistrationPointsSelectWidgetChanged();
  void UseRegistrationBtnClicked();
  void CapturVerifyBtnClicked();
  void ClearVerifyPointBtnClicked();
  void VerifyPointBtnClicked();
  /// <summary>
  /// 随机产生100个点并进行过滤，UI用进度条展示
  /// </summary>
  void LocateHipCenterBtnClicked();

  //计算追踪点到追踪阵列的方向
  double VerifyKneeModelAccuracy(KneeModel modelType);
  void DrillEndAccuracyVerifyBtnClicked();
  //Test
  void ChangeModelOpacityBtnClicked(const char* modelName, int value);
  void SetLandmarkPointBtnClicked();
  void PrintNode();
  void OnAddPointSetClicked();
  void OnCurrentSelectionChanged(QmitkSingleNodeSelectionWidget::NodeList /*nodes*/);
  void CalcualtePointSetCenterBtnClicked();
  void RotateTibiaByCenterBtnClicked();
  void ChangeModelColorBtnClicked();
  void DisplayPKADrillBtnClicked();
  void DisplayProbeBtnClicked();
  void DisplayBluntProbeBtnClicked();

  //PrePlan
  double m_TranslateLength = 5;
  double m_RotateAngle = 10;
  ViewType m_PrePlanViewType = ViewType::Axial;
  void OnPrePlanViewTypeChanged(int index);

  void OnTranslateModelButtonClicked(Direction direction);
  void OnRotateModelButtonClicked(bool up);
  void OnPrePlanSelectImageWidgetSelectChanged(QmitkSingleNodeSelectionWidget::NodeList);
  void DisplayHansRobotModelBtnClicked();
  void DisplayDiania7RobotBtnClicked();
  void DisplayDianaRobotV2BtnClicked();
  void DisplayDiana7RobotWithOfficialDH();
  void GetCurrentNode2DActorBtnClicked();
  void RotateModelAndSaveBtnClicked();
  void CalculateJointToLinkMatrix(vtkMatrix4x4* TImage2PreJoint, vtkMatrix4x4* TImage2Joint, vtkMatrix4x4* TPreJoint2Joint);
  void SaveRobotJoint2LinkBtnClicked();
  void ConnectJointControl(QPushButton* plusBtn, QPushButton* minusBtn, QLineEdit* lineEdit, int jointIndex, RobotFrame* robotFrame);
  void InitRobotSimulation();
  void RenderBtnClicked();
  RobotFrame* m_RobotFrame;
  RobotFrame2* m_RobotFrame2;
  std::vector<QLineEdit*> m_RobotJointsAngleLineEdit;
  //vtkSmartPointer<vtkAxesActor> TImage2Base;
  std::vector<vtkSmartPointer<vtkAxesActor>> JointsAxesActor;
  void OnPrePointSelect(PrePointCaptureType t);
  void PreCapturePointBtnClicked();

  void CalculateAxisBtnClicked();
  void UseGizmoBtnClicked();
  void OnBrandComboBoxShowed();
  void OnBrandComboBoxChanged();
  void OnProsthesisTypeComboxBoxChanged();
  void OnProsthesisKindComboBoxChanged();

  void ApplyProsthesisBtnClicked();
  void InitFemurImplantPos();
  void InitTibiaTrayPos();
  
  void CalculateProsBtnClicked();
  void ChangeFemurAngleUIDisplay(KneeModel kneeModel);
  void ApplyTiltAngleBtnClicked();
  void ApplyRotationAngleBtnClicked();
  void ApplyVaAngleBtnClicked();
  void ApplyProudDistanceBtnClicked();
  void MoveModelByStepBtnClicked(bool isPlus);
  void OnAngleCalculationTypeComboBoxChanged();
  void UpdateAngle(KneeModel kneeModel);

  QString AdaptBoundingObjectName(const QString& name) const;

  void SelectFemurClippedPlanePointBtnClicked();
  void CutFemurClippedPlaneBtnClicked();
  void SelectTibiaClippedPlanePointBtnClicked();
  void CutTibiaClippedPlaneBtnClicked();
  //tab intra plan
  ViewType m_IntraPlanViewType = ViewType::Axial;
  void StartIntraBtnClicked();
  void SaveIntraDataBtnClicked();
  bool isStartIntra = false;
  void OnIntraPlanViewTypeChanged(int index);
  void TranslateIntraPros(Direction direction);
  void RotatateIntraPros(bool up);
  void IntraVaAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel);
  void IntraTiltAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel);
  void IntraRotationAngleChangeBtnClicked(bool isPlus, KneeModel kneeModel);
  void IntraProudChangeBtnClicked(bool isPlus, KneeModel kneeModel);
  void UpdateIntraAngleUIDisplay();

  //tab intra Osteotomy
  void InitalOsteotomyModelBtnClicked(std::string drillEndName, std::string prosNodeName, std::string boneNodeName);
  void AddModelInterSection(std::string boneNodeName, std::string prosNodeName, std::string intersectionNodeName);
  void AddFemurBoneModelInterSectionBtnClicked();
  void AddTibiaBoneModelInterSectionBtnClicked();
  void Drill();
  vtkSmartPointer<vtkAxesActor> m_DrillEndTipAxesActor;
  void RotateIntraDrillEndTipAndUpDateAxes(Eigen::Vector3d direction, mitk::DataNode* node, double angle);
  void AddDrillEndTipModelBtnClicked();
  void GoToPoteriorInitialPosBtnClicked();
  void GoToDistalInitialPosBtnClicked();
  void GoToPosteriorChamferInitialPosBtnClicked();
  void GoToTibiaProximalInitialPosBtnClicked();
  void RobotMoveToFemurDistalBtnClicked();
  void RobotMoveToFemurPosteriorChamferBtnClicked();
  void RobotMoveToFemurPosteriorBtnClicked();
  void RobotMoveToTibiaProximalBtnClicked();

  //tab
  void SwitchNodeNameBtnClicked();
  void ReadConfigBtnClicked();
  void WriteJsonBtnClicked();
  void ReadJsonBtnClicked();
  void SelectJsonPathBtnClicked();
  void WriteTuoJsonBtnClicked();
  void GenerativeImplantSecurityBoundaryBtnClicked();
  void GenerativeTraySecurityBoundaryBtnClicked();
  //tab
  void VerifyProbeBtn();
public slots:
  void OnRotateTibia();
  void UpdateBoneIntraDisplay();
  void OnBoxModified(vtkObject* caller, long unsigned int evetnId, void* clientData, void* callData);
  void UpdateTCPAxesActor();
  //void OnWindowLevelChanged(double value);
  //void OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/, const QList<mitk::DataNode::Pointer>& nodes);

private:
    //UI
    void InitUI();
    void InitHardwareDeviceTabConnection();
    void InitRobotRegistrationTabConnection();
    void InitTCPRegistrationTabConnection();
    void InitIntraRobotMoveConnection();
    void InitPreplanTabConnection();
    void InitIntraPlanTabConnection();
    void InitModelRegistrationTabConnection();
    void InitModelDispalyTabConnection();
    void InitMakeProsthesisConfigTabConnection();
    void InitAccuracyVerifyTabConnection();
    void InitIntraOsteotomyTabConnection();
    void InitDianaRobotTabConnection();
    //Global variable
    void InitGlobalVariable();

    void LoadMITKFile(std::string filePath);

    void DisplayAuxiliaryModel();
    void TransformAuxiliaryModel(vtkMatrix4x4* matrix);
    Eigen::Vector3d GetTranslateMovementByViewType(ViewType viewType, Direction direction);
    Eigen::Vector3d GetRotationDirectionByViewType(ViewType viewType, bool isUp);

    template<typename... Args>
    bool ValidateRequiredNodes(const std::string& nodeName1, const Args&... args)
    {
        if (!this->GetDataStorage()->GetNamedNode(nodeName1))
        {
            return false;
        }

        // Recursively check the remaining nodes
        if constexpr (sizeof...(args) > 0)
        {
            return ValidateRequiredNodes(args...);
        }

        return true;
    }
    void OnTabChanged(int aIndex);
private slots:
        void RenderViewAndWidget();
        void Render3D();
        void DrawPlaneBtnClicked();
private:
  Ui::CzxTestControls m_Controls;
  mitk::BoundingShapeInteractor::Pointer m_BoundingShapeInteractor;
  PKAKukaVegaHardwareDevice* m_PKAHardwareDevice;
  IntraOsteotomy* m_IntraOsteotomy;
  QTimer* m_RequestRenderTimer{ nullptr };
  bool m_IsRotateTibia = false;
  QTimer* m_TibiaRotateTimer{ nullptr };
  QTimer* m_ProbeVerifyTimer{ nullptr };

  std::vector<Eigen::Vector3d> FemurPositions;
  QProgressBar* progressBar;
  
  std::vector<Eigen::Vector3d> probeEndOneVector;
  std::vector<Eigen::Vector3d> probeEndTwoVector;
  int ProbEndCountOne = 0;
  int ProbEndCountTwo = 0;

  lancet::NavigationObject::Pointer NavigatedImage;

  Eigen::Vector3d CondyleCenterPos;
  QShortcut* m_shortCut;
  BoundingBoxInteraction* m_BoundingBoxInteraction;
  PrePointCaptureType m_PrePointCaptureType = PrePointCaptureType::HipCenter;

  std::filesystem::path m_ModelPath = "E:\\PKAModelData\\CHUNLI-danke";
  vtkSmartPointer<vtkAxesActor> m_WorldAxes = nullptr;
  std::string m_CurrentModelName = "";
  ChunLiXGImplant* m_FemoralImplant = nullptr;
  ChunLiTray* m_TibiaTray = nullptr;
  int planeCount = 0;
  PKAPrePlanBoneModel* m_FemurBoneModel = nullptr;
  PKAPrePlanBoneModel* m_TibiaBoneModel = nullptr;
  AngleCalculationHelper* m_AngleCalculationHelper;
  mitk::DataNode* m_currentSelectedNode;

  QButtonGroup* m_TiltRadioBtnGroup;
  QButtonGroup* m_ProsRotationRadioBtnGroup;
  QButtonGroup* m_VaRadioBtnGroup;
  QButtonGroup* m_VerifyPointRadioBtnGroup;
  QButtonGroup* m_IntraProsGroup;
  bool isDisplayBoneAxes = false;
  KneeModel m_AngleCalculationTypeComboBoxSelectedKneeModel = KneeModel::Femur;
  PKADianaAimHardwareDevice* m_PKADianaAimHardwareDevice = nullptr;
  QTimer* m_AimoeVisualizeTimer{ nullptr };
  mitk::PointSet::Pointer m_SelectedRegistrationPoint;
  ModelRegistration* m_ModelRegistration;
  bool m_IsDisplayDrill = false;
  bool m_IsDisplayProbe = false;
  bool m_IsDisplayBluntProbe = false;
  ToolDisplayHelper* m_ToolDisplayHelper = nullptr;
  PreoPreparation* m_PreoPreparation = nullptr;
  bool m_IsCaptureFemurVerifyPoint = false;
  bool m_IsCaptureTibiaVerifyPoint = false;
  QWidget* m_ParentWidget;
  bool m_IsDisplayTCP = false;
  std::string m_TFemur2FemurImplantSavePath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TFemur2FemurImplantMatrix.txt";
  std::string m_TTibia2TibiaTraySavePath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\PKARegistrationData\\TTibia2TibiaTrayMatrix.txt";
  std::string m_DrillEndTipDirectoryPath = std::string(getenv("USERPROFILE")) + "\\Desktop\\PKAModelData\\DrillEndTip\\";

  std::vector<QLineEdit*> m_RobotJointAngleLineEdits;


  std::string m_InitCutFilePath = "E:\\PKAModelData";
  CutPlane m_IntraDrillPlane = CutPlane::DistalCut;
};

#endif // CzxTest_h
