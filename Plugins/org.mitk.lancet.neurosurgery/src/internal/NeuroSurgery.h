/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#ifndef NeuroSurgery_h
#define NeuroSurgery_h

#include <berryISelectionListener.h>
#include <berryIPreferences.h>
#include <berryIPreferencesService.h>
#include <berryIBerryPreferences.h>
#include <QmitkAbstractView.h>
#include <QmitkRegistrationJob.h>
#include <QmitkMappingJob.h>

#include "ui_NeuroSurgeryControls.h"

#include "QmitkSingleNodeSelectionWidget.h"
#include "mitkTrackingDeviceSource.h"

// itk
#include <itkImage.h>

// mitk
#include <mitkImage.h>

// MatchPoint
#include <mapDeploymentDLLInfo.h>
#include <mapDeploymentDLLHandle.h>
#include <mapRegistrationAlgorithmBase.h>
#include <mapIterativeAlgorithmInterface.h>
#include <mapMultiResRegistrationAlgorithmInterface.h>
#include <mapStoppableAlgorithmInterface.h>

class NeuroSurgery : public QmitkAbstractView
{
	Q_OBJECT

public:
	static const std::string VIEW_ID;

public slots:
	// Basic func
	void OnCheckDataClicked();
	void OnCheckPETMaskClicked();
	void OnCheckPETColorfyClicked();
	virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
		const QList<mitk::DataNode::Pointer>& nodes) override;
	void UseGizmoBtnClicked();
	void ResetGizmo();


	// Image Registration
	void OnFindAlgorithmClicked();
	void OnAlgorithmSelectionChanged();

	// Image Process
	void OnRegistrateImageClicked();
	void OnNodeSelectionChanged(QList<mitk::DataNode::Pointer> /*nodes*/);

	void OnRegResultIsAvailable(mitk::MAPRegistrationWrapper::Pointer spResultRegistration, const QmitkRegistrationJob* pRegJob);

	void OnFitMaskClicked();
	void OnReconstructMRASurfaceClicked();
	mitk::Image::Pointer ResampleMaskToImage(mitk::Image::Pointer maskImage, mitk::Image::Pointer referenceImage);
	void OnMaskPETRegionClicked();
	void OnPETResultVisualizeClicked();
	void OnLoadFibFileClicked();

	void OnRegJobError(QString err);
	void OnRegJobFinished();

	void OnMapJobError(QString err);
	void OnMapResultIsAvailable(mitk::BaseData::Pointer spMappedData,
		const QmitkMappingJob* job);

	void OnStartRegBtnPushed();
	void OnStopRegBtnPushed();
	void OnSaveLogBtnPushed();
	void OnLoadAlgorithmButtonPushed();

	void OnAlgorithmInfo(QString info);
	void OnAlgorithmStatusChanged(QString info);
	void OnAlgorithmIterated(QString info, bool hasIterationCount,
		unsigned long currentIteration);
	void OnLevelChanged(QString info, bool hasLevelCount, unsigned long currentLevel);

	// Segmentation
	void OnCheckProcessDataClicked();
	void OnLoadDataButtonClicked();
	void LoadDataToMITK(const QString& filePath, const QString& nodeName);

	void ConvertToSegmentation(mitk::DataNode::Pointer imageNode);
	void CreateSmoothedPolygonModel(mitk::DataNode::Pointer segmentationNode);

	void OnMaskToSegmentClicked();
	void OnSegmentToModelClicked();

	void SetVisibilityForChildren(mitk::DataNode::Pointer parentNode, bool visibility);
	void OnVisualizeButtonClicked();

	void SetNodeVisibilityInRenderWindow(mitk::DataNode::Pointer node, const std::string& windowName, bool visibility);

	void ApplyVisulize();


private:
	QWidget* m_Parent;

protected:
	virtual void CreateQtPartControl(QWidget* parent) override;
	virtual void SetFocus() override;

	// Parm Init
	void ParamsInit();

	// UI Funcs Connection
	void CreatQT_Basic();
	void CreateQT_ImageProcess();
	void CreateQT_PreoperativePlan();
	void CreateQT_IntraoperativePlan();
	void CreateQT_PostoperativeVerify();
	void CreateQT_AccVerifyy();

	// some initializetion func
	void InitializeAlgorithmComboBox();

	// Basic func
	void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
	void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
	void InitImageSelector(QmitkSingleNodeSelectionWidget* widget);

	std::string current = "";
	mitk::BaseData* m_baseDataToMove{ nullptr };
	mitk::DataNode* m_currentSelectedNode{ nullptr };

	// Image processing
	int m_RegistrationType = 0;
	mitk::DataNode::Pointer  m_MovingImageNode;
	mitk::DataNode::Pointer  m_FixedImageNode;

	typedef map::algorithm::facet::IterativeAlgorithmInterface IIterativeAlgorithm;
	typedef map::algorithm::facet::MultiResRegistrationAlgorithmInterface IMultiResAlgorithm;
	typedef map::algorithm::facet::StoppableAlgorithmInterface IStoppableAlgorithm;
	::map::algorithm::RegistrationAlgorithmBase::Pointer m_LoadedAlgorithm;

	mitk::DataNode::Pointer m_spSelectedTargetNode;
	mitk::DataNode::Pointer m_spSelectedMovingNode;

	mitk::BaseData::ConstPointer m_spSelectedTargetData;
	mitk::BaseData::ConstPointer m_spSelectedMovingData;

	mitk::DataNode::Pointer m_spSelectedTargetMaskNode;
	mitk::DataNode::Pointer m_spSelectedMovingMaskNode;

	mitk::Image::ConstPointer m_spSelectedTargetMaskData;
	mitk::Image::ConstPointer m_spSelectedMovingMaskData;

	bool m_CanLoadAlgorithm;
	bool m_ValidInputs;
	bool m_Working;

	::map::deployment::DLLHandle::Pointer m_LoadedDLLHandle;
	::map::deployment::DLLInfo::ConstPointer m_SelectedAlgorithmInfo;

	// Registration
	void PerformRigidRegistration(mitk::Image::Pointer movingImage, mitk::Image::Pointer fixedImage);
	void PerformAffineRegistration(mitk::Image::Pointer movingImage, mitk::Image::Pointer fixedImage);


	berry::IBerryPreferences::Pointer RetrievePreferences();

	mitk::Image::Pointer ExtractFirstFrame(const mitk::Image* dynamicImage);
	bool CheckInputs();
	void ConfigureProgressInfos();
	void ConfigureRegistrationControls();
	std::string GetDefaultRegJobName() const;
	mitk::DataStorage::SetOfObjects::Pointer GetRegNodes() const;
	const map::deployment::DLLInfo* GetSelectedAlgorithmDLL() const;
	QScopedPointer<berry::ISelectionListener> m_AlgorithmSelectionListener;

	void AdaptFolderGUIElements();

	void ConfigureNodeSelectors();

	void Error(QString msg);

	mitk::AffineTransform3D::Pointer m_OriginalTransform;

	Ui::NeuroSurgeryControls m_Controls;
};

#endif // NeuroSurgery_h