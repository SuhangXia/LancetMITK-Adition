/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>
#include <QMessageBox>
#include <QmitkAbstractView.h>
#include <mitkImage.h>
#include <QmitkAbstractNodeSelectionWidget.h>
#include <mitkIRenderWindowPart.h>



const std::string NeuroSurgery::VIEW_ID = "org.mitk.views.neurosurgery";

void NeuroSurgery::SetFocus()
{
    // do nothing, but you can't delete it
}

void NeuroSurgery::CreateQtPartControl(QWidget* parent)
{
    m_Controls.setupUi(parent);

    // Param Init
    ParamsInit();

    // make UI plane for different func plane.
    CreatQT_Basic();
    CreateQT_ImageProcess();
    CreateQT_PreoperativePlan();
    CreateQT_IntraoperativePlan();
    CreateQT_PostoperativeVerify();
    CreateQT_AccVerifyy();

}

void NeuroSurgery::CreatQT_Basic()
{
    /* Basic Funs
    * 1. data load
    * 2. data select
    */
    // data load
    connect(m_Controls.checkBaseData_pushButton, &QPushButton::clicked, this, &NeuroSurgery::OnCheckDataClicked);

    // Initialize selectors
    // Use AC-PC-HI to Set Pos of Brain 3d Scans
    InitImageSelector(m_Controls.mitk_MRI_Pic);
    InitImageSelector(m_Controls.mitk_PET_Pic);
    InitImageSelector(m_Controls.mitk_CT_Pic);
    InitImageSelector(m_Controls.mitk_DTI_Pic);
    InitImageSelector(m_Controls.mitk_Vessel_Pic);



}

void NeuroSurgery::CreateQT_PreoperativePlan()
{
    /* Funcs for Preoperative planning
    * 1.
    * 2.
    */

    // auto image registration

}

void NeuroSurgery::CreateQT_ImageProcess()
{
    /* Funcs for Multimodal Brain Image Processing
    * 1. Image Registration
    * 2. Processed File Load and re-processing
    * 3. Image Co-Visulizetion
    */

    // Image Registration
    InitializeAlgorithmComboBox();


    connect(m_Controls.targetNodeSelector, &QmitkAbstractNodeSelectionWidget::CurrentSelectionChanged, this, &NeuroSurgery::OnNodeSelectionChanged);
    connect(m_Controls.movingNodeSelector, &QmitkAbstractNodeSelectionWidget::CurrentSelectionChanged, this, &NeuroSurgery::OnNodeSelectionChanged);
    connect(m_Controls.targetMaskNodeSelector, &QmitkAbstractNodeSelectionWidget::CurrentSelectionChanged, this, &NeuroSurgery::OnNodeSelectionChanged);
    connect(m_Controls.movingMaskNodeSelector, &QmitkAbstractNodeSelectionWidget::CurrentSelectionChanged, this, &NeuroSurgery::OnNodeSelectionChanged);

    // 2. Processed File Load and re - processing
    connect(m_Controls.pushButton_CheckProcessDataPath, &QPushButton::clicked, this, &NeuroSurgery::OnCheckProcessDataClicked);
    connect(m_Controls.pushButton_ProcessedDataLoad, &QPushButton::clicked, this, &NeuroSurgery::OnLoadDataButtonClicked);

    connect(m_Controls.pushButton_MaskToSeg, &QPushButton::clicked, this, &NeuroSurgery::OnMaskToSegmentClicked);
    connect(m_Controls.pushButton_SegToModel, &QPushButton::clicked, this, &NeuroSurgery::OnSegmentToModelClicked);

    connect(m_Controls.pushButton_Visulize, &QPushButton::clicked, this, &NeuroSurgery::OnVisualizeButtonClicked);

    connect(m_Controls.pushButton_ApplyVisulize, &QPushButton::clicked, this, &NeuroSurgery::ApplyVisulize);





    // 3. Image Co-Visulizetion
    // data selection
    connect(m_Controls.pushButton_FindAlgorithm, &QPushButton::clicked, this, &NeuroSurgery::OnFindAlgorithmClicked);

    connect(m_Controls.pushButton_SelectAlgorithm, &QPushButton::clicked, this, &NeuroSurgery::OnAlgorithmSelectionChanged);

    // load Algorithm
    connect(m_Controls.m_pbLoadSelected, &QPushButton::clicked, this, &NeuroSurgery::OnLoadAlgorithmButtonPushed);

    // execution
    connect(m_Controls.m_pbStartReg, SIGNAL(clicked()), this, SLOT(OnStartRegBtnPushed()));
    connect(m_Controls.m_pbStopReg, SIGNAL(clicked()), this, SLOT(OnStopRegBtnPushed()));
    connect(m_Controls.m_pbSaveLog, SIGNAL(clicked()), this, SLOT(OnSaveLogBtnPushed()));

    // Gizmo
    connect(m_Controls.pushButton_Gizmo, &QPushButton::clicked, this, &NeuroSurgery::UseGizmoBtnClicked);
    connect(m_Controls.pushButton_ResetGizmo, &QPushButton::clicked, this, &NeuroSurgery::ResetGizmo);






    // 2. Image Co-Visulizetion
    // MRA
    InitImageSelector(m_Controls.mitkNodeSelectWidget_MRA);
    InitImageSelector(m_Controls.mitkNodeSelectWidget_MRA_Mask);
    InitImageSelector(m_Controls.mitkNodeSelectWidget_MRA_MaskedResult);

    // Mask fit & Region visulizetion
    connect(m_Controls.pushButton_FitMask, &QPushButton::clicked, this, &NeuroSurgery::OnFitMaskClicked);
    connect(m_Controls.pushButton_VesselReconstruction, &QPushButton::clicked, this, &NeuroSurgery::OnReconstructMRASurfaceClicked);

    // PET
    InitImageSelector(m_Controls.mitkNodeSelectWidget_PET);
    InitImageSelector(m_Controls.mitkNodeSelectWidget_PET_Mask);
    InitImageSelector(m_Controls.mitkNodeSelectWidget_PET_MaskedResult);

    connect(m_Controls.pushButton_FitMask_PET, &QPushButton::clicked, this, &NeuroSurgery::OnMaskPETRegionClicked);
    connect(m_Controls.pushButton_colorfyPET, &QPushButton::clicked, this, &NeuroSurgery::OnCheckPETColorfyClicked);
    connect(m_Controls.pushButton_PET_Visulized, &QPushButton::clicked, this, &NeuroSurgery::OnPETResultVisualizeClicked);

    // DTI
    connect(m_Controls.pushButton_FIBVisulizetion, &QPushButton::clicked, this, &NeuroSurgery::OnLoadFibFileClicked);


}

void NeuroSurgery::CreateQT_IntraoperativePlan()
{
    /* Funcs for Intraoperative planning
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_PostoperativeVerify()
{
    /* Funcs for Intraoperative planning
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_AccVerifyy()
{
    /* Funcs for NS system accuracy verification
    * 1.
    * 2.
    */
}

void NeuroSurgery::InitializeAlgorithmComboBox()
{
    // button init
    m_Controls.movingNodeSelector->SetDataStorage(this->GetDataStorage());
    m_Controls.targetNodeSelector->SetDataStorage(this->GetDataStorage());
    m_Controls.movingMaskNodeSelector->SetDataStorage(this->GetDataStorage());
    m_Controls.targetMaskNodeSelector->SetDataStorage(this->GetDataStorage());

    m_Controls.movingNodeSelector->SetSelectionIsOptional(false);
    m_Controls.targetNodeSelector->SetSelectionIsOptional(false);
    m_Controls.movingMaskNodeSelector->SetSelectionIsOptional(true);
    m_Controls.targetMaskNodeSelector->SetSelectionIsOptional(true);

    AdaptFolderGUIElements();
    CheckInputs();
    ConfigureProgressInfos();
    ConfigureRegistrationControls();
    ConfigureNodeSelectors();
}
