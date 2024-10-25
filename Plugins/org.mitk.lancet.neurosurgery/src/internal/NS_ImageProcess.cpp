/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"

#include <iostream>
#include <filesystem>

#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>
#include <berryPlatform.h>

// Qt
#include <QMessageBox>
#include <QmitkMappingJob.h>
#include <QThreadPool>
#include <QDateTime>
#include <QFileDialog>
#include <QErrorMessage>
#include <QFileInfo>

// MITK
#include <mitkStatusBar.h>
#include <mitkIOUtil.h>
#include "mitkDataStorage.h"
#include "mitkDataNode.h"
#include "mitkImage.h"
#include "mitkImageCast.h"
#include "mitkSurface.h"
#include "mitkImageToSurfaceFilter.h"
#include "mitkLookupTable.h"
#include "mitkLookupTableProperty.h"
#include "mitkPropertyList.h"
#include "mitkImageTimeSelector.h"
#include "mitkResultNodeGenerationHelper.h"
#include <mitkRegistrationHelper.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateDimension.h>
#include <mitkMAPAlgorithmHelper.h>
#include <mitkRenderingManager.h>
#include <mitkLabelSetImage.h>
#include <mitkImageToSurfaceFilter.h>
#include <mitkIRenderWindowPart.h>
#include <mitkBaseData.h>
#include <QmitkRenderWindow.h>

// VTK
#include <vtkXMLPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>

// ITK
#include "itkResampleImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkIdentityTransform.h"
#include "itkMaskImageFilter.h"
#include "itkImage.h"
#include "itkImageRegistrationMethod.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizer.h"
#include "itkMeanSquaresImageToImageMetric.h"
#include "itkRigid3DTransform.h"
#include "itkAffineTransform.h"

// MatchPoint
#include <mapImageRegistrationAlgorithmInterface.h>
#include <mapPointSetRegistrationAlgorithmInterface.h>
#include <mapRegistrationAlgorithmInterface.h>
#include <mapMaskedRegistrationAlgorithmInterface.h>
#include <mapAlgorithmEvents.h>
#include <mapAlgorithmWrapperEvent.h>
#include <mapExceptionObjectMacros.h>
#include <mapConvert.h>
#include <mapDeploymentDLLAccess.h>

void NeuroSurgery::OnRegistrateImageClicked()
{

}

void NeuroSurgery::PerformRigidRegistration(mitk::Image::Pointer movingImage, mitk::Image::Pointer fixedImage)
{
    typedef itk::Image<float, 3> ItkImageType;
    typedef itk::Rigid3DTransform<double> TransformType;
    typedef itk::RegularStepGradientDescentOptimizer OptimizerType;
    typedef itk::MeanSquaresImageToImageMetric<ItkImageType, ItkImageType> MetricType;
    typedef itk::LinearInterpolateImageFunction<ItkImageType, double> InterpolatorType;
    typedef itk::ImageRegistrationMethod<ItkImageType, ItkImageType> RegistrationType;

    ItkImageType::Pointer itkMovingImage;
    ItkImageType::Pointer itkFixedImage;
    mitk::CastToItkImage(movingImage, itkMovingImage);
    mitk::CastToItkImage(fixedImage, itkFixedImage);

    TransformType::Pointer transform = TransformType::New();
    OptimizerType::Pointer optimizer = OptimizerType::New();
    MetricType::Pointer metric = MetricType::New();
    InterpolatorType::Pointer interpolator = InterpolatorType::New();
    RegistrationType::Pointer registration = RegistrationType::New();

    registration->SetTransform(transform);
    registration->SetOptimizer(optimizer);
    registration->SetMetric(metric);
    registration->SetInterpolator(interpolator);
    registration->SetFixedImage(itkFixedImage);
    registration->SetMovingImage(itkMovingImage);

    typedef itk::CenteredTransformInitializer<TransformType, ItkImageType, ItkImageType> InitializerType;
    InitializerType::Pointer initializer = InitializerType::New();
    initializer->SetTransform(transform);
    initializer->SetFixedImage(itkFixedImage);
    initializer->SetMovingImage(itkMovingImage);
    initializer->MomentsOn();
    initializer->InitializeTransform();

    registration->SetInitialTransformParameters(transform->GetParameters());

    optimizer->SetMaximumStepLength(0.1);
    optimizer->SetMinimumStepLength(0.01);
    optimizer->SetNumberOfIterations(200);
    try
    {
        registration->Update();
    }
    catch (itk::ExceptionObject& err)
    {
        QMessageBox::critical(nullptr, tr("Registration Error"), QString::fromStdString(err.GetDescription()));
        return;
    }

    QMessageBox::information(nullptr, tr("Rigid Registration"), tr("Rigid registration completed successfully."));
}

void NeuroSurgery::PerformAffineRegistration(mitk::Image::Pointer movingImage, mitk::Image::Pointer fixedImage)
{
    typedef itk::Image<float, 3> ItkImageType;
    typedef itk::AffineTransform<double, 3> TransformType;
    typedef itk::RegularStepGradientDescentOptimizer OptimizerType;
    typedef itk::MeanSquaresImageToImageMetric<ItkImageType, ItkImageType> MetricType;
    typedef itk::LinearInterpolateImageFunction<ItkImageType, double> InterpolatorType;
    typedef itk::ImageRegistrationMethod<ItkImageType, ItkImageType> RegistrationType;

    ItkImageType::Pointer itkMovingImage;
    ItkImageType::Pointer itkFixedImage;
    mitk::CastToItkImage(movingImage, itkMovingImage);
    mitk::CastToItkImage(fixedImage, itkFixedImage);

    TransformType::Pointer transform = TransformType::New();
    OptimizerType::Pointer optimizer = OptimizerType::New();
    MetricType::Pointer metric = MetricType::New();
    InterpolatorType::Pointer interpolator = InterpolatorType::New();
    RegistrationType::Pointer registration = RegistrationType::New();

    registration->SetTransform(transform);
    registration->SetOptimizer(optimizer);
    registration->SetMetric(metric);
    registration->SetInterpolator(interpolator);
    registration->SetFixedImage(itkFixedImage);
    registration->SetMovingImage(itkMovingImage);

    registration->SetInitialTransformParameters(transform->GetParameters());

    optimizer->SetMaximumStepLength(0.1);
    optimizer->SetMinimumStepLength(0.01);
    optimizer->SetNumberOfIterations(200);

    try
    {
        registration->Update();
    }
    catch (itk::ExceptionObject& err)
    {
        QMessageBox::critical(nullptr, tr("Registration Error"), QString::fromStdString(err.GetDescription()));
        return;
    }

    QMessageBox::information(nullptr, tr("Affine Registration"), tr("Affine registration completed successfully."));
}


void NeuroSurgery::Error(QString msg)
{
    mitk::StatusBar::GetInstance()->DisplayErrorText(msg.toLatin1());
    MITK_ERROR << msg.toStdString().c_str();

    m_Controls.m_teLog->append(QStringLiteral("<font color='red'><b>") + msg + QStringLiteral("</b></font>"));
}

bool NeuroSurgery::CheckInputs() {
    if (m_LoadedAlgorithm.IsNull()) {
        m_spSelectedMovingNode = nullptr;
        m_spSelectedMovingData = nullptr;
        m_spSelectedTargetNode = nullptr;
        m_spSelectedTargetData = nullptr;
    }
    else {
        if (m_Controls.movingNodeSelector->GetSelectedNode().IsNull()) {
            m_spSelectedMovingNode = nullptr;
            m_spSelectedMovingData = nullptr;
        }
        else {
            m_spSelectedMovingNode = m_Controls.movingNodeSelector->GetSelectedNode();
            m_spSelectedMovingData = m_spSelectedMovingNode->GetData();
            auto movingImage = dynamic_cast<mitk::Image*>(m_spSelectedMovingNode->GetData());

            if (movingImage && movingImage->GetDimension() - 1 == m_LoadedAlgorithm->getMovingDimensions()
                && movingImage->GetTimeSteps() > 1) {
                m_spSelectedMovingData = ExtractFirstFrame(movingImage).GetPointer();
                m_Controls.m_teLog->append(
                    QStringLiteral("<font color='gray'><i>Selected moving image has multiple time steps. First time step is used as moving image.</i></font>"));
            }
        }

        if (m_Controls.targetNodeSelector->GetSelectedNode().IsNull()) {
            m_spSelectedTargetNode = nullptr;
            m_spSelectedTargetData = nullptr;
        }
        else {
            m_spSelectedTargetNode = m_Controls.targetNodeSelector->GetSelectedNode();
            m_spSelectedTargetData = m_spSelectedTargetNode->GetData();
            auto targetImage = dynamic_cast<mitk::Image*>(m_spSelectedTargetNode->GetData());

            if (targetImage && targetImage->GetDimension() - 1 == m_LoadedAlgorithm->getTargetDimensions()
                && targetImage->GetTimeSteps() > 1) {
                m_spSelectedTargetData = ExtractFirstFrame(targetImage).GetPointer();
                m_Controls.m_teLog->append(
                    QStringLiteral("<font color='gray'><i>Selected target image has multiple time steps. First time step is used as target image.</i></font>"));
            }
        }
    }
    m_ValidInputs = m_spSelectedMovingData.IsNotNull() && m_spSelectedTargetData.IsNotNull();
    return m_ValidInputs;
}

void NeuroSurgery::OnNodeSelectionChanged(QList<mitk::DataNode::Pointer> /*nodes*/)
{
    if (!m_Working)
    {
        CheckInputs();
        ConfigureRegistrationControls();
    }
}


mitk::Image::Pointer NeuroSurgery::ExtractFirstFrame(const mitk::Image* dynamicImage)
{
    mitk::ImageTimeSelector::Pointer imageTimeSelector = mitk::ImageTimeSelector::New();
    imageTimeSelector->SetInput(dynamicImage);
    imageTimeSelector->SetTimeNr(0);
    imageTimeSelector->UpdateLargestPossibleRegion();

    return imageTimeSelector->GetOutput();
}

void NeuroSurgery::ConfigureProgressInfos()
{
    const IIterativeAlgorithm* pIterative = dynamic_cast<const IIterativeAlgorithm*>
        (m_LoadedAlgorithm.GetPointer());
    const IMultiResAlgorithm* pMultiRes = dynamic_cast<const IMultiResAlgorithm*>
        (m_LoadedAlgorithm.GetPointer());

    m_Controls.m_progBarIteration->setVisible(pIterative);
    m_Controls.m_lbProgBarIteration->setVisible(pIterative);


    if (pIterative)
    {
        QString format = "%p% (%v/%m)";

        if (!pIterative->hasMaxIterationCount())
        {
            format = "%v";
            m_Controls.m_progBarIteration->setMaximum(0);
        }
        else
        {
            m_Controls.m_progBarIteration->setMaximum(pIterative->getMaxIterations());
        }

        m_Controls.m_progBarIteration->setFormat(format);
    }

    m_Controls.m_progBarLevel->setVisible(pMultiRes);
    m_Controls.m_lbProgBarLevel->setVisible(pMultiRes);

    if (pMultiRes)
    {
        m_Controls.m_progBarLevel->setMaximum(pMultiRes->getResolutionLevels());

    }
    else
    {
        m_Controls.m_progBarLevel->setMaximum(1);
    }

    m_Controls.m_progBarIteration->reset();
    m_Controls.m_progBarLevel->reset();
}

void NeuroSurgery::ConfigureRegistrationControls()
{
    m_Controls.m_tabSelection->setEnabled(!m_Working);
    m_Controls.m_leRegJobName->setEnabled(!m_Working);
    m_Controls.groupMasks->setEnabled(!m_Working);

    m_Controls.m_pbStartReg->setEnabled(false);
    m_Controls.m_pbStopReg->setEnabled(false);
    m_Controls.m_pbStopReg->setVisible(false);

    if (m_LoadedAlgorithm.IsNotNull())
    {
        m_Controls.m_tabSettings->setEnabled(!m_Working);
        m_Controls.m_tabExecution->setEnabled(true);
        m_Controls.m_pbStartReg->setEnabled(m_ValidInputs && !m_Working);
        m_Controls.m_leRegJobName->setEnabled(!m_Working);
        m_Controls.m_checkMapEntity->setEnabled(!m_Working);
        m_Controls.targetNodeSelector->setEnabled(!m_Working);
        m_Controls.movingNodeSelector->setEnabled(!m_Working);
        m_Controls.targetMaskNodeSelector->setEnabled(!m_Working);
        m_Controls.movingMaskNodeSelector->setEnabled(!m_Working);

        const IStoppableAlgorithm* pIterativ = dynamic_cast<const IStoppableAlgorithm*>
            (m_LoadedAlgorithm.GetPointer());

        if (pIterativ)
        {
            m_Controls.m_pbStopReg->setVisible(pIterativ->isStoppable());
        }

        typedef ::map::algorithm::facet::MaskedRegistrationAlgorithmInterface<3, 3> MaskRegInterface;
        const MaskRegInterface* pMaskReg = dynamic_cast<const MaskRegInterface*>
            (m_LoadedAlgorithm.GetPointer());

        m_Controls.groupMasks->setVisible(pMaskReg != nullptr);

        //if the stop button is set to visible and the algorithm is working ->
        //then the algorithm is stoppable, thus enable the button.
        m_Controls.m_pbStopReg->setEnabled(m_Controls.m_pbStopReg->isVisible() && m_Working);

        this->m_Controls.m_lbLoadedAlgorithmName->setText(
            QString::fromStdString(m_LoadedAlgorithm->getUID()->toStr()));
    }
    else
    {
        m_Controls.m_tabSettings->setEnabled(false);
        m_Controls.m_tabExecution->setEnabled(false);
        this->m_Controls.m_lbLoadedAlgorithmName->setText(
            QStringLiteral("<font color='red'>no algorithm loaded!</font>"));
        m_Controls.groupMasks->setVisible(false);
    }

    if (!m_Working)
    {
        this->m_Controls.m_leRegJobName->setText(QString::fromStdString(this->GetDefaultRegJobName()));
    }
}

void NeuroSurgery::OnAlgorithmSelectionChanged()
{
    m_Controls.textBrowser_actionLOG->append("Action: Check Selected Algorithm.");
    // get index
    int index = m_Controls.m_algorithmComboBox->currentIndex();

    QString dllPath = m_Controls.m_algorithmComboBox->itemData(index).toString();

    if (dllPath.isEmpty())
    {
        Error(QStringLiteral("No valid algorithm is selected. Cannot load algorithm. ABORTING."));
        return;
    }
    else
    {
        m_Controls.textBrowser_actionLOG->append("Loaded, path: " + dllPath);

    }

    // char* deployedAlgorithmLoadPath = getenv("MAP_MDRA_LOAD_PATH");
    // cout << "current path" << deployedAlgorithmLoadPath;

    std::filesystem::path currentPath = std::filesystem::current_path();
    m_Controls.textBrowser_actionLOG->append("Current path: " + QString(dllPath));

    // load .dll
    ::map::deployment::DLLHandle::Pointer tempDLLHandle = ::map::deployment::openDeploymentDLL(dllPath.toStdString());
    ::map::algorithm::RegistrationAlgorithmBase::Pointer tempAlgorithm
        = ::map::deployment::getRegistrationAlgorithm(tempDLLHandle);

    if (tempAlgorithm.IsNull())
    {
        Error(QStringLiteral("Error. Cannot load selected algorithm."));
        return;
    }

    this->m_LoadedAlgorithm = tempAlgorithm;
    this->m_LoadedDLLHandle = tempDLLHandle;

    this->m_Controls.m_AlgoConfigurator->setAlgorithm(m_LoadedAlgorithm);

    typedef ::map::algorithm::facet::MaskedRegistrationAlgorithmInterface<3, 3> MaskRegInterface;
    const MaskRegInterface* pMaskReg = dynamic_cast<const MaskRegInterface*>
        (m_LoadedAlgorithm.GetPointer());

    if (!pMaskReg)
    {
        m_spSelectedTargetMaskData = nullptr;
        m_spSelectedTargetMaskNode = nullptr;
        m_spSelectedMovingMaskData = nullptr;
        m_spSelectedMovingMaskNode = nullptr;
        m_Controls.targetMaskNodeSelector->SetCurrentSelection(QmitkAbstractNodeSelectionWidget::NodeList());
        m_Controls.movingMaskNodeSelector->SetCurrentSelection(QmitkAbstractNodeSelectionWidget::NodeList());
    }

    this->AdaptFolderGUIElements();
    this->ConfigureNodeSelectors();
    this->CheckInputs();
    this->ConfigureRegistrationControls();
    this->ConfigureProgressInfos();
    this->m_Controls.m_tabs->setCurrentIndex(1);
    this->m_Controls.m_teLog->clear();
}


const map::deployment::DLLInfo* NeuroSurgery::GetSelectedAlgorithmDLL() const
{
    return m_SelectedAlgorithmInfo;
}

void NeuroSurgery::OnLoadAlgorithmButtonPushed()
{
    map::deployment::DLLInfo::ConstPointer dllInfo = GetSelectedAlgorithmDLL();

    if (!dllInfo)
    {
        Error(QStringLiteral("No valid algorithm is selected. Cannot load algorithm. ABORTING."));
        return;
    }

    ::map::deployment::DLLHandle::Pointer tempDLLHandle = ::map::deployment::openDeploymentDLL(
        dllInfo->getLibraryFilePath());
    ::map::algorithm::RegistrationAlgorithmBase::Pointer tempAlgorithm
        = ::map::deployment::getRegistrationAlgorithm(tempDLLHandle);

    if (tempAlgorithm.IsNull())
    {
        Error(QStringLiteral("Error. Cannot load selected algorithm."));
        return;
    }

    this->m_LoadedAlgorithm = tempAlgorithm;
    this->m_LoadedDLLHandle = tempDLLHandle;

    this->m_Controls.m_AlgoConfigurator->setAlgorithm(m_LoadedAlgorithm);

    typedef ::map::algorithm::facet::MaskedRegistrationAlgorithmInterface<3, 3> MaskRegInterface;
    const MaskRegInterface* pMaskReg = dynamic_cast<const MaskRegInterface*>
        (m_LoadedAlgorithm.GetPointer());

    if (!pMaskReg)
    {
        m_spSelectedTargetMaskData = nullptr;
        m_spSelectedTargetMaskNode = nullptr;
        m_spSelectedMovingMaskData = nullptr;
        m_spSelectedMovingMaskNode = nullptr;
        m_Controls.targetMaskNodeSelector->SetCurrentSelection(QmitkAbstractNodeSelectionWidget::NodeList());
        m_Controls.movingMaskNodeSelector->SetCurrentSelection(QmitkAbstractNodeSelectionWidget::NodeList());
    }


    this->AdaptFolderGUIElements();
    this->ConfigureNodeSelectors();
    this->CheckInputs();
    this->ConfigureRegistrationControls();
    this->ConfigureProgressInfos();
    this->m_Controls.m_tabs->setCurrentIndex(1);
    this->m_Controls.m_teLog->clear();
}

void NeuroSurgery::AdaptFolderGUIElements()
{
    m_Controls.m_pbLoadSelected->setEnabled(m_CanLoadAlgorithm);
}


void NeuroSurgery::OnSaveLogBtnPushed()
{
    QDateTime currentTime = QDateTime::currentDateTime();
    QString fileName = tr("registration_log_") + currentTime.toString(tr("yyyy-MM-dd_hh-mm-ss")) +
        tr(".txt");
    fileName = QFileDialog::getSaveFileName(nullptr, tr("Save registration log"), fileName,
        tr("Text files (*.txt)"));

    if (fileName.isEmpty())
    {
        QMessageBox::critical(nullptr, tr("No file selected!"),
            tr("Cannot save registration log file. Please selected a file."));
    }
    else
    {
        std::ofstream file;

        std::ios_base::openmode iOpenFlag = std::ios_base::out | std::ios_base::trunc;
        file.open(fileName.toStdString().c_str(), iOpenFlag);

        if (!file.is_open())
        {
            mitkThrow() << "Cannot open or create specified file to save. File path: "
                << fileName.toStdString();
        }

        file << this->m_Controls.m_teLog->toPlainText().toStdString() << std::endl;

        file.close();
    }

}

void NeuroSurgery::ConfigureNodeSelectors()
{
    /* Func change Selectors */
    auto isImage = mitk::MITKRegistrationHelper::ImageNodePredicate();
    auto isPointSet = mitk::MITKRegistrationHelper::PointSetNodePredicate();
    auto isMask = mitk::MITKRegistrationHelper::MaskNodePredicate();
    mitk::NodePredicateBase::Pointer dimensionPredicate = mitk::NodePredicateOr::New(mitk::NodePredicateDimension::New(3), mitk::NodePredicateDimension::New(4)).GetPointer();


    m_Controls.movingNodeSelector->setEnabled(m_LoadedAlgorithm.IsNotNull());
    m_Controls.targetNodeSelector->setEnabled(m_LoadedAlgorithm.IsNotNull());
    m_Controls.movingMaskNodeSelector->setEnabled(m_LoadedAlgorithm.IsNotNull());
    m_Controls.targetMaskNodeSelector->setEnabled(m_LoadedAlgorithm.IsNotNull());

    if (m_LoadedAlgorithm.IsNotNull())
    {
        mitk::NodePredicateBase::ConstPointer dataPredicate;

        if (m_LoadedAlgorithm->getMovingDimensions() == 2)
        {
            dimensionPredicate = mitk::NodePredicateDimension::New(2);
        }

        if (mitk::MAPAlgorithmHelper::HasImageAlgorithmInterface(m_LoadedAlgorithm))
        {
            dataPredicate = mitk::NodePredicateAnd::New(isImage, dimensionPredicate);

            m_Controls.movingNodeSelector->SetInvalidInfo("Select valid moving image.");
            m_Controls.movingNodeSelector->SetPopUpTitel("Select moving image.");
            m_Controls.movingNodeSelector->SetPopUpHint("Select the moving image that should be registered onto the target image.");
            m_Controls.targetNodeSelector->SetInvalidInfo("Select valid target image.");
            m_Controls.targetNodeSelector->SetPopUpTitel("Select target image.");
            m_Controls.targetNodeSelector->SetPopUpHint("Select the target image that should be used as reference for the registration.");
        }

        if (mitk::MAPAlgorithmHelper::HasPointSetAlgorithmInterface(m_LoadedAlgorithm))
        {
            if (dataPredicate.IsNull())
            {
                dataPredicate = isPointSet;
                m_Controls.movingNodeSelector->SetInvalidInfo("Select valid moving point set.");
                m_Controls.movingNodeSelector->SetPopUpTitel("Select moving point set.");
                m_Controls.movingNodeSelector->SetPopUpHint("Select the moving point set that should be registered onto the target point set.");
                m_Controls.targetNodeSelector->SetInvalidInfo("Select valid target point set.");
                m_Controls.targetNodeSelector->SetPopUpTitel("Select target point set.");
                m_Controls.targetNodeSelector->SetPopUpHint("Select the target point set that should be used as reference for the registration.");
            }
            else
            {
                dataPredicate = mitk::NodePredicateOr::New(dataPredicate, isPointSet);
                m_Controls.movingNodeSelector->SetInvalidInfo("Select valid moving data.");
                m_Controls.movingNodeSelector->SetPopUpTitel("Select moving data.");
                m_Controls.movingNodeSelector->SetPopUpHint("Select the moving data that should be registered onto the target data. The algorithm supports images as well as point sets.");
                m_Controls.targetNodeSelector->SetInvalidInfo("Select valid target data.");
                m_Controls.targetNodeSelector->SetPopUpTitel("Select target data.");
                m_Controls.targetNodeSelector->SetPopUpHint("Select the target data that should be used as reference for the registration. The algorithm supports images as well as point sets.");
            }
        }
        mitk::NodePredicateBase::ConstPointer nodePredicate = dataPredicate;

        m_Controls.movingNodeSelector->SetNodePredicate(nodePredicate);
        m_Controls.targetNodeSelector->SetNodePredicate(nodePredicate);

        nodePredicate = mitk::NodePredicateAnd::New(isMask, dimensionPredicate);

        m_Controls.movingMaskNodeSelector->SetEmptyInfo("Select moving mask. (optional)");
        m_Controls.movingMaskNodeSelector->SetPopUpTitel("Select moving mask");
        m_Controls.movingMaskNodeSelector->SetPopUpHint("Select a segmentation that serves as moving mask for the registration.");
        m_Controls.targetMaskNodeSelector->SetEmptyInfo("Select target mask. (optional)");
        m_Controls.targetMaskNodeSelector->SetPopUpTitel("Select target mask");
        m_Controls.targetMaskNodeSelector->SetPopUpHint("Select a segmentation that serves as target mask for the registration.");

        m_Controls.movingMaskNodeSelector->SetNodePredicate(nodePredicate);
        m_Controls.targetMaskNodeSelector->SetNodePredicate(nodePredicate);
    }

}


void NeuroSurgery::OnStopRegBtnPushed()
{
    if (m_LoadedAlgorithm.IsNotNull())
    {
        IStoppableAlgorithm* pIterativ = dynamic_cast<IStoppableAlgorithm*>(m_LoadedAlgorithm.GetPointer());

        if (pIterativ && pIterativ->isStoppable())
        {
            if (pIterativ->stopAlgorithm())
            {

            }
            else
            {

            }

            m_Controls.m_pbStopReg->setEnabled(false);
        }
        else
        {
        }
    }
}

void NeuroSurgery::OnStartRegBtnPushed() {
    this->m_Working = true;
    this->ConfigureProgressInfos();
    m_Controls.m_progBarIteration->reset();
    m_Controls.m_progBarLevel->reset();
    this->ConfigureRegistrationControls();

    if (m_Controls.m_checkClearLog->checkState() == Qt::Checked) {
        this->m_Controls.m_teLog->clear();
    }

    QmitkRegistrationJob* pJob = new QmitkRegistrationJob(m_LoadedAlgorithm);
    pJob->setAutoDelete(true);
    pJob->m_spTargetData = m_spSelectedTargetData;
    pJob->m_spMovingData = m_spSelectedMovingData;
    pJob->m_TargetDataUID = mitk::EnsureUID(this->m_spSelectedTargetNode->GetData());
    pJob->m_MovingDataUID = mitk::EnsureUID(this->m_spSelectedMovingNode->GetData());

    if (m_spSelectedTargetMaskData.IsNotNull()) {
        pJob->m_spTargetMask = m_spSelectedTargetMaskData;
        pJob->m_TargetMaskDataUID = mitk::EnsureUID(this->m_spSelectedTargetMaskNode->GetData());
    }

    if (m_spSelectedMovingMaskData.IsNotNull()) {
        pJob->m_spMovingMask = m_spSelectedMovingMaskData;
        pJob->m_MovingMaskDataUID = mitk::EnsureUID(this->m_spSelectedMovingMaskNode->GetData());
    }

    pJob->m_JobName = m_Controls.m_leRegJobName->text().toStdString();
    pJob->m_StoreReg = true;

    connect(pJob, SIGNAL(Error(QString)), this, SLOT(OnRegJobError(QString)));
    connect(pJob, SIGNAL(Finished()), this, SLOT(OnRegJobFinished()));
    connect(pJob, SIGNAL(RegResultIsAvailable(mitk::MAPRegistrationWrapper::Pointer, const QmitkRegistrationJob*)), this,
        SLOT(OnRegResultIsAvailable(mitk::MAPRegistrationWrapper::Pointer, const QmitkRegistrationJob*)),
        Qt::BlockingQueuedConnection);

    connect(pJob, SIGNAL(AlgorithmInfo(QString)), this, SLOT(OnAlgorithmInfo(QString)));
    connect(pJob, SIGNAL(AlgorithmStatusChanged(QString)), this,
        SLOT(OnAlgorithmStatusChanged(QString)));
    connect(pJob, SIGNAL(AlgorithmIterated(QString, bool, unsigned long)), this,
        SLOT(OnAlgorithmIterated(QString, bool, unsigned long)));
    connect(pJob, SIGNAL(LevelChanged(QString, bool, unsigned long)), this, SLOT(OnLevelChanged(QString,
        bool, unsigned long)));

    QThreadPool* threadPool = QThreadPool::globalInstance();
    threadPool->start(pJob);
}

std::string NeuroSurgery::GetDefaultRegJobName() const
{

    mitk::DataStorage::SetOfObjects::ConstPointer nodes = this->GetRegNodes().GetPointer();
    mitk::DataStorage::SetOfObjects::ElementIdentifier estimatedIndex = nodes->Size();

    bool isUnique = false;
    std::string result = "Unnamed Reg";

    while (!isUnique)
    {
        ++estimatedIndex;
        result = "Reg #" + ::map::core::convert::toStr(estimatedIndex);
        isUnique = this->GetDataStorage()->GetNamedNode(result) == nullptr;
    }

    return result;
}

mitk::DataStorage::SetOfObjects::Pointer NeuroSurgery::GetRegNodes() const
{

    mitk::DataStorage::SetOfObjects::ConstPointer nodes = this->GetDataStorage()->GetAll();
    mitk::DataStorage::SetOfObjects::Pointer result = mitk::DataStorage::SetOfObjects::New();

    for (mitk::DataStorage::SetOfObjects::const_iterator pos = nodes->begin(); pos != nodes->end();
        ++pos)
    {
        if (mitk::MITKRegistrationHelper::IsRegNode(*pos))
        {
            result->push_back(*pos);
        }
    }

    return result;
}

void NeuroSurgery::OnRegResultIsAvailable(mitk::MAPRegistrationWrapper::Pointer spResultRegistration, const QmitkRegistrationJob* pRegJob) {
    mitk::DataNode::Pointer spResultRegistrationNode = mitk::generateRegistrationResultNode(
        pRegJob->m_JobName, spResultRegistration, pRegJob->GetLoadedAlgorithm()->getUID()->toStr(),
        pRegJob->m_MovingDataUID, pRegJob->m_TargetDataUID);

    if (pRegJob->m_StoreReg) {
        m_Controls.m_teLog->append(
            QStringLiteral("<b><font color='blue'> Storing registration object in data manager ... </font></b>"));
        this->GetDataStorage()->Add(spResultRegistrationNode);

        auto* renderWindowPart = this->GetRenderWindowPart();
        if (nullptr != renderWindowPart)
            renderWindowPart->RequestUpdate();
    }

    if (m_Controls.m_checkMapEntity->checkState() == Qt::Checked) {
        QmitkMappingJob* pMapJob = new QmitkMappingJob();
        pMapJob->setAutoDelete(true);
        pMapJob->m_spInputData = pRegJob->m_spMovingData;
        pMapJob->m_InputDataUID = pRegJob->m_MovingDataUID;
        pMapJob->m_spRegNode = spResultRegistrationNode;
        pMapJob->m_doGeometryRefinement = false;
        pMapJob->m_spRefGeometry = pRegJob->m_spTargetData->GetGeometry()->Clone().GetPointer();

        // get save name
        QString RegistrationDataName = m_Controls.lineEdit_SaveRegistrationName->text();
        pMapJob->m_MappedName = std::string(RegistrationDataName.toStdString().c_str());
        pMapJob->m_allowUndefPixels = true;
        pMapJob->m_paddingValue = 100;
        pMapJob->m_allowUnregPixels = true;
        pMapJob->m_errorValue = 200;
        pMapJob->m_InterpolatorLabel = "Linear Interpolation";
        pMapJob->m_InterpolatorType = mitk::ImageMappingInterpolator::Linear;

        connect(pMapJob, SIGNAL(Error(QString)), this, SLOT(OnMapJobError(QString)));
        connect(pMapJob, SIGNAL(MapResultIsAvailable(mitk::BaseData::Pointer, const QmitkMappingJob*)), this,
            SLOT(OnMapResultIsAvailable(mitk::BaseData::Pointer, const QmitkMappingJob*)),
            Qt::BlockingQueuedConnection);

        QThreadPool* threadPool = QThreadPool::globalInstance();
        threadPool->start(pMapJob);
    }
}

void NeuroSurgery::OnMapResultIsAvailable(mitk::BaseData::Pointer spMappedData, const QmitkMappingJob* job) {
    m_Controls.m_teLog->append(QStringLiteral("<b><font color='blue'>Mapped entity stored. Name: ") +
        QString::fromStdString(job->m_MappedName) + QStringLiteral("</font></b>"));

    mitk::DataNode::Pointer spMappedNode = mitk::generateMappedResultNode(job->m_MappedName,
        spMappedData, job->GetRegistration()->getRegistrationUID(), job->m_InputDataUID,
        job->m_doGeometryRefinement, job->m_InterpolatorLabel);
    this->GetDataStorage()->Add(spMappedNode);
    spMappedNode->SetOpacity(0.5);

    auto* renderWindowPart = this->GetRenderWindowPart();
    if (nullptr != renderWindowPart)
        renderWindowPart->RequestUpdate();
}




void NeuroSurgery::OnFindAlgorithmClicked()
{
    /* Find Registration Algorithm in Current Coding Raw Path */
    // 1. find
    // 2. add path and name to Combox for Selection
    // 

    m_Controls.textBrowser_actionLOG->append("Action: Find Registration Algorithm.");
    // current path
    std::filesystem::path currentPath = std::filesystem::current_path();
    // std::cout << "Current path is: " << currentPath << std::endl;

    // Construct the target directory path
    std::filesystem::path targetDir = currentPath.parent_path().parent_path() / "bin" / "Release";

    // Initialize the Registration Algorithm DLL Path map
    QMap<QString, QString> algorithmMap;

    // Check if the target directory exists
    if (std::filesystem::exists(targetDir) && std::filesystem::is_directory(targetDir)) {
        // Iterate over the target directory
        for (const auto& entry : std::filesystem::directory_iterator(targetDir)) {
            // Check if the file starts with "mdra" and has a ".dll" extension
            if (entry.is_regular_file() && entry.path().extension() == ".dll" &&
                entry.path().filename().string().rfind("mdra", 0) == 0) {

                // Extract the algorithm name
                std::string fileName = entry.path().filename().string();
                //std::size_t startPos = fileName.find('_') + 1; // Find the position after the first underscore
                std::size_t startPos = fileName.find_first_of('_', fileName.find('_') + 1) + 1;
                std::size_t endPos = fileName.rfind('_'); // Find the position of the last underscore
                std::string algorithmName = fileName.substr(startPos, endPos - startPos);

                // Insert into the algorithmMap
                algorithmMap.insert(QString::fromStdString(algorithmName), QString::fromStdString(entry.path().string()));
            }
        }
    }
    else {
        std::cerr << "Target directory does not exist: " << targetDir << std::endl;
    }

    // Add algorithm names and paths to the ComboBox
    for (auto it = algorithmMap.begin(); it != algorithmMap.end(); ++it) {
        m_Controls.m_algorithmComboBox->addItem(it.key(), it.value());
    }
}

void NeuroSurgery::OnRegJobError(QString err)
{
    Error(err);
}

void NeuroSurgery::OnRegJobFinished()
{
    this->m_Working = false;

    auto* renderWindowPart = this->GetRenderWindowPart();

    if (nullptr != renderWindowPart)
        renderWindowPart->RequestUpdate();

    this->CheckInputs();
    this->ConfigureRegistrationControls();
    this->ConfigureProgressInfos();
}

void NeuroSurgery::OnMapJobError(QString err)
{
    Error(err);
}

void NeuroSurgery::OnAlgorithmInfo(QString info)
{
    m_Controls.m_teLog->append(QStringLiteral("<font color='gray'><i>") + info + QStringLiteral("</i></font>"));
}

void NeuroSurgery::OnAlgorithmStatusChanged(QString info)
{
    m_Controls.m_teLog->append(QStringLiteral("<b><font color='blue'>") + info + QStringLiteral(" </font></b>"));
}

void NeuroSurgery::OnAlgorithmIterated(QString info, bool hasIterationCount,
    unsigned long currentIteration)
{
    if (hasIterationCount)
    {
        m_Controls.m_progBarIteration->setValue(currentIteration);
    }

    m_Controls.m_teLog->append(info);
}

void NeuroSurgery::OnLevelChanged(QString info, bool hasLevelCount, unsigned long currentLevel)
{
    if (hasLevelCount)
    {
        m_Controls.m_progBarLevel->setValue(currentLevel);
    }

    m_Controls.m_teLog->append(QStringLiteral("<b><font color='green'>") + info + QStringLiteral("</font></b>"));
}





void NeuroSurgery::OnReconstructMRASurfaceClicked()
{
    /* Use Threshold to Reconstruct Vessels */

    m_Controls.textBrowser_actionLOG->append("Action: Reconstruct MRASurface.");

    auto inputMaskedImage = dynamic_cast<mitk::Image*>(m_Controls.mitkNodeSelectWidget_MRA_MaskedResult->GetSelectedNode()->GetData());

    if (inputMaskedImage != nullptr)
    {
        // Get lower and upper threshold values
        double lowerThreshold = m_Controls.lineEdit_MRA_Reconstruct_lower->text().toDouble();
        // double upperThreshold = m_Controls.lineEdit_MRA_Reconstruct_upper->text().toDouble();

        auto mitkRecontructedSurfaces = mitk::Surface::New();
        mitk::ImageToSurfaceFilter::Pointer imageToSurfaceFilter = mitk::ImageToSurfaceFilter::New();

        // Set smoothing parameters
        if (m_Controls.checkBox_Smooth->isChecked())
        {
            int iteration = m_Controls.lineEdit_MRA_SmoothLoop->text().toInt();
            double relaxation = m_Controls.lineEdit_MRA_SmoothRate->text().toDouble();
            imageToSurfaceFilter->SmoothOn();
            imageToSurfaceFilter->SetSmoothRelaxation(relaxation);
            imageToSurfaceFilter->SetSmoothIteration(iteration);
        }

        imageToSurfaceFilter->SetInput(inputMaskedImage);
        imageToSurfaceFilter->SetThreshold(lowerThreshold);
        // imageToSurfaceFilter->SetThresholdRange(lowerThreshold, upperThreshold); // Assuming this method exists
        mitkRecontructedSurfaces = imageToSurfaceFilter->GetOutput();

        // Get the name for the surface from lineEdit_MRA_SmoothName
        QString surfaceName = m_Controls.lineEdit_MRA_SmoothName->text();

        // Render the extracted surface
        auto nodeSteelballSurfaces = mitk::DataNode::New();
        nodeSteelballSurfaces->SetName(surfaceName.toStdString().c_str());
        nodeSteelballSurfaces->SetColor(1, 0.0, 0);
        // Add new node
        nodeSteelballSurfaces->SetData(mitkRecontructedSurfaces);
        nodeSteelballSurfaces->SetOpacity(0.8);
        GetDataStorage()->Add(nodeSteelballSurfaces);
    }
}


void NeuroSurgery::OnFitMaskClicked()
{
    /* Fit T1 Brain Mask for MRA */
    // 1. load mask
    // 2. apply mask

    m_Controls.textBrowser_actionLOG->append("Action: MRA Masked.");

    auto inputImage = dynamic_cast<mitk::Image*>(m_Controls.mitkNodeSelectWidget_MRA->GetSelectedNode()->GetData());
    auto maskImage = dynamic_cast<mitk::Image*>(m_Controls.mitkNodeSelectWidget_MRA_Mask->GetSelectedNode()->GetData());

    if (inputImage != nullptr && maskImage != nullptr)
    {
        // Resample the mask to match the input image
        mitk::Image::Pointer resampledMaskImage = ResampleMaskToImage(maskImage, inputImage);

        // Convert MITK images to ITK images
        typedef itk::Image<float, 3> ItkImageType;
        ItkImageType::Pointer itkInputImage;
        mitk::CastToItkImage(inputImage, itkInputImage);

        ItkImageType::Pointer itkResampledMaskImage;
        mitk::CastToItkImage(resampledMaskImage, itkResampledMaskImage);

        // Apply the mask using itk::MaskImageFilter
        typedef itk::MaskImageFilter<ItkImageType, ItkImageType, ItkImageType> MaskFilterType;
        MaskFilterType::Pointer maskFilter = MaskFilterType::New();
        maskFilter->SetInput(itkInputImage);
        maskFilter->SetMaskImage(itkResampledMaskImage);
        maskFilter->SetOutsideValue(0);
        maskFilter->Update();

        ItkImageType::Pointer maskedImage = maskFilter->GetOutput();

        // Convert the masked ITK image back to a MITK image
        mitk::Image::Pointer mitkMaskedImage = mitk::Image::New();
        mitk::CastToMitkImage(maskedImage, mitkMaskedImage);

        // create new node for MaskedImage
        mitk::DataNode::Pointer node = mitk::DataNode::New();
        node->SetData(mitkMaskedImage);

        // Get name
        QString VesselName = m_Controls.lineEdit_MRA_MaskedName->text();
        node->SetName(VesselName.toStdString().c_str());

        node->SetColor(1.0, 0.0, 0.0);
        GetDataStorage()->Add(node);
        m_Controls.textBrowser_actionLOG->append("masked image visualized as node: " + QString::fromStdString("MaskedImage"));
    }
}

mitk::Image::Pointer NeuroSurgery::ResampleMaskToImage(mitk::Image::Pointer maskImage, mitk::Image::Pointer referenceImage)
{
    /* 
    * Resample mask deminsion to its tied pic makes used for MaskFilterType
    */
    // MaskFilterType use for Masked operation
    // But same dememsion inputs are needed

    m_Controls.textBrowser_actionLOG->append("ResampleMask.");

    typedef itk::Image<float, 3> ItkImageType;
    ItkImageType::Pointer itkMaskImage;
    mitk::CastToItkImage(maskImage, itkMaskImage);

    ItkImageType::Pointer itkReferenceImage;
    mitk::CastToItkImage(referenceImage, itkReferenceImage);

    typedef itk::ResampleImageFilter<ItkImageType, ItkImageType> ResampleFilterType;
    ResampleFilterType::Pointer resampleFilter = ResampleFilterType::New();
    resampleFilter->SetInput(itkMaskImage);
    resampleFilter->SetSize(itkReferenceImage->GetLargestPossibleRegion().GetSize());
    resampleFilter->SetOutputSpacing(itkReferenceImage->GetSpacing());
    resampleFilter->SetOutputOrigin(itkReferenceImage->GetOrigin());
    resampleFilter->SetOutputDirection(itkReferenceImage->GetDirection());
    resampleFilter->SetInterpolator(itk::LinearInterpolateImageFunction<ItkImageType, double>::New());
    resampleFilter->SetTransform(itk::IdentityTransform<double, 3>::New());
    resampleFilter->Update();


    mitk::Image::Pointer resampledMaskImage = mitk::Image::New();
    mitk::CastToMitkImage(resampleFilter->GetOutput(), resampledMaskImage);

    return resampledMaskImage;
}

void NeuroSurgery::OnMaskPETRegionClicked()
{
    m_Controls.textBrowser_actionLOG->append("Action: PET Masked.");

    auto petImage = dynamic_cast<mitk::Image*>(m_Controls.mitkNodeSelectWidget_PET->GetSelectedNode()->GetData());
    auto maskImage = dynamic_cast<mitk::Image*>(m_Controls.mitkNodeSelectWidget_PET_Mask->GetSelectedNode()->GetData());

    if (petImage != nullptr && maskImage != nullptr)
    {
        // Resample
        mitk::Image::Pointer resampledMaskImage = ResampleMaskToImage(maskImage, petImage);

        // cast MITK to ITK
        typedef itk::Image<float, 3> ItkImageType;
        ItkImageType::Pointer itkPetImage;
        mitk::CastToItkImage(petImage, itkPetImage);

        ItkImageType::Pointer itkResampledMaskImage;
        mitk::CastToItkImage(resampledMaskImage, itkResampledMaskImage);

        // itk::MaskImageFilter
        typedef itk::MaskImageFilter<ItkImageType, ItkImageType, ItkImageType> MaskFilterType;
        MaskFilterType::Pointer maskFilter = MaskFilterType::New();
        maskFilter->SetInput(itkPetImage);
        maskFilter->SetMaskImage(itkResampledMaskImage);
        maskFilter->SetOutsideValue(0);
        maskFilter->Update();

        ItkImageType::Pointer maskedImage = maskFilter->GetOutput();

        // cast
        mitk::Image::Pointer mitkMaskedImage = mitk::Image::New();
        mitk::CastToMitkImage(maskedImage, mitkMaskedImage);

        // Get name 
        QString PETMaskedName = m_Controls.lineEdit_PET_MaskedName->text();

        // save Masked Image
        mitk::DataNode::Pointer node = mitk::DataNode::New();
        node->SetData(mitkMaskedImage);
        node->SetName(PETMaskedName.toStdString().c_str());

        // 
        node->SetOpacity(0.4);
        GetDataStorage()->Add(node);


        m_Controls.textBrowser_actionLOG->append("Masked PET image visualized as node: " + QString::fromStdString("MaskedPETImage"));
    }
    else
    {
        m_Controls.textBrowser_actionLOG->append("Error: PET or Mask image not found.");
    }
}

void NeuroSurgery::OnCheckPETColorfyClicked()
{
    /* PET data colorfy */
    m_Controls.textBrowser_actionLOG->append("Action: PET Colorfied.");

    // 
    mitk::DataNode::Pointer petNode = m_Controls.mitkNodeSelectWidget_PET_MaskedResult->GetSelectedNode();
    if (!petNode)
    {
        std::cerr << "Could not find selected PET node." << std::endl;
        return;
    }

    // 
    mitk::Image::Pointer petImage = dynamic_cast<mitk::Image*>(petNode->GetData());
    if (!petImage)
    {
        std::cerr << "Selected node is not an image node." << std::endl;
        return;
    }

    // use color JET
    mitk::LookupTable::Pointer lookupTable = mitk::LookupTable::New();
    lookupTable->SetType(mitk::LookupTable::JET);


    // create LookupTableProperty and Apply it to DataNode
    mitk::LookupTableProperty::Pointer lookupTableProperty = mitk::LookupTableProperty::New();
    lookupTableProperty->SetLookupTable(lookupTable);
    petNode->SetProperty("LookupTable", lookupTableProperty);

    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void NeuroSurgery::OnPETResultVisualizeClicked()
{
    /* Show PET */
    m_Controls.textBrowser_actionLOG->append("Action: PET Visulized.");

    mitk::DataStorage::Pointer dataStorage = GetDataStorage();

    mitk::DataStorage::SetOfObjects::ConstPointer allNodes = dataStorage->GetAll();
    for (auto it = allNodes->Begin(); it != allNodes->End(); ++it)
    {
        mitk::DataNode::Pointer node = it->Value();
        if (node.IsNotNull())
        {
            std::string nodeName = node->GetName();

            // find PET_Masked T1w_Raw
            if (nodeName == "PET_Masked" || nodeName == "T1w_Raw")
            {
                node->SetVisibility(true);
            }
            else
            {
                node->SetVisibility(false);
            }
        }
    }

    mitk::RenderingManager::GetInstance()->RequestUpdateAll();

}


void NeuroSurgery::OnLoadFibFileClicked()
{
    /*
    * Get the file path from lineEdit_FIBpath
    */
    m_Controls.textBrowser_actionLOG->append("Action: Load Fib File.");

    QString filePath = m_Controls.lineEdit_FIBpath->text();
    if (filePath.isEmpty())
    {
        std::cerr << "Error: No file path provided." << std::endl;
        return;
    }

    // Read .fib file using VTK
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(filePath.toStdString().c_str());
    reader->Update();

    // Get PolyData object
    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();

    // Check point data
    vtkPointData* pointData = polyData->GetPointData();
    if (pointData)
    {
        for (int i = 0; i < pointData->GetNumberOfArrays(); ++i)
        {
            vtkDataArray* array = pointData->GetArray(i);
            std::cout << "Array Name: " << array->GetName() << std::endl;
            std::cout << "Data Type: " << array->GetDataTypeAsString() << std::endl;
        }
    }

    // Check for color data
    vtkSmartPointer<vtkUnsignedCharArray> fiberColors = vtkUnsignedCharArray::SafeDownCast(polyData->GetPointData()->GetArray("FIBER_COLORS"));
    if (fiberColors && fiberColors->GetNumberOfComponents() == 4)
    {
        std::cout << "Number of Fiber Colors: " << fiberColors->GetNumberOfTuples() << std::endl;
        polyData->GetPointData()->SetScalars(fiberColors);
    }
    else
    {
        std::cerr << "Error: FIBER_COLORS array is missing or not RGBA." << std::endl;
        return;
    }

    // Convert VTK PolyData to MITK Surface
    mitk::Surface::Pointer mitkSurface = mitk::Surface::New();
    mitkSurface->SetVtkPolyData(polyData);

    // Create a new data node
    mitk::DataNode::Pointer node = mitk::DataNode::New();
    node->SetData(mitkSurface);

    // Get the save name from lineEdit_SaveDTI
    QString saveName = m_Controls.lineEdit_SaveDTI->text();
    node->SetName(saveName.toStdString().c_str());

    // Set color properties
    node->SetProperty("scalar visibility", mitk::BoolProperty::New(true));
    node->SetProperty("color mode", mitk::BoolProperty::New(false));

    // Set line rendering properties
    node->SetProperty("line width", mitk::FloatProperty::New(2.0)); // Set line width

    // Attempt to display as lines in 2D view
    node->SetProperty("contour", mitk::BoolProperty::New(true));
    node->SetProperty("contour.line width", mitk::FloatProperty::New(2.0));

    // Set wireframe rendering properties for 3D view
    node->SetProperty("material.wireframe", mitk::BoolProperty::New(true));
    node->SetProperty("material.wireframeLineWidth", mitk::FloatProperty::New(2.0));

    // Add node to data storage
    GetDataStorage()->Add(node);

    // Update rendering
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}


void NeuroSurgery::OnCheckProcessDataClicked() {
    /* Check Processed data */
    // Assume m_Controls has been defined elsewhere and contains relevant UI elements
    m_Controls.textBrowser_actionLOG->append("Action: Check Processed Data.");

    // Get the directory path from the input
    QString directoryPath = m_Controls.lineEdit_ProcessFilePath->text();
    if (directoryPath.isEmpty()) {
        Error(QStringLiteral("Error: No directory path provided."));
        return;
    }

    m_Controls.textBrowser_actionLOG->append("Local Path: " + directoryPath);


    std::filesystem::path niiFilePath;
    bool foundNiiGz = false;

    if (std::filesystem::exists(directoryPath.toStdString()) && std::filesystem::is_directory(directoryPath.toStdString())) {
        for (const auto& entry : std::filesystem::directory_iterator(directoryPath.toStdString())) {
            std::cout << "Current file: " << entry.path() << std::endl;
            std::string filename = entry.path().filename().string();
            if (entry.is_regular_file() && filename.find(".nii.gz") != std::string::npos) {
                std::cout << "Found file: " << entry.path() << std::endl;
                niiFilePath = entry.path();
                foundNiiGz = true;
                break; // find only the first to get base name
            }
        }
    }
    else {
        Error(QStringLiteral("Error: Directory does not exist."));
        return;
    }

    if (!foundNiiGz) {
        Error(QStringLiteral("Error: No .nii.gz file found in the directory."));
        return;
    }

    // Extract the file name prefix
    QFileInfo fileInfo(QString::fromStdString(niiFilePath.string()));
    QString baseName = fileInfo.completeBaseName(); // Get the file name without extension
    int dotIndex = baseName.indexOf('.');
    if (dotIndex != -1) {
        baseName = baseName.left(dotIndex); // Extract characters before the first '.'
        m_Controls.textBrowser_actionLOG->append("Found baseName: " + baseName);
    }

    // Define the file suffixes to check
    QStringList suffixes = { ".mask.nii.gz", ".skull.label.nii.gz", ".hemi.label.nii.gz", ".svreg.label.nii.gz" };
    QList<QLineEdit*> lineEdits = {
        m_Controls.lineEdit_BrainMaskPath,
        m_Controls.lineEdit_BrainLabelPath,
        m_Controls.lineEdit_BrainTissueLabelPath,
        m_Controls.lineEdit_BrainRegionLabelPath
    };

    // Check if files exist and set paths
    for (int i = 0; i < suffixes.size(); ++i) {
        QString fullPath = directoryPath + "/" + baseName + suffixes[i];
        if (QFile::exists(fullPath)) {
            lineEdits[i]->setText(fullPath);
        }
        else {
            lineEdits[i]->setText(QStringLiteral("File not found"));
        }
    }
}

void NeuroSurgery::OnLoadDataButtonClicked()
{
    /*
    * Load Data To MITK
    */
    m_Controls.textBrowser_actionLOG->append("Action: Load Data from Paths.");

    // Get path
    QString brainMaskPath = m_Controls.lineEdit_BrainMaskPath->text();
    QString brainLabelPath = m_Controls.lineEdit_BrainLabelPath->text();
    QString brainTissueLabelPath = m_Controls.lineEdit_BrainTissueLabelPath->text();
    QString brainRegionLabelPath = m_Controls.lineEdit_BrainRegionLabelPath->text();

    // Load Data
    LoadDataToMITK(brainMaskPath, "Brain Mask");
    LoadDataToMITK(brainLabelPath, "Head Label");
    LoadDataToMITK(brainTissueLabelPath, "Brain Tissue Label");
    LoadDataToMITK(brainRegionLabelPath, "Brain Region Label");

    mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(GetDataStorage());
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void NeuroSurgery::LoadDataToMITK(const QString& filePath, const QString& nodeName)
{
    if (filePath.isEmpty() || filePath == "File not found")
    {
        m_Controls.textBrowser_actionLOG->append("Error: Invalid file path for " + nodeName);
        return;
    }

    mitk::Image::Pointer image = mitk::IOUtil::Load<mitk::Image>(filePath.toStdString());
    if (image.IsNull())
    {
        m_Controls.textBrowser_actionLOG->append("Error: Could not load " + nodeName + " from " + filePath);
        return;
    }

    mitk::DataNode::Pointer node = mitk::DataNode::New();
    node->SetData(image);
    node->SetName(nodeName.toStdString());

    GetDataStorage()->Add(node);
    m_Controls.textBrowser_actionLOG->append(nodeName + " loaded successfully.");
}

void NeuroSurgery::ConvertToSegmentation(mitk::DataNode::Pointer imageNode)
{
    /*
    * Convert Label data To Segment
    */
    if (!imageNode)
    {
        m_Controls.textBrowser_actionLOG->append("Error: Invalid image node.");
        return;
    }

    mitk::Image::Pointer image = dynamic_cast<mitk::Image*>(imageNode->GetData());
    if (image.IsNull())
    {
        m_Controls.textBrowser_actionLOG->append("Error: Could not retrieve image from node.");
        return;
    }

    mitk::LabelSetImage::Pointer segmentation = mitk::LabelSetImage::New();
    segmentation->InitializeByLabeledImage(image);

    mitk::DataNode::Pointer segmentationNode = mitk::DataNode::New();
    segmentationNode->SetData(segmentation);
    segmentationNode->SetName(imageNode->GetName() + "_Segmentation");

    // Add  as a child of the original node
    GetDataStorage()->Add(segmentationNode, imageNode);

    m_Controls.textBrowser_actionLOG->append("Converted to segmentation: " + QString::fromStdString(segmentationNode->GetName()));
}


void NeuroSurgery::CreateSmoothedPolygonModel(mitk::DataNode::Pointer segmentationNode)
{
    /*
    * Create Smoothed Polygon Model Based on Segmentation Label
    * Create All Label Values Polygon Model
    */
    if (!segmentationNode)
    {
        m_Controls.textBrowser_actionLOG->append("Error: Invalid segmentation node.");
        return;
    }

    mitk::LabelSetImage::Pointer segmentation = dynamic_cast<mitk::LabelSetImage*>(segmentationNode->GetData());
    if (segmentation.IsNull())
    {
        m_Controls.textBrowser_actionLOG->append("Error: Could not retrieve segmentation from node.");
        return;
    }

    // Get QLineEdit parms
    double threshold = m_Controls.lineEdit_Smooth_Threshold->text().toDouble();
    double targetReduction = m_Controls.lineEdit_Smooth_TargetReduction->text().toDouble();
    int smoothIteration = m_Controls.lineEdit_Smooth_SmoothIteration->text().toInt();
    double smoothRelaxation = m_Controls.lineEdit_Smooth_SmoothRelaxation->text().toDouble();

    // For all labels
    mitk::LabelSet::Pointer labelSet = segmentation->GetActiveLabelSet();
    for (auto it = labelSet->IteratorBegin(); it != labelSet->IteratorEnd(); ++it)
    {
        mitk::Label::Pointer label = it->second;
        if (label.IsNull())
            continue;

        // Create Label Mask
        mitk::Image::Pointer labelImage = segmentation->CreateLabelMask(label->GetValue());

        // ImageToSurfaceFilter
        mitk::ImageToSurfaceFilter::Pointer surfaceFilter = mitk::ImageToSurfaceFilter::New();

        // Default Smooth
        surfaceFilter->SetInput(labelImage);
        surfaceFilter->SetSmooth(true);

        // init param
        surfaceFilter->SetDecimate(mitk::ImageToSurfaceFilter::NoDecimation);
        surfaceFilter->SetThreshold(threshold);
        surfaceFilter->SetTargetReduction(targetReduction);
        surfaceFilter->SetSmoothIteration(smoothIteration);
        surfaceFilter->SetSmoothRelaxation(smoothRelaxation);
        surfaceFilter->Update();

        mitk::Surface::Pointer surface = surfaceFilter->GetOutput();

        mitk::DataNode::Pointer surfaceNode = mitk::DataNode::New();
        if (label->GetName() != "Exterior")
        {
            surfaceNode->SetData(surface);
            surfaceNode->SetName(segmentationNode->GetName() + "_Surface_" + label->GetName());
        }
        
        // set properties
        mitk::Color color = label->GetColor();
        surfaceNode->SetColor(color.GetRed(), color.GetGreen(), color.GetBlue());
        surfaceNode->SetOpacity(0.6);

        // Add  as a child of the original node
        GetDataStorage()->Add(surfaceNode, segmentationNode);
        m_Controls.textBrowser_actionLOG->append("Created smoothed polygon model for label: " + QString::fromStdString(label->GetName()));
    }
}


void NeuroSurgery::OnMaskToSegmentClicked()
{
    /*
    * Convert Mask data To Segment
    */
    m_Controls.textBrowser_actionLOG->append("Action: Process Selected Data.");

    mitk::DataNode::Pointer headNode = GetDataStorage()->GetNamedNode("Head Label");
    mitk::DataNode::Pointer brainNode = GetDataStorage()->GetNamedNode("Brain Tissue Label");
    mitk::DataNode::Pointer regionNode = GetDataStorage()->GetNamedNode("Brain Region Label");

    if (m_Controls.checkBox_Process_Head->isChecked() && headNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Head Data.");
        ConvertToSegmentation(headNode);
    }

    if (m_Controls.checkBox_Process_Brain->isChecked() && brainNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Brain Data.");
        ConvertToSegmentation(brainNode);
    }

    if (m_Controls.checkBox_Process_Region->isChecked() && regionNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Region Data.");
        ConvertToSegmentation(regionNode);
    }

    m_Controls.textBrowser_actionLOG->append("Data processing completed.");
}

void NeuroSurgery::OnSegmentToModelClicked()
{
    /*
    * Convert Segment data To Model
    */
    m_Controls.textBrowser_actionLOG->append("Action: Process Selected Data .");

    mitk::DataNode::Pointer headNode = GetDataStorage()->GetNamedNode("Head Label_Segmentation");
    mitk::DataNode::Pointer brainNode = GetDataStorage()->GetNamedNode("Brain Tissue Label_Segmentation");
    mitk::DataNode::Pointer regionNode = GetDataStorage()->GetNamedNode("Brain Region Label_Segmentation");


    if (m_Controls.checkBox_Process_Head->isChecked() && headNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Head Data.");
        CreateSmoothedPolygonModel(headNode);
    }

    if (m_Controls.checkBox_Process_Brain->isChecked() && brainNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Brain Data.");
        CreateSmoothedPolygonModel(brainNode);
    }

    if (m_Controls.checkBox_Process_Region->isChecked() && regionNode)
    {
        m_Controls.textBrowser_actionLOG->append("Processing Region Data.");
        CreateSmoothedPolygonModel(regionNode);
    }

    m_Controls.textBrowser_actionLOG->append("Data processing completed.");
}

void NeuroSurgery::OnVisualizeButtonClicked()
{
    /*
    * Selected DataNode Visualize
    */
    m_Controls.textBrowser_actionLOG->append("Action: Visualize Processed Data.");

    // Get Basis data
    mitk::DataNode::Pointer headNode = GetDataStorage()->GetNamedNode("Head Label_Segmentation");
    mitk::DataNode::Pointer brainNode = GetDataStorage()->GetNamedNode("Brain Tissue Label_Segmentation");
    mitk::DataNode::Pointer regionNode = GetDataStorage()->GetNamedNode("Brain Region Label_Segmentation");

    // Set visibility according to the selection of the check box
    if (m_Controls.checkBox_Process_Head->isChecked() && headNode)
    {
        bool visible = headNode->IsVisible(nullptr, "visible", true);
        headNode->SetVisibility(!visible);
        SetVisibilityForChildren(headNode, !visible);
        m_Controls.textBrowser_actionLOG->append("Head data " + QString(visible ? "shown." : "hidden."));
    }

    if (m_Controls.checkBox_Process_Brain->isChecked() && brainNode)
    {
        bool visible = brainNode->IsVisible(nullptr, "visible", true);
        brainNode->SetVisibility(!visible);
        SetVisibilityForChildren(brainNode, !visible);
        m_Controls.textBrowser_actionLOG->append("Brain data " + QString(visible ? "shown." : "hidden."));
    }

    if (m_Controls.checkBox_Process_Region->isChecked() && regionNode)
    {
        bool visible = regionNode->IsVisible(nullptr, "visible", true);
        regionNode->SetVisibility(!visible);
        SetVisibilityForChildren(regionNode, !visible);
        m_Controls.textBrowser_actionLOG->append("Region data " + QString(visible ? "shown." : "hidden."));
    }

    // Refresh
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void NeuroSurgery::SetVisibilityForChildren(mitk::DataNode::Pointer parentNode, bool visibility)
{
    /*
    * When setting visibility, apply it to children nodes
    * Also, the Visibility for Different Window
    */
    if (!parentNode)
    {
        m_Controls.textBrowser_actionLOG->append("Error: Invalid parent node.");
        return;
    }

    // Retrieve all child nodes
    mitk::DataStorage::SetOfObjects::ConstPointer children = GetDataStorage()->GetDerivations(parentNode);

    // Get the current render windows
    auto* iRenderWindowPart = this->GetRenderWindowPart();
    if (!iRenderWindowPart)
    {
        m_Controls.textBrowser_actionLOG->append("Error: Render window part not available.");
        return;
    }

    QmitkRenderWindow* renderWindow_sagittal = iRenderWindowPart->GetQmitkRenderWindow("sagittal");
    QmitkRenderWindow* renderWindow_axial = iRenderWindowPart->GetQmitkRenderWindow("axial");
    QmitkRenderWindow* renderWindow_coronal = iRenderWindowPart->GetQmitkRenderWindow("coronal");
    QmitkRenderWindow* renderWindow_3d = iRenderWindowPart->GetQmitkRenderWindow("3d");

    // Get the visibility of the parent node in each view
    bool sagittalVisibility = parentNode->IsVisible(renderWindow_sagittal->GetRenderer());
    bool axialVisibility = parentNode->IsVisible(renderWindow_axial->GetRenderer());
    bool coronalVisibility = parentNode->IsVisible(renderWindow_coronal->GetRenderer());
    bool view3DVisibility = parentNode->IsVisible(renderWindow_3d->GetRenderer());

    for (auto it = children->Begin(); it != children->End(); ++it)
    {
        mitk::DataNode::Pointer childNode = it->Value();
        if (childNode.IsNotNull())
        {
            // Set the visibility of the child node
            childNode->SetVisibility(visibility);

            // Set the visibility of the child node in each view
            childNode->SetVisibility(sagittalVisibility, renderWindow_sagittal->GetRenderer());
            childNode->SetVisibility(axialVisibility, renderWindow_axial->GetRenderer());
            childNode->SetVisibility(coronalVisibility, renderWindow_coronal->GetRenderer());
            childNode->SetVisibility(view3DVisibility, renderWindow_3d->GetRenderer());
        }
    }

    // Update rendering
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void NeuroSurgery::ApplyVisulize()
{
    /* 
    * Apply Visulize State, Different Modality are selectively displayed
    */
    m_Controls.textBrowser_actionLOG->append("Action: Apply Current Visualize Params.");

    auto* iRenderWindowPart = this->GetRenderWindowPart();
    QmitkRenderWindow* renderWindow_sagittal = iRenderWindowPart->GetQmitkRenderWindow("sagittal");
    QmitkRenderWindow* renderWindow_axial = iRenderWindowPart->GetQmitkRenderWindow("axial");
    QmitkRenderWindow* renderWindow_coronal = iRenderWindowPart->GetQmitkRenderWindow("coronal");
    QmitkRenderWindow* renderWindow_3d = iRenderWindowPart->GetQmitkRenderWindow("3d");

    // Get data
    std::map<std::string, mitk::DataNode::Pointer> modalityNodes = {
        {"MRI", GetDataStorage()->GetNamedNode("MRI")},
        {"PET", GetDataStorage()->GetNamedNode("PET")},
        {"DTI", GetDataStorage()->GetNamedNode("DTI")},
        {"Vessel", GetDataStorage()->GetNamedNode("Vessel")}
    };

    // reverse all Modality
    for (const auto& modality : modalityNodes)
    {
        const std::string& modalityName = modality.first;
        mitk::DataNode::Pointer node = modality.second;

        if (node)
        {
            // get Modality selected state
            QCheckBox* modalityCheckBox = m_Controls.groupBox_Visualization->findChild<QCheckBox*>(QString::fromStdString("checkBox_" + modalityName + "_Visulize"));
            if (modalityCheckBox && modalityCheckBox->isChecked())
            {
                // visulize Modality when selected
                node->SetVisibility(true);

                // get checkbox state
                QCheckBox* axialCheckBox = m_Controls.groupBox_Visualization->findChild<QCheckBox*>(QString::fromStdString("checkBox_" + modalityName + "_Axial"));
                QCheckBox* coronalCheckBox = m_Controls.groupBox_Visualization->findChild<QCheckBox*>(QString::fromStdString("checkBox_" + modalityName + "_Cor"));
                QCheckBox* sagittalCheckBox = m_Controls.groupBox_Visualization->findChild<QCheckBox*>(QString::fromStdString("checkBox_" + modalityName + "_Sag"));
                QCheckBox* view3DCheckBox = m_Controls.groupBox_Visualization->findChild<QCheckBox*>(QString::fromStdString("checkBox_" + modalityName + "_3D"));

                // set visulize funcs
                node->SetVisibility(sagittalCheckBox->isChecked(), renderWindow_sagittal->GetRenderer());
                node->SetVisibility(axialCheckBox->isChecked(), renderWindow_axial->GetRenderer());
                node->SetVisibility(coronalCheckBox->isChecked(), renderWindow_coronal->GetRenderer());
                node->SetVisibility(view3DCheckBox->isChecked(), renderWindow_3d->GetRenderer());
            }
            else if (node == nullptr)
            {
                // for node that does not exists in DataStorage, do nothing
            }
            else
            {
                // unvisulize Modality when not selected
                node->SetVisibility(false);
            }
        }
    }

    // Refresh
    mitk::RenderingManager::GetInstance()->InitializeViewsByBoundingObjects(GetDataStorage());
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();

}

void NeuroSurgery::SetNodeVisibilityInRenderWindow(mitk::DataNode::Pointer node, const std::string& windowName, bool visibility)
{
    /*
    * Uesd for Change Window Display State
    * Not for Used
    */
    auto* iRenderWindowPart = this->GetRenderWindowPart();
    if (!iRenderWindowPart)
    {
        std::cerr << "Render window part not available." << std::endl;
        return;
    }

    QmitkRenderWindow* renderWindow = iRenderWindowPart->GetQmitkRenderWindow(windowName.c_str());
    if (!renderWindow)
    {
        std::cerr << "Render window with name " << windowName << " not found." << std::endl;
        return;
    }

    node->SetProperty("visible", mitk::BoolProperty::New(visibility), renderWindow->GetRenderer());
    GetDataStorage()->Add(node);
}


void NeuroSurgery::OnApplyPresetVisulize()
{
    /*
    * Apply Preset Visulize State, Different Modality are selectively displayed
    * Apply the Best Display Param
    */
    m_Controls.textBrowser_actionLOG->append("Action: Apply Preset Visualize Params.");

    // prams init
    struct VisualizationPreset {
        bool mriVisible, mriAxial, mriCoronal, mriSagittal, mri3D;
        bool petVisible, petAxial, petCoronal, petSagittal, pet3D;
        bool dtiVisible, dtiAxial, dtiCoronal, dtiSagittal, dti3D;
        bool vesselVisible, vesselAxial, vesselCoronal, vesselSagittal, vessel3D;
    };

    // prams preset
    VisualizationPreset preset = {
        true, true, true, true, true,  // MRI
        true, true, true, true, false, // PET
        true, true, true, true, true,  // DTI
        true, true, true, true, true   // Vessel
    };

    // change checkbox
    m_Controls.checkBox_MRI_Visulize->setChecked(preset.mriVisible);
    m_Controls.checkBox_MRI_Axial->setChecked(preset.mriAxial);
    m_Controls.checkBox_MRI_Cor->setChecked(preset.mriCoronal);
    m_Controls.checkBox_MRI_Sag->setChecked(preset.mriSagittal);
    m_Controls.checkBox_MRI_3D->setChecked(preset.mri3D);

    m_Controls.checkBox_PET_Visulize->setChecked(preset.petVisible);
    m_Controls.checkBox_PET_Axial->setChecked(preset.petAxial);
    m_Controls.checkBox_PET_Cor->setChecked(preset.petCoronal);
    m_Controls.checkBox_PET_Sag->setChecked(preset.petSagittal);
    m_Controls.checkBox_PET_3D->setChecked(preset.pet3D);

    m_Controls.checkBox_DTI_Visulize->setChecked(preset.dtiVisible);
    m_Controls.checkBox_DTI_Axial->setChecked(preset.dtiAxial);
    m_Controls.checkBox_DTI_Cor->setChecked(preset.dtiCoronal);
    m_Controls.checkBox_DTI_Sag->setChecked(preset.dtiSagittal);
    m_Controls.checkBox_DTI_3D->setChecked(preset.dti3D);

    m_Controls.checkBox_Vessel_Visulize->setChecked(preset.vesselVisible);
    m_Controls.checkBox_Vessel_Axial->setChecked(preset.vesselAxial);
    m_Controls.checkBox_Vessel_Cor->setChecked(preset.vesselCoronal);
    m_Controls.checkBox_Vessel_Sag->setChecked(preset.vesselSagittal);
    m_Controls.checkBox_Vessel_3D->setChecked(preset.vessel3D);

    // apply
    ApplyVisulize();
}