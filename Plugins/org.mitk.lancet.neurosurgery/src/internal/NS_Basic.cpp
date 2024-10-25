/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"

// Qmitk
#include <QmitkAbstractView.h>
#include <QmitkSingleNodeSelectionWidget.h>
#include <QtWidgets/QFileDialog>
#include <QCheckBox>
#include <QString>

// mitk
#include <mitkImage.h>
#include <mitkDataStorage.h>
#include <mitkDataNode.h>

#include <mitkRenderingManager.h>
#include <mitkImageCast.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateProperty.h>
#include "mitkGizmo.h"

// itk
#include <itkImage.h>
#include <itkBinaryThresholdImageFilter.h>
#include <mitkITKImageImport.h>



void NeuroSurgery::OnCheckDataClicked()
{
    /* Data Test.
    * 1. basic data load
    * 2. device surface load
    */
    m_Controls.textBrowser_actionLOG->append("Action: Check Base data.");
    std::map<std::string, QCheckBox*> imageControls = {
        {"MRI", m_Controls.checkBox_MRI},
        {"PET", m_Controls.checkBox_PET},
        {"CT", m_Controls.checkBox_CT},
        {"MRA", m_Controls.checkBox_MRA},
        {"CTA", m_Controls.checkBox_CTA},
        {"DSA", m_Controls.checkBox_DSA}
    };

    // reverse
    for (const auto& [imageName, checkBox] : imageControls) {
        auto imageNode = GetDataStorage()->GetNamedNode(imageName);
        if (imageNode) {
            checkBox->setChecked(true);
            m_Controls.textBrowser_actionLOG->append(QString("load Image_%1.").arg(QString::fromStdString(imageName)));
        }
    }
}

void NeuroSurgery::OnCheckPETMaskClicked()
{

}



void NeuroSurgery::ParamsInit()
{
    /* NS Param Init Here.
    * 1. basic button value set
    * 2.
    */

    m_CanLoadAlgorithm = false;
    m_ValidInputs = false;
    m_Working = false;

    m_spSelectedTargetData = nullptr;
    m_spSelectedMovingData = nullptr;
    m_spSelectedTargetMaskData = nullptr;
    m_spSelectedMovingMaskData = nullptr;


}

void NeuroSurgery::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
    // surface data Load
    widget->SetDataStorage(GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::Surface>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(false);
    widget->SetEmptyInfo(QString("Please select a surface"));
    widget->SetPopUpTitel(QString("Select surface"));
}

void NeuroSurgery::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
    // load node data
    widget->SetDataStorage(GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::PointSet>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(false);
    widget->SetEmptyInfo(QString("Please select a point set"));
    widget->SetPopUpTitel(QString("Select point set"));
}

void NeuroSurgery::InitImageSelector(QmitkSingleNodeSelectionWidget* widget)
{
    widget->SetDataStorage(GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::Image>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(false);
    widget->SetEmptyInfo(QString("Please select an image"));
    widget->SetPopUpTitel(QString("Select image"));
}

void NeuroSurgery::OnSelectionChanged(berry::IWorkbenchPart::Pointer source, const QList<mitk::DataNode::Pointer>& nodes)
{
    std::string nodeName;

    m_Controls.textBrowser_actionLOG->append("User Click.");
    foreach(mitk::DataNode::Pointer node, nodes)
    {
        if (node.IsNull())
        {
            return;
        }
        node->GetName(nodeName);

        if (node != nullptr)
        {
            current = nodeName;
            m_currentSelectedNode = node;
            m_baseDataToMove = node->GetData();
            if (m_currentSelectedNode == nullptr) {
                std::cerr << "Error: No node selected." << std::endl;
                return;
            }
            else {

                m_Controls.textBrowser_actionLOG->append("Object Selected: " + QString::fromStdString(nodeName));
            }
        }
    }
}

void NeuroSurgery::UseGizmoBtnClicked()
{
    auto modelNode = GetDataStorage()->GetNamedNode(current);

    if (mitk::Gizmo::HasGizmoAttached(modelNode, GetDataStorage()) == 1)
    {
        mitk::Gizmo::RemoveGizmoFromNode(modelNode, GetDataStorage());
        m_Controls.textBrowser_actionLOG->append("Gizmo removed from node: " + QString::fromStdString(current));
    }
    else
    {
        auto gizmoNode = mitk::Gizmo::AddGizmoToNode(modelNode, GetDataStorage());
        m_Controls.textBrowser_actionLOG->append("Gizmo added to node: " + QString::fromStdString(current));

        // 启用旋转模式
        auto gizmo = dynamic_cast<mitk::Gizmo*>(gizmoNode->GetData());
        if (gizmo)
        {
            gizmo->SetAllowRotation(true);
            gizmo->UpdateRepresentation();
            m_Controls.textBrowser_actionLOG->append("Rotation mode enabled for Gizmo.");

            // 保存原始变换
            if (!m_OriginalTransform)
            {
                m_OriginalTransform = modelNode->GetData()->GetGeometry()->GetIndexToWorldTransform()->Clone();
            }
        }
    }
}

void NeuroSurgery::ResetGizmo()
{
    auto modelNode = GetDataStorage()->GetNamedNode(current);
    if (!modelNode)
    {
        m_Controls.textBrowser_actionLOG->append("Error: No node selected to reset Gizmo.");
        return;
    }

    if (m_OriginalTransform)
    {
        // 恢复原始变换
        modelNode->GetData()->GetGeometry()->SetIndexToWorldTransform(m_OriginalTransform);
        mitk::RenderingManager::GetInstance()->RequestUpdateAll();
        m_Controls.textBrowser_actionLOG->append("Gizmo reset to original state.");
    }
    else
    {
        m_Controls.textBrowser_actionLOG->append("Error: No original transform found to reset.");
    }
}