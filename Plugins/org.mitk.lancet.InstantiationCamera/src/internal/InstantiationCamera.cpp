/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "InstantiationCamera.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

const std::string InstantiationCamera::VIEW_ID = "org.mitk.views.instantiationcamera";

void InstantiationCamera::SetFocus()
{
}

void InstantiationCamera::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  m_Camera = new AriemediCamera();
  m_CameraConnectionTab = new CameraConnectionTab(m_Controls, GetDataStorage(), m_Camera, parent);
}

void InstantiationCamera::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  // iterate all selected objects, adjust warning visibility
  foreach (mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
    {
      return;
    }
  }
}
