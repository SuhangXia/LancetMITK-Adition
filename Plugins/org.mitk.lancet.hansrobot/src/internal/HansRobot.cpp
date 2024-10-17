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
#include "HansRobot.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

const std::string HansRobot::VIEW_ID = "org.mitk.views.hansrobot";

void HansRobot::callUpdate()
{
	m_ConnectionTab->upDateUi();
}

void HansRobot::SetFocus()
{
}

void HansRobot::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  m_Robot = new LancetJakaRobot();
  m_Camera = new AimCamera();
  m_ConnectionTab = new ConnectionTab(m_Controls, this->GetDataStorage(), m_Robot, m_Camera);

  m_RobotArmRegistrationTab= new RobotArmRegistrationTab(m_Controls, this->GetDataStorage(), m_Robot, m_Camera );
 
}

void HansRobot::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  //// iterate all selected objects, adjust warning visibility
  //foreach (mitk::DataNode::Pointer node, nodes)
  //{
  //  if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
  //  {
  //    m_Controls.labelWarning->setVisible(false);
  //    m_Controls.buttonPerformImageProcessing->setEnabled(true);
  //    return;
  //  }
  //}

  //m_Controls.labelWarning->setVisible(true);
  //m_Controls.buttonPerformImageProcessing->setEnabled(false);

	//Suhang test git
}
