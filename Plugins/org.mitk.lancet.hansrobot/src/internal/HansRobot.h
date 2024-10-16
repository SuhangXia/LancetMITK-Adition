/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef HansRobot_h
#define HansRobot_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>
#include "AimCamera.h"
#include "ui_HansRobotControls.h"
#include <LancetJakaRobot.h>
#include "ConnectionTab.h"
#include "RobotArmRegistrationTab.h"


/**
  \brief HansRobot

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class HansRobot : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;
  void callUpdate();
public slots:
    
protected:
  virtual void CreateQtPartControl(QWidget *parent) override;
 
  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  /// \brief Called when the user clicks the GUI button
  void DoImageProcessing();

  Ui::HansRobotControls m_Controls;
  LancetJakaRobot* m_Robot;
  AimCamera* m_Camera;
  ConnectionTab* m_ConnectionTab;
  RobotArmRegistrationTab* m_RobotArmRegistrationTab;
  
  //std::shared_ptr<RobotArmRegistrationTab> mRobotArmRegistrationTab;
};

#endif // HansRobot_h
