/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef InstantiationCamera_h
#define InstantiationCamera_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>
#include <mitkIRenderWindowPartListener.h>
#include "ui_InstantiationCameraControls.h"
#include <AriemediCamera.h>
#include "CameraConnectionTab.h"

/**
  \brief InstantiationCamera

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class InstantiationCamera : public QmitkAbstractView//, public mitk::IRenderWindowPartListener
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  //void RenderWindowPartActivated(mitk::IRenderWindowPart* renderWindowPart) override;
  //void RenderWindowPartDeactivated(mitk::IRenderWindowPart* renderWindowPart) override;

  /// \brief Called when the user clicks the GUI button

private:
  Ui::InstantiationCameraControls m_Controls;
  AriemediCamera* m_Camera;
  CameraConnectionTab* m_CameraConnectionTab;
};

#endif // InstantiationCamera_h
