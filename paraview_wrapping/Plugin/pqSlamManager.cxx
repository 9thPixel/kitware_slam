//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Timothée Couble (Kitware SAS)
//         Julia Sanchez (Kitware SAS)
// Creation date: 2023-01-23
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include "pqSlamManager.h"

// Qt Includes.
#include <QAction>
#include <QMenu>
#include <QMessageBox>
#include <QSignalMapper>

// ParaView Includes.
#include <pqApplicationCore.h>
#include <pqPipelineSource.h>
#include <pqProxy.h>
#include <pqServerManagerModel.h>
#include <pqVCRController.h>

// VTK Includes
#include <vtkSMSourceProxy.h>
#include <vtkSMProperty.h>
#include <vtkSMProxy.h>
#include <vtkSlam.h>

#include <iostream>

//-----------------------------------------------------------------------------
pqSlamManager::pqSlamManager(QObject* p /*=0*/)
  : QObject(p)
{
}

//-----------------------------------------------------------------------------
pqSlamManager::~pqSlamManager() = default;

//-----------------------------------------------------------------------------
void pqSlamManager::onStartup()
{
  std::cout << "Message from pqSlamManager: Application Started" << std::endl;
  pqServerManagerModel* smModel = pqApplicationCore::instance()->getServerManagerModel();
  this->connect(
    smModel, SIGNAL(sourceAdded(pqPipelineSource*)), SLOT(onSourceAdded(pqPipelineSource*)));
}

//-----------------------------------------------------------------------------
void pqSlamManager::onShutdown()
{
  std::cout << "Message from pqSlamManager: Application Shutting down" << std::endl;
}

void pqSlamManager::onSourceAdded(pqPipelineSource* src)
{
  vtkSMSourceProxy* sourceProxy = src->getSourceProxy();
  pqServerManagerModel* model = pqApplicationCore::instance()->getServerManagerModel();
  if (QString(sourceProxy->GetXMLGroup()) == "filters" &&
    QString(sourceProxy->GetXMLName()) == "SlamOnline")
  {
    QObject::connect(
      src, SIGNAL(dataUpdated(pqPipelineSource*)), this, SLOT(onDataUpdated(pqPipelineSource*)));
  }
}

void pqSlamManager::onDataUpdated(pqPipelineSource* src)
{
  vtkSMProxy* proxy = src->getProxy();
  if (!proxy)
  {
    return;
  }
  vtkSlam* filter = vtkSlam::SafeDownCast(proxy->GetClientSideObject());
  if (!filter)
  {
    return;
  }

  if (filter->GetOutputKeypointsInWorldCoordinates())
  {
    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Question);
    msgBox.setText("The SLAM detected a loop closure at ...");
    msgBox.setInformativeText("Do you want to improve the SLAM by closing the loop?");
    msgBox.setDetailedText("More information about this loop closure.");
    msgBox.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Apply);
    int ret = msgBox.exec();
    switch (ret)
    {
      case QMessageBox::Apply:
        filter->SetOutputKeypointsInWorldCoordinates(false);
        break;
      case QMessageBox::Cancel:
        filter->SetOutputKeypointsInWorldCoordinates(false);
        break;
      default:
        // should never be reached
        break;
    }
  }
}
