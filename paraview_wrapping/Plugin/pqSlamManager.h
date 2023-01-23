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

#ifndef pqSlamManager_h
#define pqSlamManager_h

#include <QObject>

#include <vtkEventQtSlotConnect.h>

#include <vector>

class pqPipelineSource;
class vtkSlam;

class pqSlamManager : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

public:
  pqSlamManager(QObject* p = 0);
  ~pqSlamManager();

  // Callback for startup.
  void onStartup();

  // Callback for shutdown.
  void onShutdown();

protected Q_SLOTS:
  void onSourceAdded(pqPipelineSource* src);
  void onDataUpdated(pqPipelineSource* src);

private:
  Q_DISABLE_COPY(pqSlamManager)
};

#endif
