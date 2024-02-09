//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
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

#ifndef sqSlamToolbar_h
#define sqSlamToolbar_h

#include <QToolBar>

class sqSlamToolbar : public QToolBar
{
  Q_OBJECT;
  using Superclass = QToolBar;

public:
  sqSlamToolbar(const QString& title, QWidget* parent = nullptr);
  sqSlamToolbar(QWidget* parent = nullptr);
  ~sqSlamToolbar() override;

private Q_SLOTS:
  void updateEnableState();

private:
  Q_DISABLE_COPY(sqSlamToolbar);
  void constructor();

  struct sqInternals;
  std::unique_ptr<sqInternals> Internals;
};

#endif
