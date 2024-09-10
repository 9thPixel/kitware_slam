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

#ifndef sqInitializeDialogReaction_h
#define sqInitializeDialogReaction_h

#include <pqReaction.h>

class sqInitializeDialog;

class sqInitializeDialogReaction : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;

public:
  sqInitializeDialogReaction(QAction* parent);

  /**
   * Shows the initialize dialog
   */
  static void showInitializeDialog();

protected:
  /**
   * Called when the action is triggered.
   */
  void onTriggered() override;

private:
  Q_DISABLE_COPY(sqInitializeDialogReaction)

  static QPointer<sqInitializeDialog> Dialog;
};

#endif