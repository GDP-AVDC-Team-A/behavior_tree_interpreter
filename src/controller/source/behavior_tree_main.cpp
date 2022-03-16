/*!********************************************************************************
 * \brief Main Qt GUI
          Initialize the behavior tree control panel GUI
 * \authors   Daniel Del Olmo, Jorge Luis Pascual, Carlos Valencia, Adrián Cabrera.
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdio.h>
#include <ros/ros.h>
#include "../include/behavior_tree_control_view.h"
#include <QApplication>
#include <signal.h>
#include "../include/global.h"
TreeItem * itemPaused ;
TreeItem * parentItemPaused;
bool paused;
bool expandedText;
bool cancelled;
int visualState;
int lastVisualState;
bool abortingWait;
ExecutionTree * lastExecutionTree;
bool processing_belief_query;
bool doneFirst;
/*****************************************************************************
** Implementation
*****************************************************************************/

void signalhandler(int sig)
{
  if (sig == SIGINT)
  {
    qApp->quit();
  }
  else if (sig == SIGTERM)
  {
    qApp->quit();
  }
}

void spinnerThread(){
  ros::spin();

}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"behavior_tree_control_keyboard"); //ros node started.
  QApplication app(argc, argv);
  BehaviorTreeControlView w(argc, argv);

  w.show();
  std::thread thr(&spinnerThread);

  signal(SIGINT,signalhandler);
  signal(SIGTERM,signalhandler);

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();
  return result;
}
