/*!********************************************************************************
 * \brief     BehaviorTreeEditorView
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
#include "../include/behavior_tree_control_view.h"
 
BehaviorTreeControlView::BehaviorTreeControlView(int argc, char** argv, QWidget *parent) :
QWidget(parent),
ui(new Ui::BehaviorTreeControlView)
{
  //window always on top
  sleep(1.5);	
  setWindowIcon(QIcon(":/img/img/tree_control.png"));
  setWindowTitle("BEHAVIOR TREE CONTROL");
  Qt::WindowFlags flags = windowFlags();
  setWindowFlags(flags | Qt::WindowStaysOnTopHint);

  ui->setupUi(this); //connects all ui's triggers

  //Add the control panel widget
  behavior_tree_control = new BehaviorTreeControl(this); 
  ui->gridLayout->addWidget(behavior_tree_control);

  //Settings Widget
  setWidgetDimensions();

  //Establishment of connections
  setUp();
}


BehaviorTreeControlView::~BehaviorTreeControlView()
{
  delete ui;
  delete behavior_tree_control;
}

/*------------------------------------------------------------
--------------------------------------------------------------
                Getters and setters
--------------------------------------------------------------
------------------------------------------------------------*/
BehaviorTreeControl* BehaviorTreeControlView::getBehaviorTreeControl()
{
  return behavior_tree_control;
}

void BehaviorTreeControlView::setWidgetDimensions(){
  //namespace pt = boost::property_tree;

  QScreen *screen = QGuiApplication::primaryScreen();
  QRect  screenGeometry = screen->geometry();

  int y0 = screenGeometry.height()/2;
  int x0 = screenGeometry.width()/2;
  int height =heightV;//root.get<int>("BEHAVIOR_TREE_INTERPRETER.height");
  int width= widthV;//root.get<int>("BEHAVIOR_TREE_INTERPRETER.width");

  this->resize(width,height);
  this->move(x0+/*root.get<int>("BEHAVIOR_TREE_INTERPRETER.position.x")*/position_x,y0+/*root.get<int>("BEHAVIOR_TREE_INTERPRETER.position.y")*/position_y);
}

void BehaviorTreeControlView::setUp()
{ 
  ros::NodeHandle n("~");
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("start_task", start_task, "start_task");


  //Service comunications
  activate_task_srv=n.serviceClient<aerostack_msgs::RequestBehaviorActivation>("/"+drone_id_namespace+"/"+start_task);


  //Subscribers

  //n.param<std::string>("window_event_topic", window_event_topic, "window_event");


  //Subscribers
 //window_event_sub = n.subscribe("/" + drone_id_namespace +  "/" + window_event_topic, 10, &BehaviorTreeControlView::windowOpenCallback, this);

  //Publishers
  //window_event_pub = n.advertise<aerostack_msgs::WindowEvent>("/" + drone_id_namespace +  "/" + window_event_topic , 1, true);
}


/*------------------------------------------------------------
--------------------------------------------------------------
                Handlers for the main widget
--------------------------------------------------------------
------------------------------------------------------------*/
void BehaviorTreeControlView::killMe()
{
#ifdef Q_OS_WIN
  enum { ExitCode = 0 };
  ::TerminateProcess(::GetCurrentProcess(), ExitCode);
#else
  qint64 pid = QCoreApplication::applicationPid();
  QProcess::startDetached("kill -9 " + QString::number(pid));
#endif // Q_OS_WIN
}
