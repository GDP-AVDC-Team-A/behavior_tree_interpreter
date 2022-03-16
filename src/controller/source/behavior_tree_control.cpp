/*!********************************************************************************
 * \brief     BehaviorTreeControl
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

#include "../include/behavior_tree_control.h"
#include <iostream>
#include <fstream>
#include <QtConcurrent>
#include <future>
#include "../include/global.h"

std::mutex BehaviorTreeControl::mutexVisual;

 //void  BehaviorTreeControl::closeDialog();
BehaviorTreeControl::BehaviorTreeControl(QWidget *parent) :
QWidget(parent),
ui(new Ui::BehaviorTreeControl)
{
  //initialization of visual components of the GUI.
  visualState=0;//this is initialization to landed state.
  ui->setupUi(this); //connects all ui's triggers
  changeVisual();

  teleoperationActivated=false;
  hider=false;
  hider3=false;
  paused=false;
  pause_continue_changer= false;
  tree = new BehaviorTree(this);
  expand_text_button = new QCheckBox("Show description", this);
  tree_label = new QLabel("Behavior Tree");
  QCheckBox * beliefCheck = new QCheckBox("Show variables",this);
  beliefCheck->setCheckState(Qt::Unchecked);
  beliefs_label = new QLabel("Variables");
  beliefs_text = new QTextEdit(this);
  behavior_viewer = (BehaviorTreeControlView*) parent;
  //Widgets
  QGridLayout * p = new QGridLayout();
  p->addWidget(tree_label, 0, 0, 1, 1);
  p->addWidget(expand_text_button, 0, 1, 1, 1,Qt::AlignLeft);
  p->addWidget(beliefCheck,0,2,1,1,Qt::AlignRight);
  this->QWidget::setWindowTitle(QString::fromStdString("Behavior Tree"));

  ui->groupBox_3->hide();
  ui->groupBox_4->hide(),
  ui->groupBox_5->hide();

  beliefs_text->hide();
  beliefs_label->hide();

  ui->label_wifi->hide();
  ui->value_wifi->hide();

  ui->gridLayout_6->addLayout(p, 0, 0, 1, 1,Qt::AlignTop);
  ui->gridLayout_6->addWidget(tree, 1, 0, 1, 1);
  ui->gridLayout_6->addWidget(beliefs_label, 3, 0, 1, 1);
  ui->gridLayout_6->addWidget(beliefs_text, 4, 0, 1, 1);
  is_takenOff = false;
  correct_format = false;
  beliefs_text->setMaximumHeight(80);
  beliefs_text->setText(QString::fromStdString(""));
  ui->value_battery->setText(QString::number(100) +  "%");
  //Set Flight time
  this->current_time = new QTime(0, 0, 0);
  setTimerInterval(1000);// 1 second = 1000
  flight_timer = new QTimer(this);
  flight_timer->start(1000);

  //Q_EMIT(expand_text_button->stateChanged(2));
  expand_text_button->setCheckState(Qt::Checked);
  ui->groupBox_3->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");
  ui->groupBox_4->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");
  ui->groupBox_5->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");

  //Default configuration folder
  homePath = QDir::homePath().toStdString();

  //Establishment of connections
  setUp(); 

  //Settings keyboard control
  setKeyboard();

  //Load tree
  loadTreeFile();

  //Connects
  QObject::connect(ui->execute_tree_button, SIGNAL(clicked()), this, SLOT(executeTreeMission()));
  QObject::connect(ui->emergency_land_tree_button, SIGNAL(clicked()), this, SLOT(landTreeMission()));
  QObject::connect(ui->finish_teleoperation_button, SIGNAL(clicked()), this, SLOT(landTreeMission()));
  QObject::connect(flight_timer, SIGNAL(timeout()), this, SLOT(setFlightTime()));
  QObject::connect(expand_text_button, SIGNAL(stateChanged(int)), tree, SLOT(expandTreeText(int)));
  QObject::connect(ui->teleoperation_button,SIGNAL(clicked()), this,SLOT(enableManualControl()));
  QObject::connect(beliefCheck,SIGNAL(stateChanged(int)), this,SLOT(expandVariable(int)));
  QObject::connect(ui->abort_tree_button, SIGNAL(clicked()), this, SLOT(abortTreeMission()));
  QObject::connect(ui->pause_mission, SIGNAL(clicked()), this, SLOT(pauseMission()));

}

/*------------------------------------------------------------
--------------------------------------------------------------
                      Destructor
--------------------------------------------------------------  
------------------------------------------------------------*/
BehaviorTreeControl::~BehaviorTreeControl()
{
  delete ui;
  delete flight_timer;
  delete current_time;
  delete tree;
  delete expand_text_button;
  delete tree_label;
  delete beliefs_label;
  delete beliefs_text;
}

/*------------------------------------------------------------
--------------------------------------------------------------
                Getters and setters
--------------------------------------------------------------
------------------------------------------------------------*/
void BehaviorTreeControl::expandVariable(int state){//this method makes possible to expand the variables section
  if (state==0){
    beliefs_text->hide();
    beliefs_label->hide();
  }
  else{
    beliefs_text->show();
    beliefs_label->show();
  }

}
void BehaviorTreeControl::enableManualControl(){//this method enable to activate the manual control
  if (hider == true){
    visualState=4;
    changeVisual();
    teleoperationActivated=false;
    hider=false;

  }
  else{
    teleoperationActivated=true;
    hider=true;
    if(visualState==2){
      lastVisualState=visualState;
      visualState=5;
      changeVisual();
    }
    if(visualState==3){
      lastVisualState=visualState;
      visualState=7;
      changeVisual();
    }
  }
  
}
BehaviorTree* BehaviorTreeControl::getBehaviorTree()
{
  return this -> tree;
}

std::string BehaviorTreeControl::getText()
{
  std::string result = this->beliefs_text->toPlainText().toUtf8().constData();
  return result;
}

void BehaviorTreeControl::setText(std::string texto)
{
  text = QString(texto.c_str());
}

void BehaviorTreeControl::setTimerInterval(double ms)
{
  d_interval = qRound(ms);
  if (d_interval >= 0 )
    d_timerId = startTimer(d_interval);
}

void BehaviorTreeControl::setFlightTime()
{
  if (is_takenOff)
  {
    this->current_time->setHMS(this->current_time->addSecs(+1).hour(), this->current_time->addSecs(+1).minute(), this->current_time->addSecs(+1).second());
    ui->value_flight_time->setText(this->current_time->toString());
  }
}

void BehaviorTreeControl::setStartBlockingTextInput()//this is used at the beginning of the mission.
{
  visualState=1;
  changeVisual();
  pause_continue_changer= false;
  beliefs_text->setReadOnly(true);
  

  if (lastVisualState!=3 && lastVisualState !=2){

  lastVisualState=visualState;
  visualState=8;
  changeVisual();
  visualState=lastVisualState;

  
}
}

void BehaviorTreeControl::setStopBlockingTextInput() //this is used at the end of the mission.
{
  if (visualState!=2 && visualState!=3 && visualState!=6){

    lastVisualState=9;
    visualState=9;
    changeVisual();
    lastVisualState=0;
    visualState=0;
    changeVisual();
  }
  if (visualState==6 ){

    visualState=-1;
  }
  if(visualState!=3){
    tree->connectCustomContextMenu();

  }
  beliefs_text->setReadOnly(false);

 if (visualState!=-1 ){
     missionStateMsgs.data=false;
    mission_state_publ.publish(missionStateMsgs);
 }
   
}

void BehaviorTreeControl::update()//this is an important method because it makes possible the process of using variables.
{
  beliefs_text->setText(text);
  processing_belief_query = false;
}

void BehaviorTreeControl::setUp()
{   ros::NodeHandle nh("~");
  //Nodes
nh.param<std::string>("start_task", start_task, "start_task");
nh.param<std::string>("list_of_running_task", list_of_running_tasks, "list_of_running_tasks");
nh.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
nh.param<std::string>("behavior_tree_execute", behavior_tree_execute, "behavior_tree_execute");
nh.param<std::string>("mission_state", mission_state_topic, "mission_state");
nh.param<std::string>("mission_configuration_folder", mission_configuration_folder, "$(env AEROSTACK_STACK)/configs/drone1");
nh.param<std::string>("check_behavior_format", check_behavior_format, "check_behavior_format");
if (!nh.getParam("drone_driver_sensor_battery", drone_driver_sensor_battery))
  drone_driver_sensor_battery = "/sensor_measurement/battery_state";

/*if (!nh.getParam("wifiIsOk", wifi_connection_topic))
  wifi_connection_topic = "wifiIsOk";*/
ros::param::get("~pose_ref_topic_name", pose_ref_topic_name);
    if ( pose_ref_topic_name.length() == 0)
    {
        pose_ref_topic_name="motion_reference/pose";
    }

  //Service communications
check_behavior_format_srv=nh.serviceClient<aerostack_msgs::CheckBehaviorFormat>("/"+drone_id_namespace+"/"+check_behavior_format);
activate_task_srv = nh.serviceClient<aerostack_msgs::StartTask>("/" + drone_id_namespace +  "/" + start_task);

std::cout << "Mission configuration folder: " << mission_configuration_folder << std::endl;

  //Subscribers
list_of_behaviors_sub = nh.subscribe("/" + drone_id_namespace +  "/" + list_of_running_tasks, 1000, &BehaviorTreeControl::newBehaviorCallback, this);
battery_subs = nh.subscribe("/" + drone_id_namespace + "/" + drone_driver_sensor_battery, 1000, &BehaviorTreeControl::batteryCallback, this);
//wificonnection_subs = nh.subscribe("/" + drone_id_namespace + "/"  + wifi_connection_topic, 1, &BehaviorTreeControl::wifiConnectionCheckCallback, this);
ui->value_vehicle->setText( QString::fromUtf8(drone_id_namespace.c_str()));

  //Publishers
mission_state_publ=nh.advertise<std_msgs::Bool>("/"+ drone_id_namespace +  "/" +mission_state_topic ,1, true);
pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/"+pose_ref_topic_name, 1, true);

}

/*------------------------------------------------------------
--------------------------------------------------------------
                    File handlers
--------------------------------------------------------------
------------------------------------------------------------*/
void BehaviorTreeControl::loadTreeFile()
{
  file_route = mission_configuration_folder + "/behavior_tree_mission_file.yaml";

  std::ifstream aux_file(file_route);

  if (aux_file.fail())
  {
    windowManager('c', "Loading mission error", "behavior_tree_mission_file.yaml is not defined in configuration folder. Please create it using the Behavior Tree Editor.");
  }

  else
  {
    TreeFileManager* tfm = new TreeFileManager();;
    root_node = tfm->loadTree(file_route);
    delete tfm;
    //if (root_node != nullptr && checkTree(root_node))
    //{
      correct_format = true;
      tree->createMissionByTreeItem(root_node);
      tree->show();
      tree_label->show();
    //}
  }
}

/*------------------------------------------------------------
--------------------------------------------------------------
                Check format tree
--------------------------------------------------------------
------------------------------------------------------------*/
bool BehaviorTreeControl::checkTree(TreeItem *item)//This method check if there is any error in the mission.
{
  bool isCorrect = true;
  bool correctChildren = true;

  NodeType item_nodetype = item->getNodeType();

  std::string node_type_name = "";
  std::string incorrect_children = " has an incorrect number of children. ";
  std::string correct_error = ". Please correct this error before trying again.";

  switch(item_nodetype)
  {
    //The following nodes can't have children
    case NodeType::TASK:
    node_type_name = "TASK";

    case NodeType::QUERY: 
    node_type_name = node_type_name.empty()? "QUERY" : node_type_name;

    case NodeType::ADD_BELIEF:
    node_type_name = node_type_name.empty()? "ADD_BELIEF" : node_type_name;

    case NodeType::REMOVE_BELIEF:
    {
      node_type_name = node_type_name.empty()? "REMOVE_BELIEF" : node_type_name;

      if(item->childCount() != 0)
      {
        windowManager('c', "Bad tree format", "The node '" + item->getNodeName() + "'" + incorrect_children + "A node of '" + node_type_name + "' type can not have children" + correct_error);
        correctChildren = false;
      }

      if(item_nodetype == NodeType::TASK)
      {
        //std::cout << "estan en el checker"<< std::endl;
        //behavior_msg.name = item->getBehaviorType();
        //behavior_msg.arguments = this->processData(item->getNodeAttributes());
        //check_format_msg_req.behavior = behavior_msg;
        //check_behavior_format_srv.call(check_format_msg_req, check_format_msg_res);


        if(!check_format_msg_res.ack)
        {
          error_behavior = "The node '" + item->getNodeName() + "' has invalid arguments.\n";
          error_behavior += check_format_msg_res.behavior_error_code;
          windowManager('c', "Bad tree format", error_behavior);
          correctChildren = false;
        }
      } 
      break;
    }

    //The following nodes can have just one child
    case NodeType::SUCCEEDER:
    node_type_name = "SUCCEEDER";

    case NodeType::INVERTER:
    node_type_name = node_type_name.empty()? "INVERTER" : node_type_name;

    case NodeType::REPEATER:
    {
      node_type_name = node_type_name.empty()? "REPEATER" : node_type_name;

      if(item->childCount() != 1)
      {
        windowManager('c', "Bad tree format", "The node '" + item->getNodeName() + "'" + incorrect_children + "A node of '" + node_type_name + "' type can have just one child" + correct_error);
        correctChildren = false;
      }
      break;
    }

    //The following nodes must have at least one child
    case NodeType::SEQUENCE:
    node_type_name = "SEQUENCE";

    case NodeType::PARALLEL:
    node_type_name = node_type_name.empty()? "PARALLEL" : node_type_name;

    case NodeType::REPEAT_UNTIL_FAIL:
    node_type_name = node_type_name.empty()? "REPEAT_UNTIL_FAIL" : node_type_name;

    case NodeType::SELECTOR:
    {
      node_type_name = node_type_name.empty()? "SELECTOR" : node_type_name;

      if(item->childCount() < 1)
      {
        windowManager('c', "Bad tree format", "The node '" + item->getNodeName() + "'" + incorrect_children + "A node of '" + node_type_name + "' type must have at least one child" + correct_error);
        correctChildren = false;
      }
      break;
    }
  }

  if(!correctChildren)
    return false;

  //Checking each of it's children recursively
  //If current checked node is wrong then stop check 
  if(item->childCount()>0)
  {
    for(int i = 0; i < item->childCount(); i++)
    {
      isCorrect = isCorrect && checkTree(item->child(i));
    }
  }

  return isCorrect;
}

std::string BehaviorTreeControl::outsideProcessData(std::string raw_arguments)
{
  std::string result = this->processData(raw_arguments);
  return result;
}

std::string BehaviorTreeControl::processData(std::string raw_arguments) // this method looks for variables in the arguments
{

  std::string text = getText();
  if (text != "") 
  {
    YAML::Node node_beliefs = YAML::Load(text);
    for(YAML::const_iterator it=node_beliefs.begin();it!=node_beliefs.end();++it) 
    {
      std::string name = SUBSTITUTION_S + it->first.as<std::string>();
      if (raw_arguments.find(name)) 
      {
        std::string data = processType(it);
        boost::replace_all(raw_arguments, name, data);
      }
    }
  }
  return raw_arguments;
}

std::string BehaviorTreeControl::processType(YAML::const_iterator it) //this method processes the arguments.
{
  std::string res = "";
  switch (it->second.Type()) 
  {
    case YAML::NodeType::Scalar: 
    {
      res = it->second.as<std::string>();
      break;
    }
    case YAML::NodeType::Sequence: 
    {
      std::vector<double> vec = it->second.as<std::vector<double>>();
      std::ostringstream oss;
      if (!vec.empty())
      {
        std::copy(vec.begin(), vec.end()-1,
          std::ostream_iterator<double>(oss, ","));
        oss << vec.back();
      }
      res = "[" + oss.str() + "]";
      break;
    }
  }
  return res;
}

/*------------------------------------------------------------
--------------------------------------------------------------
                    Button actions
--------------------------------------------------------------
------------------------------------------------------------*/
void BehaviorTreeControl::landTreeMission()
{ 

aerostack_msgs::StartTask::Request msg;
aerostack_msgs::StartTask::Response res;
aerostack_msgs::TaskCommand behavior;
behavior.name = "LAND";
behavior.priority = 4;
msg.task = behavior;
activate_task_srv.call(msg,res);
if(!res.ack)
  std::cout << res.error_message << std::endl;
if (visualState==3 || visualState==2 )//if (paused)
{
 tree->et->setColor(itemPaused,"#c41306");
 tree->et->setColorBackground(itemPaused,"#ffffff");
 while(true){
 	TreeItem * i = itemPaused->getParent(); 
 	tree->et->setColor(i,"#c41306");
    tree->et->setColorBackground(i,"#ffffff");
    if (i->getParent()==NULL){
    	break;
    }
    itemPaused=i;
 }

 visualState=0;
 lastVisualState=0;
}
if (visualState==5 ||visualState==7){
  tree->et->setColor(itemPaused,"#c41306");
 tree->et->setColorBackground(itemPaused,"#ffffff");
 while(true){
 	TreeItem * i = itemPaused->getParent(); 
 	tree->et->setColor(i,"#c41306");
    tree->et->setColorBackground(i,"#ffffff");
    if (i->getParent()==NULL){
    	break;
    }
    itemPaused=i;
 }
 visualState=0;
 lastVisualState=0;
}
setStopBlockingTextInput();
lastVisualState=visualState;
QTimer::singleShot(2400, this, SLOT(waitTimeForStart()));
lastVisualState=visualState;
visualState=8;
changeVisual();
visualState=lastVisualState;
tree->connectCustomContextMenu();

}
void BehaviorTreeControl::waitTimeForStart(){

  lastVisualState=visualState;
  visualState=9;
  changeVisual();
  visualState=lastVisualState;

}

void BehaviorTreeControl::executeTreeMission()//this method executes the mission and calls executetree to do it
{ 

  doneFirst=false;
  tree->executeTree();
  
  missionStateMsgs.data=true;
  mission_state_publ.publish(missionStateMsgs);

}
void BehaviorTreeControl::closeDialog(){    
 BehaviorTree * myTree = getBehaviorTree();
 myTree->cancelTree();
}
void BehaviorTreeControl::pauseMission(){
  if (!pause_continue_changer)
  { 
    lastVisualState=visualState;
    visualState=8;
    changeVisual();
    visualState=lastVisualState;


    lastVisualState = visualState;
    tree->disconnectCustomContextMenu();
    visualState=3;
    changeVisual();
    pause_continue_changer= true;
    tree->pauseMission();
  }
  else{
    lastVisualState=visualState;
    visualState=6;
    changeVisual();
    pause_continue_changer= false;
    tree->continueMission();
  }
}
void BehaviorTreeControl::abortTreeMission()
{ 
  BehaviorTree * myTree = getBehaviorTree();
  lastVisualState= visualState;
  visualState=2;
  changeVisual();
  myTree->cancelTree();
  /*std::thread thr(&BehaviorTreeControl::closeDialog,this);
  thr.join();*/
}
void BehaviorTreeControl::changeVisual(){// this is the method that changes the visual components . This will be explain in the file explanation.txt
sleep(0.5);
  switch(visualState){
    case 0://inicial landed
    ui->execute_tree_button->show();
    ui->pause_mission->hide();
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->teleoperation_button->hide();
    ui->abort_tree_button->hide();
    ui->emergency_land_tree_button->hide();
    ui->emergency_land_tree_button->setText("Emergency land");
    ui->finish_teleoperation_button->hide();
    ui->groupBox_3->hide();
    ui->groupBox_4->hide();
    ui->groupBox_5->hide();
    ui->keys_teleop->hide();
    ui->teleoperation_button->setText("Change to teleoperation");
    hider=false;
    if (lastVisualState==3){
      tree->et->setColor(itemPaused,"#c41306");
      tree->et->setColorBackground(itemPaused,"#ffffff");
    }
    break;
    
    case 1://started
    ui->abort_tree_button->show();
    ui->emergency_land_tree_button->show();
    ui->pause_mission->setText("Pause mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->pause_mission->show();
    if (lastVisualState!=2){
      ui->execute_tree_button->hide();
    }
    ui->teleoperation_button->setText("Change to teleoperation");                
    ui->teleoperation_button->hide();
    ui->finish_teleoperation_button->hide();
    break;

    case 2://aborting
    tree->connectCustomContextMenu();
    ui->abort_tree_button->hide();
    ui->emergency_land_tree_button->hide();
    ui->finish_teleoperation_button->setText("Emergency land");
    ui->finish_teleoperation_button->show();
    ui->pause_mission->hide();
    ui->teleoperation_button->show();
    break ;

    case 3://paused
    tree->connectCustomContextMenu();
    ui->emergency_land_tree_button->setText("Emergency land");
    ui->emergency_land_tree_button->show();
    ui->abort_tree_button->show();
    ui->teleoperation_button->show();
    ui->pause_mission->setText("Continue mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/play.png"));
    ui->pause_mission->show();
    ui->finish_teleoperation_button->hide();
    break;
    
    case 4://teleoperation off
    ui->groupBox_3->hide();
    ui->groupBox_4->hide();
    ui->groupBox_5->hide();
    ui->teleoperation_button->setText("Change to teleoperation");
    visualState=lastVisualState;
    ui->keys_teleop->hide();
    changeVisual();
    break;

    case 5://teleoperation on cancelled
    tree->disconnectCustomContextMenu();
    ui->groupBox_3->show();
    ui->groupBox_4->show();
    ui->groupBox_5->show();
    ui->teleoperation_button->setText("Return to mission control");
    ui->keys_teleop->show();
    break;
    
    case 6://continue
    ui->pause_mission->setText("Pause mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->pause_mission->show();
    ui->teleoperation_button->hide();
    break;

    case 7://teleoperation on paused
    tree->disconnectCustomContextMenu();
    ui->groupBox_3->show();
    ui->groupBox_4->show();
    ui->groupBox_5->show();
    ui->keys_teleop->show();
    ui->emergency_land_tree_button->hide();
    ui->pause_mission->hide();
    ui->abort_tree_button->hide();
    ui->teleoperation_button->setText("Return to mission control");
    ui->finish_teleoperation_button->setText("Finish teleoperation and land");
    ui->finish_teleoperation_button->show();
    break;
    
    case 8://disable
    ui->teleoperation_button->setEnabled(false);
    ui->emergency_land_tree_button->setEnabled(false);
    ui->pause_mission->setEnabled(false);
    ui->execute_tree_button->setEnabled(false);
    ui->abort_tree_button->setEnabled(false);
    ui->finish_teleoperation_button->setEnabled(false);
    break;

    case 9://enable
    ui->teleoperation_button->setEnabled(true);
    ui->emergency_land_tree_button->setEnabled(true);
    ui->pause_mission->setEnabled(true);
    ui->execute_tree_button->setEnabled(true);
    ui->abort_tree_button->setEnabled(true);
    ui->finish_teleoperation_button->setEnabled(true);
    break;

    case 10://finish
    lastVisualState=0;
    visualState=0;
    changeVisual();
    break;
  }

}

/*------------------------------------------------------------
--------------------------------------------------------------
                  Keyboard manager
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::setKeyboard()
{ 
  isAKeyPressed = false;

  setFocusPolicy(Qt::StrongFocus);
  acceptedKeys.insert(0x01000012, false); //Tecla UP
  acceptedKeys.insert(0x01000013, false); //Tecla DOWN
  acceptedKeys.insert(0x01000014, false); //Tecla LEFT
  acceptedKeys.insert(0x01000015, false); //Tecla RIGHT
  acceptedKeys.insert(0x51, false); //Tecla Q
  acceptedKeys.insert(0x41, false); //Tecla A
  acceptedKeys.insert(0x54, false); //Tecla T
  acceptedKeys.insert(0x59, false); //Tecla Y
  acceptedKeys.insert(0x48, false); //Tecla H
  acceptedKeys.insert(0x5a, false); //Tecla Z
  acceptedKeys.insert(0x58, false); //Tecla X
  acceptedKeys.insert(0x52, false); //Tecla R
}

void BehaviorTreeControl::keyPressEvent(QKeyEvent *e)
{

  if(teleoperationActivated){


    QWidget::keyPressEvent(e);

    if(!isAKeyPressed && acceptedKeys.contains(e->key()))
    {
      isAKeyPressed = true;
      switch(e->key())
      {
        case Qt::Key_R:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x52, true);
            onResetCommandButton();
          }
          break;
        }
        case Qt::Key_T:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x54, true);
            onTakeOffButton();

          }
          break;
        }
        case Qt::Key_Y:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x59, true);
            onLandButton();

          }
          break;
        }
        case Qt::Key_H:
        {
          acceptedKeys.insert(0x48, true);
          aerostack_msgs::StartTask::Request msg;
          aerostack_msgs::StartTask::Response res;
          aerostack_msgs::TaskCommand behavior;
          behavior.name = "HOVER";
          behavior.priority = 2;
          msg.task = behavior;
          activate_task_srv.call(msg,res);
          if(!res.ack)
            std::cout << res.error_message << std::endl;

          break;
        }
        case Qt::Key_Right:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000014, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.priority = 2;
            behavior.parameters = "direction: \"RIGHT\"\nspeed: 0.4";
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
   
          break;
        
        }
        case Qt::Key_Left:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000012, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"LEFT\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Down:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000015, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"BACKWARD\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Up:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000013, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"FORWARD\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Q:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x51, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"UP\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_A:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x41, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"DOWN\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Z:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x5a, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "ROTATE";
            behavior.parameters = "relative_angle: +179";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_X:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x58, true);
            aerostack_msgs::StartTask::Request msg;
            aerostack_msgs::StartTask::Response res;
            aerostack_msgs::TaskCommand behavior;
            behavior.name = "ROTATE";
            behavior.parameters = "relative_angle: -179";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;
         
          }
          break;
        }
      }
    }
  }
}

void BehaviorTreeControl::onTakeOffButton()
{
  aerostack_msgs::StartTask::Request msg;
  aerostack_msgs::StartTask::Response res;
  aerostack_msgs::TaskCommand behavior;
  behavior.name = "TAKE_OFF";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::onLandButton()
{


  aerostack_msgs::StartTask::Request msg;
  aerostack_msgs::StartTask::Response res;
  aerostack_msgs::TaskCommand behavior;
  behavior.name = "LAND";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::onResetCommandButton()
{
  /** NOTE:
  Shows strange behaviour when the drone has been ordered to rotate previously,
  and a stabilize command was not issued after the rotation.
 */
  aerostack_msgs::StartTask::Request msg;
  aerostack_msgs::StartTask::Response res;
  aerostack_msgs::TaskCommand behavior;
  behavior.name = "ROTATE";
  behavior.parameters = "angle: 0";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::keyReleaseEvent(QKeyEvent *e)
{
  if(e->isAutoRepeat() || !acceptedKeys.contains(e->key()))
  {
    isAKeyPressed = false;
    e->ignore();
  }
  else if(acceptedKeys.contains(e->key()) && acceptedKeys.value(e->key()))
  {
    acceptedKeys.insert(e->key(), false);
    aerostack_msgs::StartTask::Request msg;
    aerostack_msgs::StartTask::Response res;
    aerostack_msgs::TaskCommand behavior;
    behavior.name = "HOVER";
    msg.task = behavior;
    behavior.priority = 1;
    if(e->key() != Qt::Key_Y && e->key() != Qt::Key_T && e->key() != Qt::Key_R)
      if(!res.ack)
        std::cout << res.error_message << std::endl;
      isAKeyPressed = false;
      QWidget::keyReleaseEvent(e);
    }
    else
    {
      isAKeyPressed = false;
      e->ignore();
      QWidget::keyReleaseEvent(e);
    }
  }

/*------------------------------------------------------------
--------------------------------------------------------------
                Messages output manager
--------------------------------------------------------------
------------------------------------------------------------*/
  void BehaviorTreeControl::windowManager(char type, std::string title, std::string message)
  {
    QMessageBox *msg_error = new QMessageBox(QMessageBox::Critical, title.c_str(), message.c_str(), QMessageBox::Ok,this);
    msg_error->setWindowFlags(msg_error->windowFlags() | Qt::WindowStaysOnTopHint);
    msg_error->exec();

  }

/*------------------------------------------------------------
--------------------------------------------------------------
                      Callback methods
--------------------------------------------------------------
------------------------------------------------------------*/
  void BehaviorTreeControl::newBehaviorCallback(const aerostack_msgs::ListOfRunningTasks &msg)
  {

    if (msg.list_of_running_tasks.size() != 0)
    {
      for (int i = 0; i < msg.list_of_running_tasks.size(); i++)
      {
        if (msg.list_of_running_tasks[i].task_command.name == "TAKE_OFF")
        {
          this->current_time->setHMS(00, 00, 00);
          ui->value_flight_time->setText(this->current_time->toString());
          is_takenOff = true;
        }
        else if (msg.list_of_running_tasks[i].task_command.name == "LAND")
          is_takenOff = false;
      }
    //Battery
      if (battery_msgs.percentage*100 <= 25.0 && battery_msgs.percentage*100 != 0)
      {
        QPalette* palette = new QPalette();
        palette->setColor(QPalette::WindowText, Qt::red);
        ui->value_battery->setPalette(*palette);
      }
      ui->value_battery->setText(QString::number(battery_msgs.percentage*100) +  "%");

    //Connection
      /*if (is_wifi_connected)
        ui->value_wifi->setText("connected");
      else
        ui->value_wifi->setText("disconnected");*/
    }
  }

  void BehaviorTreeControl::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  {
    battery_msgs = *msg;
    if (battery_msgs.percentage*100 <= 25.0 && battery_msgs.percentage*100 != 0)
    {
      QPalette* palette = new QPalette();
      palette->setColor(QPalette::WindowText, Qt::red);
      ui->value_battery->setPalette(*palette);
    }
    ui->value_battery->setText(QString::number(battery_msgs.percentage*100) +  "%");
  }

  /*void BehaviorTreeControl::wifiConnectionCheckCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    is_wifi_connected = msg->data;
    if (is_wifi_connected)
      ui->value_wifi->setText("connected");
    else
      ui->value_wifi->setText("disconnected");
  }*/

