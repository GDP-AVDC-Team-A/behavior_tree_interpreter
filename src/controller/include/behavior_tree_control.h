/*!*******************************************************************************************
 *  \file      behavior_tree_control.h
 *  \brief     Behavior Tree Control definition file.
 *  \details   The control panel shows drone-related information and manages the buttons for interaction with the drone.
 *  \author    Abraham Carrera, Daniel Del Olmo
********************************************************************************************/

#ifndef BEHAVIOR_TREE_CONTROL_H
#define BEHAVIOR_TREE_CONTROL_H
#define CTE_POSE (1.00)
 
#include <ros/ros.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
#include <droneMsgsROS/ListOfBehaviors.h>
#include <aerostack_msgs/ListOfRunningTasks.h>
#include <aerostack_msgs/TaskCommand.h>
#include <aerostack_msgs/StartTask.h>
#include "droneMsgsROS/battery.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>

#include <QWidget>
#include <QTime>
#include <QTimer>
#include <QObject>
#include <QTextEdit> 
#include <QLabel>
#include <QCheckBox>
#include <QGridLayout>
#include <QString>
#include <QMessageBox>
#include <QSplitter>
#include <QDir>
#include <string>

#include <thread>
#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "yaml-cpp/yaml.h"

#include <mutex>
#include "behavior_tree.h"
#include "tree_item.h"
#include "tree_file_manager.h"
#include "behavior_tree_control_view.h"

#include "ui_behavior_tree_control.h"

namespace Ui {
  class BehaviorTreeControl;
}

class BehaviorTreeControlView;

class BehaviorTreeControl : public QWidget
{
  Q_OBJECT

public:

bool teleoperationActivated;
  void closeDialog();
  //Constructor & Destructor
  explicit BehaviorTreeControl(QWidget *parent);
  ~BehaviorTreeControl();
  QCheckBox* expand_text_button;
  bool correct_format;
  bool hider;
  bool hider3;
  bool pause_continue_changer;
  void changeVisual();
  static std::mutex mutexVisual;
//this is for the teleoperation
  geometry_msgs::PoseStamped motion_reference_pose_msg; 
  ros::Publisher pose_reference_publ;
  std::string pose_ref_topic_name;
//
/*!********************************************************************************************************************
   *  \brief      This method returns the current variables' textbox content
   **********************************************************************************************************************/
  std::string getText();

  /*!********************************************************************************************************************
   *  \brief      This method copies the given text to the private variable 'text' which conforms the variables' textbox content
   **********************************************************************************************************************/
  void setText(std::string texto);

  /*!********************************************************************************************************************
   *  \brief   This slot checks if an item is correctly defined
   **********************************************************************************************************************/
  bool checkItem(TreeItem* item);

  /*!********************************************************************************************************************
   *  \brief   This slot checks if a tree is correctly defined
   **********************************************************************************************************************/
  bool checkTree(TreeItem* item);

  /*!********************************************************************************************************************
   *  \brief   This method launch a window which contains detailed error
   **********************************************************************************************************************/
  void windowManager(char type, std::string title, std::string message);

 


private:
  Ui::BehaviorTreeControl* ui;

  ros::ServiceClient activate_task_srv;
  ros::ServiceClient check_behavior_format_srv;
  ros::NodeHandle n;
  QKeyEvent *lastEvent;
  ros::Subscriber list_of_behaviors_sub;
  ros::Subscriber behavior_event_sub;
  ros::Subscriber battery_subs;
  //ros::Subscriber wificonnection_subs;
  ros::Publisher mission_state_publ;
  ros::Publisher behavior_command_publ;

  std_msgs::Bool missionStateMsgs;
  aerostack_msgs::RequestBehaviorActivation::Request req;

  aerostack_msgs::RequestBehaviorActivation::Response res;
  aerostack_msgs::BehaviorCommandPriority behavior_msg;
  aerostack_msgs::CheckBehaviorFormat::Request check_format_msg_req;
  aerostack_msgs::CheckBehaviorFormat::Response check_format_msg_res;
  //droneMsgsROS::battery battery_msgs;
  sensor_msgs::BatteryState battery_msgs;
  BehaviorTree* tree;
  BehaviorTreeControlView* behavior_viewer;
  TreeItem* root_node;

  QTimer* flight_timer; //Timer that sends the timeout signal every second.
  QTime* current_time;
  QString text;
  QLabel* tree_label;
  QLabel* beliefs_label;
  QTextEdit* beliefs_text;

  QMap<int, bool> acceptedKeys;
  QMessageBox error_message;
  QMessageBox* msg_error;
  QAbstractButton* m_save_button;
  QAbstractButton* m_cancel_button;
  QAbstractButton* m_dont_save_button;

  std::string check_behavior_format;
  std::string activate_behavior;
  std::string drone_id_namespace;
  std::string list_of_running_tasks;
  std::string behavior_event;
  std::string drone_driver_sensor_battery;
  //std::string wifi_connection_topic;
  std::string behavior_tree_execute_str;
  std::string behavior_tree_execute;
  std::string mission_configuration_folder;
  std::string folder_name;
  std::string homePath;
  std::string default_folder;
  std::string file_route;
  std::string behavior_command;
  std::string mission_state_topic;
  std::ifstream aux_file;
  std::string error_behavior;
  std::string my_stack_directory;

  std::string start_task;

  int d_interval;
  int d_timerId;
  bool is_takenOff;
  //bool is_wifi_connected;
  bool isAKeyPressed;

  /*!********************************************************************************************************************
   *  \brief      This method returns the BehaviorTree
   **********************************************************************************************************************/
  BehaviorTree* getBehaviorTree();

  /*!********************************************************************************************************************
   *  \brief     This method initializes the timer that informs about the time the drone has been flying.
   *  \param ms  The interval at which the timer works.
  *********************************************************************************************************************/
  void setTimerInterval(double ms);

  /*!********************************************************************************************************************
  *  \brief      This method is the responsible for seting up connections.
  *********************************************************************************************************************/
  void setUp();

  void setKeyboard();

  /*!********************************************************************************************************************
   *  \brief      This method is executed when the user wants to load a tree from a file.
   **********************************************************************************************************************/
  void loadTreeFile();

  //void behaviorEventCallBack(const aerostack_msgs::BehaviorEvent & msg);

  /*!**********************************************************************************************************************
   *  \brief     This callback is executed when the list of behaviors is modified.
   *  \details   It's purpose is to control the state of the drone and the actions the GUI should allow the user to execute.
   *************************************************************************************************************************/
  void newBehaviorCallback(const aerostack_msgs::ListOfRunningTasks & msg);

  /*!************************************************************************
   *  \brief     Receives the battery status.
   ***************************************************************************/
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

  /*!************************************************************************
   *  \brief     Receives the state of the WiFi connection
   **************************************************************************/
  //void wifiConnectionCheckCallback(const std_msgs::Bool::ConstPtr& msg);

  /*!********************************************************************************************************************
  *  \brief      This method takes action when the user wants to make the drone to take off.
  *********************************************************************************************************************/
  void onTakeOffButton();

  /*!********************************************************************************************************************
  *  \brief      This method takes action when the user wants to make the drone to land.
  *********************************************************************************************************************/
  void onLandButton();

  /*!********************************************************************************************************************
  *  \brief      This method takes action when the user wants to reset the drone.
  *  \details    Resets angles (yaw).
  *********************************************************************************************************************/
  void onResetCommandButton();

  void keyPressEvent(QKeyEvent* e);
  void keyReleaseEvent(QKeyEvent* e);





  public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief     This method takes action when the user wants to make the drone to land.
   *********************************************************************************************************************/
  void landTreeMission();
  void expandVariable(int);
  void pauseMission();
  void waitTimeForStart();
  /*!********************************************************************************************************************
   *  \brief      This method starts a behavior tree execution
   *********************************************************************************************************************/
  void executeTreeMission();
  void enableManualControl();

  /*!********************************************************************************************************************
   *  \brief      This method cancels the current behavior tree execution
   *********************************************************************************************************************/
  void abortTreeMission();


  /*!********************************************************************************************************************
   *  \brief      This method informs about the time the drone has been flying.
   *********************************************************************************************************************/
  void setFlightTime();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when a tree is executed. It disables the variables textbox editing.
   **********************************************************************************************************************/
  void setStartBlockingTextInput();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when a tree's execution is finished or canceled. It enables the variables textbox editing.
   **********************************************************************************************************************/
  void setStopBlockingTextInput();
  
  /*!********************************************************************************************************************
   *  \brief   Emitted when the tree's visual representation needs to be refreshed
   **********************************************************************************************************************/
  void update();

  std::string outsideProcessData(std::string raw_arguments);

  /*!********************************************************************************************************************
  *  \brief      This method processes the arguments of a node and replaces the variables with the correct argument if needed.
  **********************************************************************************************************************/
  std::string processData(std::string raw_arguments);

  /*!********************************************************************************************************************
  *  \brief      This method helps the processQueryData method by processing the argument data differently depending on which type it is.
  **********************************************************************************************************************/
  std::string processType(YAML::const_iterator it);

  Q_SIGNALS:
  void finished();

};

#endif // BEHAVIOR_TREE_CONTROL_H
