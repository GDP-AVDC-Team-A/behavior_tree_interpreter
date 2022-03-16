/*!*******************************************************************************************
 *  \file       execution_tree.h
 *  \brief      ExecutionTree definition file.
 *  \details    In charge of the trees execution. 
 *  \author     Jorge Luis Pascual, Carlos Valencia.
 *  \copyright Copyright 2017 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************************/
#ifndef EXECUTION_TREE_H
#define EXECUTION_TREE_H
 
#include <iostream>
#include <stdio.h>
#include <thread>
#include <stdlib.h>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/RequestBehaviorDeactivation.h>
#include <aerostack_msgs/TaskStopped.h>
#include <aerostack_msgs/TaskCommand.h>
#include <aerostack_msgs/StartTask.h>
#include <aerostack_msgs/BehaviorCommandPriority.h> //Mensaje con el argumento y el nombre
#include <aerostack_msgs/BehaviorActivationFinished.h> //Recibir terminacion
#include <aerostack_msgs/QueryBelief.h>
#include <droneMsgsROS/ConsultBelief.h> 
#include <droneMsgsROS/AddBelief.h>  
#include <droneMsgsROS/RemoveBelief.h> 
#include <droneMsgsROS/InitiateBehaviors.h> 
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include "yaml-cpp/yaml.h"
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <boost/algorithm/string/replace.hpp>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>
#include "behavior_tree.h"
#include "tree_item.h"
#include <QMessageBox>
#include <QWidget>
#include <list>

static const std::string COLOR_BLUE = "#74b6f7";
static const std::string COLOR_GREEN = "#006400";
static const std::string COLOR_RED = "#c41306";
static const std::string COLOR_BLACK = "#000000";
static const std::string COLOR_PURPLE = "#ce42f5";
static const std::string COLOR_CIAN = "#02E5E7";
static const std::string SUBSTITUTION_S = "+";

class BehaviorTreeVisualizer;
class BehaviorTree;
class ExecutionTree : public QObject
{

  Q_OBJECT

public:
  explicit ExecutionTree(BehaviorTree * tree = 0);
  ~ExecutionTree();
  std::thread thr2;
int mutex_int;
bool processDataArgs(std::string query);

  /*!********************************************************************************************************************
  *  \brief      This method starts the execution of the tree, starting from a given node
  **********************************************************************************************************************/
  void executeTree(TreeItem* item, std::list<int> ignored_items_list);//int ignored_items, int secondChilds);

  /*!********************************************************************************************************************
  *  \brief      This method executes a node. It is a recurrent method.
  **********************************************************************************************************************/
  int executeItem(TreeItem* item, std::list<int> ignored_items_list); //int ignored_items,int secondChilds);

  /*!********************************************************************************************************************
  *  \brief      This method gets the return value of the execution of a tree.
  **********************************************************************************************************************/
  int getReturnedValue();

    /*!********************************************************************************************************************
  *  \brief      This method gets the BehaviorTree that created this object.
  **********************************************************************************************************************/
  BehaviorTree* getBehaviorTree();

  /*!********************************************************************************************************************
  *  \brief      This method checks wether the tree is currently being executed or not
  **********************************************************************************************************************/
  bool isRunning();

  /*!********************************************************************************************************************
  *  \brief      This method is used whenever a parallel node has to be executed.
  **********************************************************************************************************************/
  static void executeParallelTree(ExecutionTree* et, TreeItem* item,std::list<int> ignored_items_list);// int ignored_items, int secondChilds);

  static std::mutex mutex;

  /*!********************************************************************************************************************
  *  \brief      This method sets the color of a row corresponding to a node
  **********************************************************************************************************************/
  void setColor(TreeItem* item, std::string color);

  void setColorBackground(TreeItem* item, std::string color);

  void resetColor(TreeItem* item);
  TreeItem *lastItem;
  int used;


private:

  /*!********************************************************************************************************************
  *  \brief      This method is used as a condition to wait or continue the execution of a tree. 
  **********************************************************************************************************************/
  void behaviorCompletedCallback(const aerostack_msgs::TaskStopped &msg);

  /*!********************************************************************************************************************
  *  \brief      This method processes the arguments of a node and replaces the variables with the correct argument if needed.
  **********************************************************************************************************************/
  std::string processData(std::string raw_arguments);

  /*!********************************************************************************************************************
  *  \brief      This method processes the argument of a query node to format it correctly.
  **********************************************************************************************************************/
  std::string processQueryData(YAML::Node query);

  /*!********************************************************************************************************************
  *  \brief      This method helps the processQueryData method by processing the argument data differently depending on which type it is.
  **********************************************************************************************************************/
  std::string processType(YAML::const_iterator it);

  /*!********************************************************************************************************************
  *  \brief      This method updates the variables textbox content of BehaviorTreeVisualizer
  **********************************************************************************************************************/
  void setText(std::string str);

	bool running;
	int tree_returned_value;
	bool waiting;


  bool behavior_value;
  TreeItem *actual_item;
  BehaviorTree *behavior_tree;

	std::vector<ExecutionTree*> trees_in_parallel;
	std::vector<std::thread*> threads;

	static std::condition_variable condition;
	std::mutex behavior_mutex;
	std::condition_variable behavior_condition_variable;

  ros::NodeHandle n;
  ros::Subscriber behavior_sub;
  ros::ServiceClient activate_task_srv;
  ros::ServiceClient initiate_behaviors_srv;
  ros::ServiceClient execute_query_srv;
  ros::ServiceClient add_belief_srv;
  ros::ServiceClient remove_belief_srv;
  ros::ServiceClient cancel_behavior_srv;
 // aerostack_msgs::RequestBehaviorActivation::Response res_activate;
 // aerostack_msgs::RequestBehaviorActivation::Request req_activate;
  aerostack_msgs::StartTask::Request req_activate;
  aerostack_msgs::StartTask::Response res_activate;
  droneMsgsROS::InitiateBehaviors msg_initiate;

  std::string topic_behavior_completed;
  std::string activate_behavior;
  std::string initiate_behaviors;
  std::string execute_query;
  std::string add_belief;
  std::string remove_belief;
  std::string error_code;
 std::string drone_id_namespace;
  std::string cancel_behavior;
  std::string task_stopped;
  std::string start_task;

  //Eliminar
	std::string tis;

public Q_SLOTS:
  void cancelExecution();
  void pauseExecution();

Q_SIGNALS:

  /*!********************************************************************************************************************
  *  \brief      This signal is emitted when the tree is finished executing
  **********************************************************************************************************************/
  void finished();


  /*!********************************************************************************************************************
  *  \brief      This signal is emitted whenever the tree's representation needs to be refreshed
  **********************************************************************************************************************/
  void update();

  /*!********************************************************************************************************************
  *  \brief      This signal is emitted to update the text of the variables textbox 
  **********************************************************************************************************************/
  void updateText();
};

#endif // EXECUTION_TREE
