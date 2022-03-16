/*!*******************************************************************************************
 *  \file       tree_file_manager.h
 *  \brief      TreeFileManager definition file.
 *  \details    This file is in charge of the tree files loading and saving.
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
#ifndef TREE_FILE_MANAGER_H
#define TREE_FILE_MANAGER_H

#include <iostream>
#include <fstream>
#include "tree_item.h"
#include <QTextEdit>
#include "behavior_tree.h"
#include "behavior_dialog.h"
#include "yaml-cpp/yaml.h"

#include <aerostack_msgs/CheckBehaviorFormat.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>

/*
namespace Ui {
class TreeFileManager;
}
*/

class TreeFileManager 
{
public:
  TreeFileManager();
  ~TreeFileManager();

public:

  /*!********************************************************************************************************************
  *  \brief      This method saves the current BehaviorTree to the specified file
  **********************************************************************************************************************/
  void saveTree(TreeItem* root_item, std::string save_route);

  /*!********************************************************************************************************************
  *  \brief      This method loads a BehaviorTree from the specified file
  **********************************************************************************************************************/
  TreeItem* loadTree(std::string load_route);
  std::vector<std::string> getBehaviors();
  bool checkParameters(std::string task,std::string parameters);

  bool loadConfiguration(std::string file_path);
  std::string behavior_catalog_path;



private:
  /*!********************************************************************************************************************
  *  \brief      This method is recursively used when saving a BehaviorTree
  **********************************************************************************************************************/
  void recursive_save(TreeItem* item, int id_count, int parent);

  int id_count;
  YAML::Emitter emitter;
  std::string drone_id_namespace;
  std::string check_behavior_format;
  std::vector<std::string> available_behaviors;
  std::string consult_available_behaviors;


  std::vector<std::string> behaviors_loaded;
  std::map<std::string, std::map<std::string,std::vector<std::string>>> behaviors_loaded_complete;

  //ros::NodeHandle nh;
  ros::ServiceClient check_behavior_format_srv;
  ros::ServiceClient consult_available_behaviors_srv;

  aerostack_msgs::BehaviorCommandPriority behavior_msg;
  aerostack_msgs::CheckBehaviorFormat check_format_msg;
  aerostack_msgs::CheckBehaviorFormat::Response check_format_msg_res;
  aerostack_msgs::CheckBehaviorFormat::Request check_format_msg_req;

  droneMsgsROS::ConsultAvailableBehaviors behaviors_message;
  droneMsgsROS::ListOfBehaviors behaviors_list;



};

#endif
