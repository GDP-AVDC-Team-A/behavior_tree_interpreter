/*!*******************************************************************************************
 *  \file       behavior_tree.h
 *  \brief      BehaviorTree definition file.
 *  \details    The BehaviorTree, along with behavior_dialog, behavior_tree_control, tree_item and execution_tree allows a user to create
 *							missions with a tree structure via the HMI.
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
#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <ros/ros.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
#include <aerostack_msgs/StartTask.h>
#include <aerostack_msgs/CheckBehaviorFormat.h>
#include <aerostack_msgs/RequestBehaviorActivation.h> 
#include <aerostack_msgs/TaskCommand.h>
#include <aerostack_msgs/ListOfBeliefs.h>
#include "std_msgs/String.h"
 
#include <QTreeWidget>
#include <QWidget>
#include <QTreeWidgetItem>
#include <QMouseEvent>
#include <QString>
#include <QStringList>
#include <QPoint>
#include <QMenu>
#include <QIcon>
#include <QPixmap>
#include <QBrush>
#include <QColor>
#include <QTextEdit>
#include <QCloseEvent>
#include <QMessageBox>
#include <QErrorMessage>
#include <QApplication>
#include <list>


#include <iostream>

#include "behavior_tree_control.h"
#include "execution_tree.h"
#include "behavior_dialog.h"

class TreeItem;
class ExecutionTree;
class BehaviorTreeControl;
class BehaviorTree : public QTreeWidget
{
	Q_OBJECT

public:
	explicit BehaviorTree(QWidget* parent = 0);
	~BehaviorTree();
  	bool hider2;
  	bool has_root;
        std::string followPathState;
  	std::thread* executing_tree;
  	std::string all_beliefs;
        void continueMission();
	void pauseMission();
 	void pauseMissionCallback(const std_msgs::String::ConstPtr& msg);



  /*!********************************************************************************************************************
  *  \brief      This method adds a top level item to the tree
  **********************************************************************************************************************/
	void addTopLevelItem(TreeItem* );

  /*!***************************BehaviorTree *visualized_tree;*****************************************************************************************
  *  \brief      This method gets the Tree control
  *  \return			Returns the BehaviorTreeControl
  **********************************************************************************************************************/
	BehaviorTreeControl* getVisualizer();

  /*!********************************************************************************************************************
  *  \brief 		 This method gets the ExecutionTree     
  *  \return 		 Returns the ExecutionTree
  **********************************************************************************************************************/
	ExecutionTree * getExecutionTree();
        std::list<int> getIgnoredItems(std::vector<TreeItem*> children, /*std::vector<TreeItem*>::iterator iterator,*/TreeItem * item_to_execute, std::list<int> res);

protected:

  /*!********************************************************************************************************************
   *  \brief      This method expands all the tree's nodes text
   **********************************************************************************************************************/
	void expandText(TreeItem* item);

  /*!********************************************************************************************************************
   *  \brief      This method minimizes all the tree's nodes text
   **********************************************************************************************************************/
	void minimizeText(TreeItem* item);

private:
  ros::NodeHandle n;
  ros::Subscriber list_of_beliefs_sub;
  ros::Subscriber path_state_sub;
  ros::ServiceClient activate_task_srv;
  aerostack_msgs::StartTask::Response res_activate;
  aerostack_msgs::StartTask::Request req_activate;

	TreeItem* root_item;
	TreeItem* parent_item_for_child;
	TreeItem* parent_item_for_sibling;
	TreeItem* parent_item_for_modify;
  TreeItem* root_before;
	BehaviorDialog* window;
	BehaviorTreeControl * control_parent;

  QMenu* contextMenu;
  QPoint point;


  std::string activate_behavior;
  std::string drone_id_namespace;



	bool running = false;
	bool is_menu_created = false;
  bool running_control_panel;
  bool isTextExpanded;

  std::string termination_code;
  std::string start_task;


public:
   ExecutionTree* et;

public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the user right clicks anywhere inside the tree's widget.   
   **********************************************************************************************************************/
	void onCustomContextMenu(const QPoint &);


  /*!********************************************************************************************************************
   *  \brief   This slot is executed when a mission is created with a given root node.
   **********************************************************************************************************************/
	void createMissionByTreeItem(TreeItem* root);

  /*!********************************************************************************************************************
   *  \brief   This slot returns wheter the tree is being executed (true) or not (false)
   **********************************************************************************************************************/
	bool isRunning();

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the BehaviorDialog window is closed.
   **********************************************************************************************************************/
	void windowFinished(TreeItem*);

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the user wants to add a child node
   **********************************************************************************************************************/
	void addChild(const QPoint &);

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the execution of the tree is started
   **********************************************************************************************************************/
	void executeTree();

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the execution of the tree from a certain node is started.
   **********************************************************************************************************************/
	void executeTreeFromItem();

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the tree's execution has finished or has been canceled
   **********************************************************************************************************************/
	void joinExecutionTree();

  /*!********************************************************************************************************************
   *  \brief   This slot is executed whenever the tree's visual representation needs to be refreshed
   **********************************************************************************************************************/
	void updateBackground();

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the tree's execution is canceled
   **********************************************************************************************************************/
	void cancelTree();
	
  /*!********************************************************************************************************************
   *  \brief   This slot expands or minimizes the nodes' text
   **********************************************************************************************************************/
  void expandTreeText(int);

  /*!********************************************************************************************************************
  *  \brief      This method connects the BehaviorTree and ExecutionTree signals to the BehaviorTreeControl slots
  **********************************************************************************************************************/
  void connectExecuteTree(QTreeWidget* behavior_tree, QObject* et,bool from_item);

  /*!********************************************************************************************************************
  *  \brief      This method disconnects the above connections.
  **********************************************************************************************************************/
  void disconnectExecuteTree(QTreeWidget* behavior_tree, QObject* et);

  /*!********************************************************************************************************************
  *  \brief      This method active the custom context menu
  **********************************************************************************************************************/
  void connectCustomContextMenu();

  /*!********************************************************************************************************************
  *  \brief      This method disable the custom context menu
  **********************************************************************************************************************/
  void disconnectCustomContextMenu();

  void setUp();

  void setStyleTreeSheet();

  void setTerminationCode(std::string);

  void terminationMessage();

Q_SIGNALS:

  /*!********************************************************************************************************************
   *  \brief   Emitted when the execution has started
   **********************************************************************************************************************/
	void executionStarted();

  /*!********************************************************************************************************************
   *  \brief   Emitted when a node is removed
   **********************************************************************************************************************/
	void executeRemoveItemAction(const QPoint &);

  /*!********************************************************************************************************************
   *  \brief   Emitted when a child node is added
   **********************************************************************************************************************/
	void executeAddChildAction(const QPoint &);

  /*!********************************************************************************************************************
   *  \brief   Emitted when a sibling node is added
   **********************************************************************************************************************/
	void executeAddSiblingAction(const QPoint &);

  /*!********************************************************************************************************************
   *  \brief   Emitted when a node is being modified
   **********************************************************************************************************************/
	void executeModifyItemWidgetAction(const QPoint &);

  /*!********************************************************************************************************************
   *  \brief   Emitted when the tree's visual representation needs to be refreshed
   **********************************************************************************************************************/
  void update();

  void pauseExecutionSignal();

  /*!********************************************************************************************************************
   *  \brief   Emitted when the tree's execution is canceled
   **********************************************************************************************************************/
  void cancelExecutionSignal();
};

#endif //BEHAVIOR_TREE
