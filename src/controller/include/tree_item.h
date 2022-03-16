/*!*******************************************************************************************
 *  \file       tree_item.h
 *  \brief      TreeItem definition file.
 *  \details    The BehaviorTree, along with behavior_dialog, behavior_tree_visualizer, behavior_tree and execution_tree allows a user to create
 *                          missions with a tree structure via the HMI. 
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
#ifndef TREE_ITEM_H
#define TREE_ITEM_H

#include <QWidget>
#include <QTreeWidget>
#include <QPoint>
#include <iostream>
#include <QString>
#include <QStringList>
#include <unistd.h>
#include <QIcon>
#include <QBrush>
#include <QColor>
#include <QPixmap>
#include <QColor>
#include <QBrush>
#include <QPropertyAnimation>
#include <sstream>
#include <string>

class BehaviorTree;

enum NodeType 
{
	SEQUENCE,
	TASK,
	SELECTOR,
	PARALLEL,
	SUCCEEDER,
	INVERTER,
	QUERY,
	REPEAT_UNTIL_FAIL,
  REPEATER,
  ADD_BELIEF,
  REMOVE_BELIEF
};

class TreeItem : public QTreeWidgetItem
{

public:   
    explicit TreeItem(TreeItem *parent = 0, NodeType node_type = NodeType::SEQUENCE, std::string = "LAND");
    ~TreeItem();

  /*!********************************************************************************************************************
  *  \brief      This method adds a child to a node
  **********************************************************************************************************************/
    void addChild(TreeItem * item);

  /*!********************************************************************************************************************
  *  \brief      This method removes a child from a node
  **********************************************************************************************************************/
    void removeChild(TreeItem * item);

  /*!********************************************************************************************************************
  *  \brief      This method returns the "index"th child of a node 
  **********************************************************************************************************************/
    TreeItem * child(int index);

  /*!********************************************************************************************************************
  *  \brief      This method returns the number of children a node has
  **********************************************************************************************************************/
    int childCount();

  /*!********************************************************************************************************************
  *  \brief      This method returns the NodeType of a node
  **********************************************************************************************************************/
    NodeType getNodeType();

  /*!********************************************************************************************************************
  *  \brief      This method returns the BehaviorType of a node
  **********************************************************************************************************************/
    std::string getBehaviorType();

  /*!********************************************************************************************************************
  *  \brief      This method returns the parent node of a node
  **********************************************************************************************************************/
    TreeItem * getParent();

  /*!********************************************************************************************************************
  *  \brief      This method returns the children vector of a node
  **********************************************************************************************************************/
    std::vector<TreeItem*> getChildren();

  /*!********************************************************************************************************************
  *  \brief      This method returns a node's name
  **********************************************************************************************************************/
    std::string getNodeName();

  /*!********************************************************************************************************************
  *  \brief      This method returns only the name given by the user of a node
  **********************************************************************************************************************/
    std::string getPartialNodeName();

  /*!********************************************************************************************************************
  *  \brief      This method returns the attributes or arguments of a node
  **********************************************************************************************************************/
    std::string getNodeAttributes();


    std::string getTerminationResult();

  /*!********************************************************************************************************************
  *  \brief      This method returns whether a node is the tree's root or not
  **********************************************************************************************************************/
    bool isRoot();

  /*!********************************************************************************************************************
  *  \brief      This method returns whether a Behavior type node is recurrent or not
  **********************************************************************************************************************/
    bool isRecurrent();

  /*!********************************************************************************************************************
  *  \brief      This method returns wheter a Behavior type node is activated or not
  **********************************************************************************************************************/
    bool isActivated();

  /*!********************************************************************************************************************
  *  \brief      This method sets a node to be the root of the tree
  **********************************************************************************************************************/
    void setRoot(bool isRoot);

  /*!********************************************************************************************************************
  *  \brief      This method sets a node's name
  **********************************************************************************************************************/
    void setNodeName(std::string text);

  /*!********************************************************************************************************************
  *  \brief      This method sets a node's partial name
  **********************************************************************************************************************/
    void setPartialNodeName(std::string text);

  /*!********************************************************************************************************************
  *  \brief      This method sets the arguments of a node
  **********************************************************************************************************************/
    void setNodeAttributes(std::string text);

  /*!********************************************************************************************************************
  *  \brief      This method sets the NodeType of a node
  **********************************************************************************************************************/
    void setNodeType(NodeType nodetype);

  /*!********************************************************************************************************************
  *  \brief      This method sets the BehaviorType of a node
  **********************************************************************************************************************/
    void setBehaviorType(std::string behaviortype);

  /*!********************************************************************************************************************
  *  \brief      This method sets the background color of a node
  **********************************************************************************************************************/
    void setColor(std::string color);

  /*!********************************************************************************************************************
  *  \brief      This method sets the background color of a node
  **********************************************************************************************************************/
    void setColorBackground(std::string color);

  /*!********************************************************************************************************************
  *  \brief      This method sets a node to be/not be recurrent
  **********************************************************************************************************************/
    void setRecurrent(bool recurrent);

  /*!********************************************************************************************************************
  *  \brief      This method sets a node to be/not be activated
  **********************************************************************************************************************/
    void setActivate(bool activate);


    void setTerminationResult(std::string texto);
    void blinkingItem();


  /*!********************************************************************************************************************
  *  \brief      This method removes a node
  **********************************************************************************************************************/
    void removeItemWidget();

  /*!********************************************************************************************************************
  *  \brief      This method returns the string equivalent to a NodeType
  **********************************************************************************************************************/
    std::string nodeTypeToString(NodeType node);

  /*!********************************************************************************************************************
  *  \brief      This method returns the NodeType equivalent to a string
  **********************************************************************************************************************/
    NodeType stringToNodeType(std::string str);

  /*!********************************************************************************************************************
  *  \brief      This method modifies a node's data with the given data
  **********************************************************************************************************************/
    void modifyNode(std::string node_name, NodeType node_type, std::string behavior_type, bool is_recurrent, bool activate, std::string arguments);

private:
	std::string attributes;
	std::string node_name;
  std::string partial_node_name;
  std::string termination_result = "";

	bool root;
  bool recurrent;
  bool activate;

	TreeItem *parent_node;

	NodeType node_type;
	std::string behavior_type;
	std::vector<TreeItem*> children;

	TreeItem * parent;
};

#endif // TREE_ITEM
