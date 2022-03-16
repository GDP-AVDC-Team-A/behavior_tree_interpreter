/*!********************************************************************************
 * \brief TreeItem
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
#include "../include/tree_item.h"
#include "../include/behavior_tree.h" //BehaviorTree class is included here to prevent recursive includes with behavior_tree.
#include "../include/global.h"
TreeItem::TreeItem(TreeItem *parent, NodeType node_type, std::string behavior_type) : QTreeWidgetItem(parent)
{
	this->parent_node = parent;
	this->node_type = node_type;
	this->behavior_type = behavior_type;
	this->root = false;
	this->setFlags(Qt::ItemIsEnabled);
	QPixmap icono_pixmap = QPixmap(":/images/images/action.png");
	QIcon icono_action = QIcon(icono_pixmap);
	this->setIcon(0,icono_action);
}

void TreeItem::modifyNode(std::string node_name, NodeType node_type, std::string behavior_type, bool is_recurrent, bool activate, std::string arguments)
{
	this->node_name = node_name;
	this->node_type = node_type;
	this->behavior_type = behavior_type;
	this->recurrent = is_recurrent;
	this->activate = activate;
	this->attributes = arguments;

	QPixmap icon_pixmap = QPixmap(":/images/images/action.png");

	switch(node_type) 
	{
		case NodeType::SEQUENCE:
		{
			this->partial_node_name = " [Execute all actions in sequence until one fails]";
			icon_pixmap = QPixmap(":/images/images/sequence.png");
			break;
		}
		case NodeType::SELECTOR: 
		{
			this->partial_node_name = " [Execute all actions in sequence until one succeeds]";
			icon_pixmap = QPixmap(":/images/images/selector.png");
			break;
		}
		case NodeType::SUCCEEDER: 
		{
			this->partial_node_name = " [Inverts the result of the child node]";
			icon_pixmap = QPixmap(":/images/images/succeeder.png");
			break;
		}
		case NodeType::INVERTER: 
		{
			this->partial_node_name = " [Inverts the result of the child node]";
			icon_pixmap = QPixmap(":/images/images/inverter.png");
			break;
		}
		case NodeType::QUERY: 
		{
			this->partial_node_name = " [Query to the belief memory]";
			icon_pixmap = QPixmap(":/images/images/query.png");
			break;
		}
		case NodeType::ADD_BELIEF: 
		{
			this->partial_node_name = " [Add belief to the belief memory]";
			icon_pixmap = QPixmap(":/images/images/tree_action.png");
			break;
		}
		case NodeType::REMOVE_BELIEF: 
		{
			this->partial_node_name = " [Remove belief from the belief memory]";
			icon_pixmap = QPixmap(":/images/images/tree_action.png");
			break;
		}
		case NodeType::PARALLEL: 
		{
			YAML::Node args = YAML::Load(this->attributes);
			partial_node_name = " [Execute actions in parallel. Node succeeds if ";
			partial_node_name = partial_node_name + args["n"].as<std::string>();
			partial_node_name = partial_node_name + " child node(s) succeed(s). Fails otherwise]";
			icon_pixmap = QPixmap(":/images/images/parallel.png");
			break;
		}
		case NodeType::REPEAT_UNTIL_FAIL: 
		{
			YAML::Node args = YAML::Load(this->attributes);
			partial_node_name = " [Execute all actions in sequence in a loop until one fails (max frequency ";
			partial_node_name = partial_node_name + args["t"].as<std::string>();;
			partial_node_name = partial_node_name + " Hz)]";
			icon_pixmap = QPixmap(":/images/images/loop.png");
			break;
		}
		case NodeType::REPEATER: 
		{
			YAML::Node args = YAML::Load(this->attributes);
			partial_node_name = " [Execute all actions in sequence in a loop until ";
			partial_node_name = partial_node_name + args["n"].as<std::string>();;
			partial_node_name = partial_node_name + " iterations are executed]";
			icon_pixmap = QPixmap(":/images/images/loop.png");
			break;
		}
		case NodeType::TASK: 
		{
			partial_node_name = " [" + behavior_type + "]";
			icon_pixmap = QPixmap(":/images/images/action.png");
			break;
		}
	}
	this->setIcon(0,icon_pixmap);
}

TreeItem::~TreeItem()
{

}

/*
Setter functions
*/
void TreeItem::setRoot(bool boolean)
{
	this->root = boolean;
}

void TreeItem::setNodeName(std::string texto)
{
	this->node_name = texto;
	this->setText(0, QString::fromUtf8(texto.c_str()));
}

void TreeItem::setPartialNodeName(std::string texto)
{
	this->partial_node_name = texto;
}

void TreeItem::setNodeAttributes(std::string attributes)
{
	this->attributes = attributes;
}

void TreeItem::setNodeType(NodeType nodetype)
{
	this->node_type = nodetype;
}

void TreeItem::setBehaviorType(std::string behaviortype)
{
	this->behavior_type = behaviortype;
}

void TreeItem::setRecurrent(bool recurrent)
{
	this->recurrent = recurrent;
}

void TreeItem::setActivate(bool activate)
{
	this->activate = activate;
}

void TreeItem::setTerminationResult(std::string texto)
{
        this->termination_result = texto;
}


/*
Getter functions
*/
bool TreeItem::isRoot()
{
	return this->root;
}

bool TreeItem::isRecurrent()
{
	return this->recurrent;
}

bool TreeItem::isActivated()
{
	return this->activate;
}

std::string TreeItem::getNodeName()
{
	return this->node_name;
}

std::string TreeItem::getPartialNodeName()
{
	return this->partial_node_name;
}

std::string TreeItem::getNodeAttributes()
{
	return this->attributes;
}

NodeType TreeItem::getNodeType()
{
	return this->node_type;
}

std::string TreeItem::getBehaviorType()
{
	return this->behavior_type;
}

TreeItem * TreeItem::getParent()
{
	return this->parent_node;
}

std::string TreeItem::getTerminationResult()
{
        return this->termination_result;
}

std::vector<TreeItem*> TreeItem::getChildren()
{
	return this->children;
}
/*
Other functionalities
*/
void TreeItem::addChild(TreeItem * item) 
{
	this->QTreeWidgetItem::addChild(item);
	if(item->getParent() != 0) 
	{
		item->setRoot(false);
	}
	//item->getParent()->getChildren().insert(item->getParent()->getChildren().begin(),item);
	children.push_back(item);
}

void TreeItem::removeChild(TreeItem * item)
{
	std::vector<TreeItem*>::iterator iterator = children.begin();
	int i = 0;
	for(; iterator != children.end();)
		if(item == children.at(i))
		{
			this->QTreeWidgetItem::removeChild(item);
			children.erase(iterator);
		} else {
			++iterator;
			i++;
		}
}
void TreeItem::blinkingItem(){

	std::string nameItem;
	std::string partialName;
	std::string space = " ";
	nameItem = this->getNodeName();
	partialName = nameItem;
	nameItem = nameItem+ this->getPartialNodeName();
	this->setNodeName(nameItem);
	while(paused){
		this->setNodeName(space);
		sleep(1);
		if (expandedText){
			this->setNodeName(nameItem);
		}
		else{
			this->setNodeName(partialName);
		}
		sleep(1);
	}

}

TreeItem * TreeItem::child(int index) 
{
	return children[index];
}

int TreeItem::childCount() 
{
	return children.size();
}

void TreeItem::removeItemWidget() 
{
	if(this->root)
	{
		delete this;
	} else 
	{
		TreeItem *item_parent = this->parent_node;
		TreeItem *item_to_remove = this;
		std::vector<TreeItem*>::iterator iterator = item_parent->getChildren().begin();
		TreeItem *childToRemove;
		bool deleted = false;
		int i = 0;
		for (; (iterator != item_parent->getChildren().end()) && !deleted;)
		{
			if (this == item_parent->getChildren().at(i))
			{
				item_parent->removeChild(item_parent->getChildren().at(i));
				deleted = true;
			}
			else 
			{
				++iterator;
				i++;
			}
		}
		delete this;
	}
}

std::string TreeItem::nodeTypeToString(NodeType node)
{
	switch (node)
	{
		case NodeType::PARALLEL:
		{
			return "PARALLEL";
		}
		case NodeType::REPEAT_UNTIL_FAIL:
		{
			return "REPEAT_UNTIL_FAIL";
		}
		case NodeType::QUERY:
		{
			return "QUERY";
		}
		case NodeType::TASK:
		{
			return "TASK";
		}
		case NodeType::SELECTOR:
		{
			return "SELECTOR";
		}
		case NodeType::SEQUENCE:
		{
			return "SEQUENCE";
		}
		case NodeType::INVERTER:
		{
			return "INVERTER";
		}
		case NodeType::SUCCEEDER:
		{
			return "SUCCEEDER";
		}
		case NodeType::REPEATER:
		{
			return "REPEATER";
		}
		case NodeType::ADD_BELIEF:
		{
			return "ADD BELIEF";
		}
		case NodeType::REMOVE_BELIEF:
		{
			return "REMOVE BELIEF";
		}
	}
}

NodeType TreeItem::stringToNodeType(std::string str)
{
	if (str == "PARALLEL") 
	{
		return NodeType::PARALLEL;
	}
	if (str == "REPEAT_UNTIL_FAIL") 
	{
		return NodeType::REPEAT_UNTIL_FAIL;
	}
	if (str == "REPEAT UNTIL FAIL") 
	{
		return NodeType::REPEAT_UNTIL_FAIL;
	}
	if (str == "QUERY") 
	{
		return NodeType::QUERY;
	}
	if (str == "TASK")
	{
		return NodeType::TASK;
	}
	if (str == "SELECTOR") 
	{
		return NodeType::SELECTOR;
	}
	if (str == "SEQUENCE") 
	{
		return NodeType::SEQUENCE;
	}
	if (str == "INVERTER") 
	{
		return NodeType::INVERTER;
	}
	if (str == "SUCCEEDER") 
	{
		return NodeType::SUCCEEDER;
	}
	if (str == "REPEATER") 
	{
		return NodeType::REPEATER;
	}
	if (str == "ADD BELIEF") 
	{
		return NodeType::ADD_BELIEF;
	}
	if (str == "REMOVE BELIEF") 
	{
		return NodeType::REMOVE_BELIEF;
	}
}

void TreeItem::setColor(std::string color)
{
	QColor future_color = QColor(color.c_str());
	QBrush pincel = QBrush(future_color);
	this->setForeground(0, pincel);
}

void TreeItem::setColorBackground(std::string color)
{
        QColor future_color = QColor(color.c_str());
        QBrush pincel = QBrush(future_color);
        this->setBackground(0, pincel);
}
