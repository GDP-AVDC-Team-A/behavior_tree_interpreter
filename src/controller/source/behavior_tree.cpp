/*!********************************************************************************
 * \brief     This class describes the behavior tree object.
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
#include "../include/behavior_tree.h"
#include "../include/tree_item.h" //TreeItem class is included here to prevent recursive includes with tree_item.
#include "../include/global.h"
#include "std_msgs/String.h"
#include <string>
   
BehaviorTree::BehaviorTree(QWidget *parent) : QTreeWidget(parent)
{
	//window always on top
	QWidget::setLocale(QLocale());
	this->headerItem()->setHidden(true);
	this->setContextMenuPolicy(Qt::CustomContextMenu);
	this->resize(600, 300);
	this->setFocusPolicy(Qt::FocusPolicy::NoFocus);
	this->header()->setStretchLastSection(false);
	this->header()->setSectionResizeMode(QHeaderView::ResizeToContents);

	qRegisterMetaType<QVector<int>>("QVector<int>");

	control_parent = (BehaviorTreeControl*) parent;

	has_root = false;
	running = false;
	isTextExpanded = true;
	//followPath_rotate= false;
	//followPathState = "";
	//signal is emitted when the widget's contextMenuPolicy is Qt::CustomContextMenu
	QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
	QObject::connect(this, SIGNAL(executionStarted()), parent, SLOT(setStartBlockingTextInput()));

	//Style sheet customization
	setStyleTreeSheet();

    //Establishment of connections
	setUp();
}

/*------------------------------------------------------------
--------------------------------------------------------------
                      Destructor
--------------------------------------------------------------  
------------------------------------------------------------*/
BehaviorTree::~BehaviorTree()
{

}

/*------------------------------------------------------------
--------------------------------------------------------------
                Getters and setters
--------------------------------------------------------------
------------------------------------------------------------*/
void BehaviorTree::setStyleTreeSheet()
{
	setIconSize(QSize(25, 25));
	this->setStyleSheet(" \
		QTreeView::branch { \
			background: white; \
		} \
		QTreeView::branch:has-siblings:!adjoins-item { \
			border-image: url(:/images/images/vline.png) 0; \
		} \
		QTreeView::branch:has-siblings:adjoins-item { \
			border-image: url(:/images/images/branch-more.png) 0; \
		} \
		QTreeView::branch:!has-children:!has-siblings:adjoins-item { \
			border-image: url(:/images/images/branch-end.png) 0; \
		} \
		QTreeView::branch:has-children:!has-siblings:closed, \
		QTreeView::branch:closed:has-children:has-siblings { \
			border-image: url(:/images/images/branch-more_proto_mas.png) 0; \
		} \
		QTreeView::branch:has-children:!has-siblings:closed, \
		QTreeView::branch:closed:has-children:!has-siblings { \
			border-image: url(:/images/images/branch-end_proto_mas.png) 0; \
		} \
		QTreeView::branch:open:has-children:has-siblings  { \
			border-image: url(:/images/images/branch-more_proto_menos.png) 0; \
		} \
		QTreeView::branch:open:has-children:!has-siblings  { \
			border-image: url(:/images/images/branch-end_proto_menos.png) 0; \
		} \
		QTreeView::item:selected { \
			background-color:transparent; \
			color:black; \
		} \
		");
}

bool BehaviorTree::isRunning()
{
	return running;
}

BehaviorTreeControl* BehaviorTree::getVisualizer()
{
	return control_parent;
}

ExecutionTree * BehaviorTree::getExecutionTree(){
	if(running)
		return this -> et;
	else
		return nullptr;
}

void BehaviorTree::setUp()
{	ros::NodeHandle n("~");
	n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
	n.param<std::string>("start_task", start_task, "start_task");
	activate_task_srv=n.serviceClient<aerostack_msgs::StartTask>("/"+drone_id_namespace+"/"+start_task);
	n.param<std::string>("all_beliefs", all_beliefs, "all_beliefs");
    



    //connect(this,SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(terminationMessage()));
}

/*------------------------------------------------------------
--------------------------------------------------------------
                      Buttons actions
--------------------------------------------------------------  
------------------------------------------------------------*/
void BehaviorTree::createMissionByTreeItem(TreeItem * root)
{
	root_item = root;
	this->addTopLevelItem(root_item);
	if (root != 0) {
		if (isTextExpanded) {
			expandTreeText(2);
		}
		else {
			expandTreeText(0);
		}
		this->expandAll();
	}
}

/*
This function is called whenever we do a right click
It creates the context menu and disables any functionality that cannot be executed. This depends on where we right clicked.
*/
void BehaviorTree::onCustomContextMenu(const QPoint &p)
{
	if(itemAt(p)!=0)
	{
		TreeItem *item_clicked;
		if(is_menu_created)
		{
			contextMenu->close();
			delete contextMenu;
			is_menu_created = false;
		}
		contextMenu = new QMenu("Menu", this);
		is_menu_created = true;
		item_clicked = (TreeItem*)itemAt(p);
		TreeItem *item_clicked_parent;

		int children;
		//Check if the clicked node can have children
		if(item_clicked->getNodeType()==NodeType::SUCCEEDER)
		{
			children = item_clicked->childCount();
		}
		if(item_clicked->getNodeType()==NodeType::INVERTER)
		{
			children = item_clicked->childCount();
		}
		if(item_clicked->getNodeType()==NodeType::SELECTOR)
		{
			children = item_clicked->childCount();
		}

		//Check if the clicked node can have siblings
		item_clicked_parent = item_clicked->getParent();

		point = p;

		std::string termination_cause;
		termination_cause = item_clicked->getTerminationResult();
		QAction action("Execute tree from this node", this);
		contextMenu->addAction(&action);
		connect(&action, SIGNAL(triggered()), this, SLOT(executeTreeFromItem()));
		QAction action2("View termination state", this);
                //action2.setText(termination_cause);

		contextMenu->addAction(&action2);
		connect(&action2, SIGNAL(triggered()), this, SLOT(terminationMessage()));

		contextMenu->exec(mapToGlobal(p));
	}
	else if (itemAt(p)==0)
	{
		point = p;
		if(is_menu_created)
		{
			contextMenu->close();
			delete contextMenu;
			is_menu_created = false;
		}
		is_menu_created = true;
		contextMenu = new QMenu("Menu", this);
	}
}


/*
This function adds a root to the tree if it doesn't have one already
*/
void BehaviorTree::addTopLevelItem(TreeItem *top_level_item)
{
	if (has_root) {
		if (root_before != 0) {
			delete root_before;
		}
	}
	if (top_level_item != 0) {
		root_before = top_level_item;
		has_root = true;
		top_level_item->setRoot(true);
	}
	else {
		has_root = false;
	}
	QTreeWidget::addTopLevelItem(top_level_item);
}

/*
This function creates an ExecutionTree object to execute the tree's nodes, starting at the root
*/
void BehaviorTree::executeTree()
{	sleep(0.5);
	
	this->setTerminationCode("");
	if(has_root){
		if (running) 
		{

			joinExecutionTree();
		}
		et = new ExecutionTree(this);
		running = true;
		if(control_parent->correct_format)
		{				
				QObject::connect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution())); //connectQAppToExecutionTree
				connectExecuteTree(this,et,true);
				running = true;
				et->setColor(root_item,COLOR_BLACK);
				std::list<int> x;
				executing_tree = new std::thread(std::ref(ExecutionTree::executeParallelTree),et,root_item,x); //0,0);
				disconnectCustomContextMenu();
				Q_EMIT(executionStarted());

			} 
			else 
			{
				running = false;
			}
		}

		else
			QMessageBox::information(this,tr("Cannot execute tree"),"The behavior tree is not defined or defined incorrectly");
	}

/*
This function creates an ExecutionTree object to execute the tree's nodes, starting at the clicked item
*/
	void BehaviorTree::executeTreeFromItem()
	{   sleep(0.5);
		this->setTerminationCode("");
		TreeItem * item_to_execute = (TreeItem*)itemAt(point);
		TreeItem * parent_item_to_execute;
		if(!item_to_execute->isRoot())
		{			
			if (running) 
			{

				joinExecutionTree();
			}
			parent_item_to_execute = item_to_execute->getParent();
			while(parent_item_to_execute->getParent()!=NULL){
				parent_item_to_execute=parent_item_to_execute->getParent();
			}
			int secondChilds=0;
			int ignored_items = 0;
			et = new ExecutionTree(this);
			running = true;
			if(control_parent->correct_format)
			{
				std::vector<TreeItem*> children = parent_item_to_execute->getChildren();
				std::vector<TreeItem*>::iterator iterator = children.begin();
				children = parent_item_to_execute->getChildren();
				iterator = children.begin();
				std::list<int> x;
				std::list<int> ignored_items_list=getIgnoredItems(children/*,iterator*/,item_to_execute,x);
				QObject::connect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution()));
				disconnectCustomContextMenu();
				connectExecuteTree(this,et,true);
			    if (itemPaused!= NULL){
			    et->setColor(parent_item_to_execute,COLOR_BLACK);
				et->setColor(itemPaused,COLOR_BLACK);
				et->setColorBackground(itemPaused,"#FFFFFF");
			    }
		
				executing_tree = new std::thread(std::ref(ExecutionTree::executeParallelTree),et,parent_item_to_execute,ignored_items_list);// ignored_items,secondChilds);
				Q_EMIT(executionStarted());

			}
			else 
			{
				running = false;
			}
		} 
		else 
		{
			if (running) 
			{

				joinExecutionTree();
			}
			et = new ExecutionTree(this);
		//if(control_parent->checkTree(item_to_execute))
			if(control_parent->correct_format)
			{	
				running = true;
				QObject::connect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution()));
				connectExecuteTree(this,et,true); 
				std::list<int> x;
				executing_tree = new std::thread(std::ref(ExecutionTree::executeParallelTree),et,item_to_execute, x);
				Q_EMIT(executionStarted());
			}
			else 
			{
				running = false;
			}
		}
	}

	void BehaviorTree::joinExecutionTree() {
	
		if(visualState!=-1){


		if (visualState!=3 && visualState!=6){
			executing_tree->join();
			ExecutionTree::mutex.lock();
			disconnectExecuteTree(this,et);
	//QObject::disconnect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution()));

			delete executing_tree;
			running = false;
			delete et;
			if (is_menu_created) {
				contextMenu->close();
				delete contextMenu;
				is_menu_created = false;
			}
			ExecutionTree::mutex.unlock();

		}
	}
		
	}


	void BehaviorTree::updateBackground()
	{
		ExecutionTree::mutex.lock();
		repaint();
		ExecutionTree::mutex.unlock();
	}

	void BehaviorTree::cancelTree()
	{	
//et = new ExecutionTree(this);
     //QObject::connect(this, SIGNAL(cancelExecutionSignal()), et, SLOT(cancelExecution()));
		if (running) {
			aerostack_msgs::TaskCommand behavior;
			behavior.name = "HOVER";
			behavior.priority= 2;
			req_activate.task = behavior;
			running=false;
			activate_task_srv.call(req_activate,res_activate);

			if(!res_activate.ack)
				std::cout << res_activate.error_message << std::endl;
			
		}
		connectCustomContextMenu();
		if (lastVisualState==3){
		running=false;
    	//setColor(itemPaused,"#66023C");
    	itemPaused->setColor("#66023C");
    	//setColorBackground(itemPaused,"#ffffff");
    	itemPaused->setColorBackground("#ffffff");
    }
    else{
		Q_EMIT(cancelExecutionSignal());
    }

	}
	void BehaviorTree::continueMission(){
		this->setTerminationCode("");
		TreeItem * item_to_execute = itemPaused;
		TreeItem * parent_item_to_execute;

		if(!item_to_execute->isRoot())
		{			
			if (running) 
			{

				joinExecutionTree();
			}
			parent_item_to_execute =item_to_execute->getParent();
			while(parent_item_to_execute->getParent()!=NULL){
				parent_item_to_execute=parent_item_to_execute->getParent();
			}
			int ignored_items = 0;
			int secondChilds=0;
			//et = new ExecutionTree(this);
			et = lastExecutionTree;
			running = true;
			if(control_parent->correct_format)
			{
				std::vector<TreeItem*> children = parent_item_to_execute->getChildren();
				
				std::list<int> x;
				std::list<int> ignored_items_list=getIgnoredItems(children,/*iterator,*/item_to_execute,x);
			    
				QObject::connect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution()));
				disconnectCustomContextMenu();
				connectExecuteTree(this,et,true);
				executing_tree = new std::thread(std::ref(ExecutionTree::executeParallelTree),et,parent_item_to_execute,ignored_items_list);
			}
			else 
			{
				running = false;
			}
		 }
		else 
		{
			if (running) 
			{   
				joinExecutionTree();
			}
			et = lastExecutionTree;
			if(control_parent->correct_format)
			{	
				running = true;
				QObject::connect(qApp,SIGNAL(lastWindowClosed()), et, SLOT(cancelExecution()));
				connectExecuteTree(this,et,true); 
				std::list<int> x;
				executing_tree = new std::thread(std::ref(ExecutionTree::executeParallelTree),et,item_to_execute,x);
			}
			else 
			{
				running = false;
			}
		}
}
	std::list<int> BehaviorTree::getIgnoredItems(std::vector<TreeItem*>  children, /*std::vector<TreeItem*>::iterator  iterator,*/ TreeItem * item_to_execute, std::list<int> res){
	
			int i = 0;
			std::vector<TreeItem*>::iterator  iterator = children.begin();
			for(; iterator != children.end();)
			{	
				
				

				if(item_to_execute == children.at(i))
			 	{   

					  res.push_back(i);
					  break;
				} 
				if (!children.at(i)->getChildren().empty()){
					std::vector<TreeItem*> children2 = children.at(i)->getChildren();
				
					std::vector<TreeItem*>::iterator iterator2 = children2.begin();

					res.push_back(i);
					int size=res.size();
					
			        res = getIgnoredItems(children2, /*iterator2,*/ item_to_execute,res);
			        
			        std::list <int>::iterator it; 

			        for(it = res.begin(); it != res.end(); ++it){

                    }
                    if (!res.empty()){
                    	if (res.size()==size){
			        	res.pop_back();
			        	}
			        	else{
			        	  break;
			        	}
                    }
			        
			        
				}
					++iterator;
					i++;

			}
			return res;	
	}
	void BehaviorTree::pauseMission( ) {
		aerostack_msgs::TaskCommand behavior;
		behavior.name = "HOVER";
		behavior.priority= 2;
		req_activate.task = behavior;

		activate_task_srv.call(req_activate,res_activate);
		if(!res_activate.ack)
			std::cout << res_activate.error_message << std::endl;
		Q_EMIT(pauseExecutionSignal());
      	//disconnectCustomContextMenu();

	}
	void BehaviorTree::windowFinished(TreeItem *the_item)
	{
		this->expandAll();
		if (isTextExpanded) {
			expandTreeText(0);
		}
		else {
			expandTreeText(2);
		}
		QObject::connect(window, SIGNAL(windowAccepted(TreeItem *)), this, SLOT(windowFinished(TreeItem *)));
	}

/*
Next function is called whenever we want to add a child
*/

	void BehaviorTree::addChild(const QPoint &p)
	{
		parent_item_for_child = (TreeItem*)itemAt(p);
		window = new BehaviorDialog(this, parent_item_for_child);
		window->show();
		QObject::connect(window, SIGNAL(windowAccepted(TreeItem *)), this, SLOT(windowFinished(TreeItem *)));
	}

	void BehaviorTree::expandTreeText(int checkState)
	{
		if (has_root) {
			if(checkState == 2)
			{			
				expandText(root_item);
				isTextExpanded=true;
			}
			else if(checkState == 0)
			{
				minimizeText(root_item);
				isTextExpanded=false;
			}
		}
	}

	void BehaviorTree::expandText(TreeItem *item)
	{  expandedText = true;
		std::string text = item->getNodeName();
		text = text +  item->getPartialNodeName();
		item->setText(0, QString::fromStdString(text));
		if(item->childCount()>0)
		{
			for(int i = 0; i < item->childCount(); i++)
			{
				expandText(item->child(i));
			}
		}
	}

	void BehaviorTree::minimizeText(TreeItem *item)
	{expandedText = false;
		item->setText(0, QString::fromStdString(item->getNodeName()));
		if(item->childCount()>0)
		{
			for(int i = 0; i < item->childCount(); i++)
			{
				minimizeText(item->child(i));
			}
		}
	}

	void BehaviorTree::connectExecuteTree(QTreeWidget* behavior_tree, QObject* et,bool from_item)
	{
		if(from_item)
		{
		}
		QObject::connect(behavior_tree, SIGNAL(cancelExecutionSignal()), et, SLOT(cancelExecution()));

		BehaviorTree *bt = (BehaviorTree*) behavior_tree;
		QObject::connect(behavior_tree, SIGNAL(pauseExecutionSignal()), et, SLOT(pauseExecution()));
		
		QObject::connect(et, SIGNAL(finished()), bt, SLOT(joinExecutionTree()));
		QObject::connect(et, SIGNAL(finished()), bt->getVisualizer(), SLOT(setStopBlockingTextInput()));
		QObject::connect(et, SIGNAL(updateText()), bt->getVisualizer(), SLOT(update()));
		QObject::connect(behavior_tree, SIGNAL(executionStarted()), bt->getVisualizer(), SLOT(setStartBlockingTextInput()));
	}

	void BehaviorTree::disconnectExecuteTree(QTreeWidget* behavior_tree, QObject* et)
	{	 
		BehaviorTree *bt = (BehaviorTree*) behavior_tree;
		QObject::disconnect(behavior_tree, SIGNAL(pauseExecutionSignal()), et, SLOT(pauseExecution()));

		QObject::disconnect(behavior_tree, SIGNAL(cancelExecutionSignal()), et, SLOT(cancelExecution()));
		QObject::disconnect(et, SIGNAL(finished()), bt, SLOT(joinExecutionTree()));
		QObject::disconnect(et, SIGNAL(updateText()),bt->getVisualizer(), SLOT(update()));
		QObject::disconnect(et, SIGNAL(finished()),bt->getVisualizer(), SLOT(setStopBlockingTextInput()));
		QObject::disconnect(behavior_tree, SIGNAL(executionStarted()), bt->getVisualizer(), SLOT(setStartBlockingTextInput()));
	}

	void BehaviorTree::setTerminationCode(std::string code){
		termination_code = code;
	}

	void BehaviorTree::terminationMessage(){
    //if (termination_code.size() != 0){
    //    this->getVisualizer()->windowManager('e', "Execution failed", termination_code);
    //}
		TreeItem * item_to_view = (TreeItem*)itemAt(point);
		if (item_to_view->getTerminationResult().size() != 0){
			this->getVisualizer()->windowManager('e', "Termination state", item_to_view->getTerminationResult());
		}
	}


	void BehaviorTree::connectCustomContextMenu()
	{
		QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
	}


	void BehaviorTree::disconnectCustomContextMenu()
	{
		QObject::disconnect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
	}

