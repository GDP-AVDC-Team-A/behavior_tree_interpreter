/*!********************************************************************************
 * \brief ExecutionTree
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
#include "../include/execution_tree.h"
#include "../include/global.h"
std::mutex ExecutionTree::mutex;
std::condition_variable ExecutionTree::condition;
 
ExecutionTree::ExecutionTree(BehaviorTree * parent) : QObject() 
{
  ros::NodeHandle nh("~");
  nh.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  nh.param<std::string>("task_stopped", task_stopped, "task_stopped");
  nh.param<std::string>("start_task", start_task, "start_task");
  nh.param<std::string>("execute_query", execute_query, "query_belief");
  nh.param<std::string>("add_belief", add_belief, "add_belief");
  nh.param<std::string>("remove_belief", remove_belief, "remove_belief");
  
  running = true;
  waiting = false;
  tree_returned_value = false;
  cancelled = false;
  abortingWait=false;
  lastItem=NULL;
  behavior_tree = (BehaviorTree*) parent;
  behavior_sub=nh.subscribe("/"+drone_id_namespace+"/"+task_stopped, 1000, &ExecutionTree::behaviorCompletedCallback,this);
  activate_task_srv=nh.serviceClient<aerostack_msgs::StartTask>("/"+drone_id_namespace+"/"+start_task);
  initiate_behaviors_srv=nh.serviceClient<droneMsgsROS::InitiateBehaviors>("/"+drone_id_namespace+"/"+initiate_behaviors);//preguntar
  execute_query_srv=nh.serviceClient<aerostack_msgs::QueryBelief>("/"+drone_id_namespace+"/"+execute_query);
  add_belief_srv=nh.serviceClient<droneMsgsROS::AddBelief>("/"+drone_id_namespace+"/"+add_belief);
  remove_belief_srv=nh.serviceClient<droneMsgsROS::RemoveBelief>("/"+drone_id_namespace+"/"+remove_belief);
  mutex_int=1;
  droneMsgsROS::InitiateBehaviors msg_initiate;
  initiate_behaviors_srv.call(msg_initiate);
  used=0;
  QObject::connect(this,SIGNAL(update()), behavior_tree, SLOT(updateBackground()));
}

ExecutionTree::~ExecutionTree() 
{
  QObject::disconnect(this,SIGNAL(update()), behavior_tree, SLOT(updateBackground()));
}

BehaviorTree* ExecutionTree::getBehaviorTree()
{
  return this->behavior_tree;
}

void ExecutionTree::executeTree(TreeItem * item,std::list<int> ignored_items_list)//int ignored_items, int secondChilds) 
{

  if (visualState!=6){
      resetColor(item);
  }
  int ret_val = executeItem(item,ignored_items_list);//ignored_items,secondChilds);
  mutex.lock();
  tree_returned_value = ret_val;
  running = false;
  condition.notify_all();
  if (visualState!=2){
    Q_EMIT(finished());
  }
  mutex.unlock();
}
void ExecutionTree::cancelExecution(){
  
    lastVisualState=visualState;
    visualState=8;
    BehaviorTree * tree = getBehaviorTree();
    BehaviorTreeControl *btc= tree->getVisualizer();
    btc->changeVisual();     
    visualState=lastVisualState;
    abortingWait=true;


}
void ExecutionTree::pauseExecution(){
  visualState=3;

}
int ExecutionTree::executeItem(TreeItem * item, std::list<int> ignored_items_list)//int ignored_items, int secondChilds) 
{
  std::cout << "Node: " << item->getNodeName() << " is been executed" << std::endl;
  YAML::Node node = YAML::Load(item->getNodeAttributes());
  behavior_tree->scrollToItem(item, QAbstractItemView::EnsureVisible); //auto-scrolls to current item
  behavior_tree->update();
  item->setTerminationResult(error_code);
  if (!(item->getNodeName()=="HOVER")){
    itemPaused= item;
    parentItemPaused=itemPaused->getParent();
  }
  switch(item->getNodeType()) 
  {
    case NodeType::SEQUENCE: 
    {
      
      int ignored_items=0;
      if (ignored_items_list.empty()){
          int ignored_items=0;
      }
      else{
        ignored_items=ignored_items_list.front();
        ignored_items_list.pop_front();
      }
      for (int i = 0 + ignored_items; i < item->childCount(); i++) 
      {
      
        int k = executeItem(item->child(i),ignored_items_list);//ignored_items,secondChilds);
        std::list<int> x;
        ignored_items_list= x;
        
        if (k==0) 
        {          
          //if (cancelled)
          if (visualState==2)
          {
            setColor(item,COLOR_BLACK);
            item->setTerminationResult("cancelled");
            return 0;
          //}
          }
          if (visualState==3) 
        //if (paused)
          {
            item->setTerminationResult("paused");
            lastExecutionTree = this;
            return 0;

          }
          setColor(item,COLOR_RED);
          return 0; 
        }


      }
      setColor(item,COLOR_GREEN);
      item->setTerminationResult("SUCCESSFUL");
      return 1;
    }
    case NodeType::INVERTER: 
    {

      setColor(item,COLOR_BLUE);
      int value;
      std::list<int> x;
      int k = executeItem(item->child(0),x);
      if (k==0)
        {value=1;}
      if (k==2){
        value=2;
      }
      else{
        value =0;
      }
      //if (cancelled)
      if (visualState==2) 
      {
        setColor(item,COLOR_BLACK);
        return 0;
      }
      if (value==1) 
      {
        setColor(item,COLOR_GREEN);
      }
      if (value==0)
      {

        setColor(item,COLOR_RED);
      }
      return value;
    }
    case NodeType::REPEAT_UNTIL_FAIL: 
    {

      setColor(item,COLOR_BLUE);
      int number = item->childCount();
      int i = 0;
      bool failure = false;
      while (true) 
      {
        ros::Duration t(1/node["t"].as<double>());
        ros::Time startTime = ros::Time::now();
        for (int j = 0; j < item->childCount(); j++) 
        {
          std::list<int> x;
          if (executeItem(item->child(j),x)==0 || visualState==2)//cancelled) 
          {
            failure = true;
            break;
          }
        }
        i++;
        i %= number;
        ros::Duration secondsPassed = ros::Time::now() - startTime;
        ros::Duration timeLeft = t - secondsPassed;
        while (timeLeft > ros::Duration(0) && !failure && visualState!=2)//!cancelled) 
        {
          timeLeft.sleep();
          secondsPassed = ros::Time::now() - startTime;
          timeLeft = t - secondsPassed;
        }
        if (failure || visualState==2)//cancelled) 
        {
          break;
        }
      }
      if (visualState==3) 
      //if (paused)
      {
        setColor(item,COLOR_CIAN);
        return 0;

      }
      if (visualState==2) 
        //if (cancelled) 
      {
        setColor(item,COLOR_BLACK);
        return 0;
      }
      if (failure) 
      {

        setColor(item,COLOR_RED);
      }
      else {
        setColor(item,COLOR_GREEN);
      }
      return 1;
    }
    case NodeType::REPEATER: 
    {

      setColor(item,COLOR_BLUE);
      int n = node["n"].as<int>();
      int res;
      for (int i = 0; i < n; i++) 
      {
        std::list<int> x;
        res = executeItem(item->child(0), x);
        if(res==0)
        {
          break;
        }
        if (visualState==2) 
        //if (cancelled) 
        {
          setColor(item,COLOR_BLACK);
          return 0;
        }
        if (visualState==3) 
        //if (paused)
        {
          setColor(item,COLOR_CIAN);
          return 0;
        }
      }
      setColor(item,COLOR_GREEN);
      return 1;
    }
    case NodeType::SELECTOR: 
    {

      setColor(item,COLOR_BLUE);
      for (int i = 0; i < item->childCount(); i++) 
      {
        std::list<int> x;
        int value = executeItem(item->child(i), x);
        if (cancelled) 
        {
          setColor(item,COLOR_BLACK);
          return 0;
        }
        if (value==1) 
        {
          setColor(item,COLOR_GREEN);
          return 1;
        }
      }

      setColor(item,COLOR_RED);
      return 0;
    }
    case NodeType::QUERY: 
    {
      mutex_int=1;
      processing_belief_query = true;
      setColor(item,COLOR_BLUE);
      aerostack_msgs::QueryBelief query;
      query.request.query = processData(node["query"].as<std::string>());
      execute_query_srv.call(query);
      if (visualState==2) 
      //if (cancelled) 
      {
        setColor(item,COLOR_BLACK);
        return 0;
      }
      if (visualState==3) 
      //if (paused)
      {
        setColor(item,COLOR_CIAN);
        return 0;

      }
      if (!query.response.success) 
      {

        setColor(item,COLOR_RED);
        return 0;
      }
      std::string res = "";
      YAML::Node node = YAML::Load(behavior_tree->getVisualizer()->getText());
      YAML::Node node2 = YAML::Load(query.response.substitutions);
      for(YAML::const_iterator it=node2.begin();it!=node2.end();++it) 
      {
        node[it->first.as<std::string>()] = processType(it);
      }
      res = processQueryData(node);
      setText(res);
      setColor(item,COLOR_GREEN);
      sleep(1);
      return 1;
    }
    case NodeType::SUCCEEDER: 
    {

      setColor(item,COLOR_BLUE);
      std::list<int> x;
      executeItem(item->child(0), x);
      setColor(item,COLOR_GREEN);
      return 1;
    }
    case NodeType::TASK: 
    {

      setColorBackground(item,COLOR_BLUE);
      setColor(item,"#ffffff");
      std::unique_lock<std::mutex> lock(behavior_mutex);
      mutex.lock();
      actual_item = item;
      if(lastItem!= NULL && !doneFirst && lastItem->getBehaviorType().compare("TAKE_OFF")==0){
         BehaviorTree * tree = getBehaviorTree();
          BehaviorTreeControl *btc= tree->getVisualizer();
          btc->waitTimeForStart();
          doneFirst=true;

      }

      lastItem= item;
      aerostack_msgs::StartTask::Request msg;
      aerostack_msgs::StartTask::Response res;
      aerostack_msgs::TaskCommand behavior;
      //aerostack_msgs::BehaviorCommandPriority behavior;
      behavior.name = item->getBehaviorType();
      behavior.priority = 2;
      std::string args = processData(item->getNodeAttributes());
      if (!args.empty()){
       bool arg_with_variables =processDataArgs(args);//checkArgsWithoutVariables
       if (arg_with_variables){
         setColor(item,COLOR_RED);
         setColorBackground(item,"#ffffff");
         item->setTerminationResult("ARGUMENTS HAVE VARIABLES WITH UNKNOWN VARIABLES");
         mutex.unlock();
         return 0;
       }
      }
      
      behavior.parameters = args;
      req_activate.task = behavior;
      bool result = true;
      if (!item->isRecurrent()) 
      {

        int i=0;
        while(i<2000){
          i++;
        }
        bool result = activate_task_srv.call(req_activate,res_activate);
        behavior_value = result && res_activate.ack;
        if(!res_activate.ack){
          std::cout << res_activate.error_message << std::endl;

        }
      }
      else 
      {      

        if (item->isActivated())
        {
          result = activate_task_srv.call(req_activate,res_activate);
          behavior_value = result && res_activate.ack;
        }

      }
      if (!res_activate.ack) 
      {
        mutex.unlock();

        getBehaviorTree()->setTerminationCode(error_code);

        setColor(item,COLOR_RED);
        setColorBackground(item,"#ffffff");
        item->setTerminationResult(error_code);
        return 0;
      }
      if (!item->isRecurrent()) 
      {
        waiting = true;
        mutex.unlock();
        behavior_condition_variable.wait(lock);

      }
      else 
      {
        mutex.unlock();
      }
      if (false) 
      //if (cancelled) 
      {
        setColor(item,"#66023C");
        setColorBackground(item,"#ffffff");
        item->setTerminationResult("CANCELLED");
        return 0;
      }
      
      else 
      {
        if (behavior_value) 
        {
          item->setTerminationResult("SUCCESSFUL");
          setColor(item,COLOR_GREEN);
        }
        else 
        {
          sleep(1.5);
          //if (!cancelled && !paused)
          if(visualState!=2 && visualState!=3)
          {
           item->setTerminationResult(error_code);
           setColorBackground(item,"#ffffff");

           setColor(item,COLOR_RED);
         }
         if (visualState==2){
          while(!abortingWait){

           }

          abortingWait=false;
          lastVisualState=visualState;
          visualState=9;
          BehaviorTree * tree = getBehaviorTree();
          BehaviorTreeControl *btc= tree->getVisualizer();
          btc->changeVisual(); 
          visualState=lastVisualState;
          
          setColor(item,"#66023C");
          setColorBackground(item,"#ffffff");
          item->setTerminationResult("CANCELLED");
          return 0;

        }
        if (visualState==3) 
        //if (paused)
        {
          lastVisualState=visualState;
          visualState=9;
          BehaviorTree * tree = getBehaviorTree();
          BehaviorTreeControl *btc= tree->getVisualizer();
          btc->changeVisual(); 
          visualState=lastVisualState;


          setColorBackground(item,"#868D8D");
          setColor(item,"#ffffff");
          item->setTerminationResult("PAUSED");
          getBehaviorTree()->setTerminationCode(error_code);
          return 0;
        }
        getBehaviorTree()->setTerminationCode(error_code);
        return 0;
      }

    }
    setColorBackground(item,"#ffffff");
    if(behavior_value==0)
    {
      return 0;
    }
    else{
      return 1;
    }
  }
  case NodeType::PARALLEL: 
  {

    setColor(item,COLOR_BLUE);
    std::unique_lock<std::mutex> lock(mutex);
    int number_of_successes = node["n"].as<int>();
    for (int i = 0; i < item->childCount(); i++) 
    {
      ExecutionTree * et = new ExecutionTree(behavior_tree);
      trees_in_parallel.push_back(et);
      std::list<int> x;
      threads.push_back(new std::thread(std::ref(ExecutionTree::executeParallelTree),et,item->child(i),x));// 0,0));
    }
    bool value = false;
    int failures = 0;
    int successes = 0;
    int number_of_failures = item->childCount()-number_of_successes + 1;
    while (successes < number_of_successes && failures < number_of_failures) 
    {
      condition.wait(lock);
      successes = 0;
      failures = 0;
      for (int i = 0; i < trees_in_parallel.size(); i++) 
      {
        if (!trees_in_parallel[i]->isRunning()) {
          if (trees_in_parallel[i]->getReturnedValue()) 
          {
            successes +=1;
          }
          else 
          {
            failures +=1;
          }
        }
      }
      
    }
    lock.unlock();
    if (successes >= number_of_successes) 
    {
      value = true;
    }
    for (int i = 0; i < trees_in_parallel.size(); i++) 
    {
      threads[i]->join();
      delete threads[i];
      delete trees_in_parallel[i];
    }
    trees_in_parallel.clear();
    threads.clear();
    if (visualState==2) 
    //if (cancelled) 
    {
      setColor(item,COLOR_BLACK);
      return 0;
    }
    //if (paused)
    if (visualState==3) 
    {

      setColor(item,COLOR_CIAN);
      return 0;
    }
    if (value) 
    {
      setColor(item,COLOR_GREEN);
    }
    else 
    {

      setColor(item,COLOR_RED);
    }
    if (value ==0){
      return 0;
    }
    else{
      return 1;
    }
  }
  case NodeType::ADD_BELIEF: 
  {

    setColor(item,COLOR_BLUE);
    droneMsgsROS::AddBelief beliefs;
    beliefs.request.belief_expression = processData(node["belief_expression"].as<std::string>());
    bool aux = node["multivalued"].as<bool>();
    beliefs.request.multivalued = aux;
    add_belief_srv.call(beliefs);
    if (visualState==2) 
//if (cancelled) 
    {
      setColor(item,COLOR_BLACK);
      return 0;
    }
    if (visualState==3) 
    //if (paused)
    {

      setColor(item,COLOR_CIAN);
      return 0;

    }
    if (!beliefs.response.success) 
    {

      setColor(item,COLOR_RED);
      return 0;
    }
    setColor(item,COLOR_GREEN);
    return 1;
  }
  case NodeType::REMOVE_BELIEF: 
  {

    setColor(item,COLOR_BLUE);
    droneMsgsROS::RemoveBelief beliefs;
    beliefs.request.belief_expression = processData(node["belief_expression"].as<std::string>());
    remove_belief_srv.call(beliefs);
    if (visualState==2) 
    //if (cancelled) 
    {
      setColor(item,COLOR_BLACK);
      return 0;
    }
    if (visualState==3) 
    //if (paused)
    {

      setColor(item,COLOR_CIAN);
      return 0;

    }
    if (!beliefs.response.success) 
    {

      setColor(item,COLOR_RED);
      return 0;
    }
    setColor(item,COLOR_GREEN);
    return 1;
  }
  default: 
  {
  }
  visualState=10;
  BehaviorTree * tree = getBehaviorTree();
  BehaviorTreeControl *btc= tree->getVisualizer();
  btc->changeVisual(); 
}
}

void ExecutionTree::executeParallelTree(ExecutionTree * et, TreeItem* item,std::list<int> ignored_items_list)// int ignored_items,int secondChilds)  
{
  et->executeTree(item, ignored_items_list);//ignored_items,secondChilds);
}


int ExecutionTree::getReturnedValue() 
{
  return tree_returned_value;
}

bool ExecutionTree::isRunning() 
{
  return running;
}

void ExecutionTree::behaviorCompletedCallback(const aerostack_msgs::TaskStopped &msg) 
{ 
  mutex.lock();
  std::vector <std::string> termination_causes = {"GOAL_ACHIEVED", "TIME_OUT", "WRONG_PROGRESS", "PROCESS_FAILURE", "INTERRUPTED"};
  error_code = termination_causes.at(msg.termination_cause-1);
  std::cout <<error_code << std::endl; 
  if (waiting && msg.name == actual_item->getBehaviorType()) 
  {
    waiting = false;
    behavior_value = (msg.termination_cause == aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
    behavior_condition_variable.notify_all();
  }
  mutex.unlock();
}

void ExecutionTree::setColor(TreeItem* item, std::string color) 
{
  item->setColor(color);
  Q_EMIT(update());
}

void ExecutionTree::setColorBackground(TreeItem* item, std::string color)
{
  item->setColorBackground(color);
  Q_EMIT(update());
}

void ExecutionTree::resetColor(TreeItem* item){

  NodeType item_nodetype = item->getNodeType();

  if(item->childCount()>0)
  {
    for(int i = 0; i < item->childCount(); i++)
    {
      resetColor(item->child(i));
    }
  }
  item->setColor("#000000");
}

std::string ExecutionTree::processData(std::string raw_arguments) 
{
  
  while(processing_belief_query){
    if (mutex_int==1){
      break;
    }
  }
  mutex_int=0;
  BehaviorTree * tree = getBehaviorTree();
  BehaviorTreeControl *btc= tree->getVisualizer();
  std::string text = btc->getText();

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
bool ExecutionTree::processDataArgs(std::string query){
    //std::cout << query <<std::endl;

  if (query.find("+")!= std::string::npos)
  {
    return true;
  }
  return false;
}

std::string ExecutionTree::processQueryData(YAML::Node query) 
{
  std::string res = "";
  for(YAML::const_iterator it=query.begin();it!=query.end();++it) 
  {
    res = res + it->first.as<std::string>() + ": " + processType(it);
    res = res + "\n";
  }
  return res;
}

std::string ExecutionTree::processType(YAML::const_iterator it) 
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

void ExecutionTree::setText(std::string str) 
{
  behavior_tree->getVisualizer()->setText(str);
  Q_EMIT(updateText());
}

