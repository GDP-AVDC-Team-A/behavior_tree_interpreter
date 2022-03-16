/*
  BehaviorDialog  
  @author  Daniel Del Olmo, Jorge Luis Pascual, Carlos Valencia.
  @date    10-2018
  @version 2.0
*/
/*!********************************************************************************
 * \brief    This window shows information about the behaviors
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
#include "../include/behavior_dialog.h"
 
BehaviorDialog::BehaviorDialog(QWidget *parent, TreeItem *padre) : QDialog(parent), ui(new Ui::BehaviorDialog)
{
  ui->setupUi(this);
  my_layout=ui->layout;
  this->padre = padre;
  this->is_modifying = false;
  
  /*
  Hiding widgets until they have to show
  */
  hideAllWidgets();
  ui->add_belief_content->show();

  resize(minimumSize());
    ros::NodeHandle n("~");
  n.param<std::string>("consult_available_behaviors", consult_available_behaviors, "consult_available_behaviors");
  std::string drone_id_namespace;
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  consult_available_behaviors_srv=n.serviceClient<droneMsgsROS::ConsultAvailableBehaviors>("/"+drone_id_namespace+"/"+consult_available_behaviors);
  droneMsgsROS::ConsultAvailableBehaviors behaviors_message;
  droneMsgsROS::ListOfBehaviors behaviors_list;
  consult_available_behaviors_srv.call(behaviors_message);
  behaviors_list = behaviors_message.response.available_behaviors;
  available_behaviors = behaviors_list.behaviors;

  QList<QString> *available_behaviors_list = new QList<QString>();
  for(int i = 0; i < available_behaviors.size(); i++)
  {
    std::string behavior_aux = available_behaviors[i];
    available_behaviors_list->append(QString::fromStdString(behavior_aux));
  }

  QStringList definitive_list = *available_behaviors_list;
  ui->behavior_combobox->addItems(definitive_list);

  /* 
  Connections
  */
  connect(ui->accept_button, SIGNAL(clicked()), this, SLOT(actionAccept()));
  connect(ui->cancel_button, SIGNAL(clicked()), this, SLOT(actionCancel()));
  connect(ui->node_type_combobox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(nodeTypeComboBoxChanged(const QString &)));
  connect(ui->behavior_combobox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(behaviorComboBoxChanged(const QString &)));
}

BehaviorDialog::~BehaviorDialog()
{
  delete ui;
}

BehaviorDialog::BehaviorDialog(QWidget *parent, TreeItem *padre, TreeItem *treeitem_clicked) : QDialog(parent), ui(new Ui::BehaviorDialog)
{
  /*
  This constructor is only called when modifying an existing node
  */
  ui->setupUi(this);
  my_layout=ui->layout;
  this->padre = padre;
  this->treeitem_clicked = treeitem_clicked;
  this->is_modifying = true;
  ros::NodeHandle n("~");
  n.param<std::string>("consult_available_behaviors", consult_available_behaviors,
    "consult_available_behaviors");
  std::string drone_id_namespace;
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
    //std::cout << "namespace del dialog "<< drone_id_namespace<< std::endl;
  consult_available_behaviors_srv=n.serviceClient<droneMsgsROS::ConsultAvailableBehaviors>("/"+drone_id_namespace+"/"+consult_available_behaviors);
  droneMsgsROS::ConsultAvailableBehaviors behaviors_message;
  droneMsgsROS::ListOfBehaviors behaviors_list;
  consult_available_behaviors_srv.call(behaviors_message);
  behaviors_list = behaviors_message.response.available_behaviors;
  available_behaviors = behaviors_list.behaviors;

  QList<QString> *available_behaviors_list = new QList<QString>();
  for(int i = 0; i < available_behaviors.size(); i++)
  {
    std::string behavior_aux = available_behaviors[i];
    available_behaviors_list->append(QString::fromStdString(behavior_aux));
  }

  QStringList definitive_list = *available_behaviors_list;
  ui->behavior_combobox->addItems(definitive_list);

  int children;
  children = treeitem_clicked->childCount();
  QString name;
  QString name_aux;
  NodeType nodetype;
  std::string behaviortype;
  YAML::Node yamlnode = YAML::Load(treeitem_clicked->getNodeAttributes());

  /*
  Conditions to disable certain entries in the ComboBox so the user can't modify the NodeType to an invalid NodeType
  */
  if(children == 1)
  {
    QModelIndex index_to_disable = ui->node_type_combobox->model()->index(0,0);
    QVariant v(0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(1,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(2,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(6,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
  } 
  else if(children > 1)
  {
    QModelIndex index_to_disable = ui->node_type_combobox->model()->index(0,0);
    QVariant v(0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(1,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(2,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(5,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(6,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(8,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
    index_to_disable = ui->node_type_combobox->model()->index(9,0);
    ui->node_type_combobox->model()->setData(index_to_disable, v, Qt::UserRole -1);
  }

  /*
  Setting up which nodes to show and retrieve the values from the node
  */
  name = QString::fromStdString(treeitem_clicked->getNodeName());
  ui->name_content->insert(name);
  nodetype = treeitem_clicked->getNodeType();

  /*
  Values are retrieved from the object created previously and shown in the window when modifying an object
  */
  hideAllWidgets();

  if(nodetype == NodeType::TASK)
  {
    ui->arguments_label->show();
    ui->behavior_combobox->show();
    ui->behavior_content->show();
    ui->behavior_label->show();
    ui->mode_label->show();
    ui->behavior_mode_combobox->show();
    ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("BEHAVIOR")));
    behaviortype = treeitem_clicked->getBehaviorType();
    ui->behavior_combobox->setCurrentIndex(ui->behavior_combobox->findText(QString::fromStdString(behaviortype)));
    ui->behavior_content->setPlainText(QString::fromStdString(treeitem_clicked->getNodeAttributes()));
    resize(minimumSize());
    if(treeitem_clicked->isRecurrent())
    {
      if(treeitem_clicked->isActivated())
      {
        ui->behavior_mode_combobox->setCurrentIndex(ui->behavior_mode_combobox->findText(QString::fromStdString("ACTIVATE BEHAVIOR")));
      }
      else
      {
        ui->behavior_mode_combobox->setCurrentIndex(ui->behavior_mode_combobox->findText(QString::fromStdString("DEACTIVATE BEHAVIOR")));
      }
    }
    else
    {
      ui->behavior_mode_combobox->setCurrentIndex(ui->behavior_mode_combobox->findText(QString::fromStdString("EXECUTE BEHAVIOR")));
    }
  } 
  else //Not a behavior
  {
    resize(minimumSize());
    if(nodetype == NodeType::PARALLEL)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("PARALLEL")));
      ui->parallel_argument_content_one->show();
      ui->parallel_argument_one->show();
      int argument_n = yamlnode["n"].as<int>();
      ui->parallel_argument_content_one->setValue(argument_n);
      resize(minimumSize());
    }
    else if(nodetype == NodeType::REPEAT_UNTIL_FAIL)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("REPEAT UNTIL FAIL")));
      ui->ruf_argument_content_one->show();
      ui->ruf_argument_one->show();
      double argument_n = yamlnode["t"].as<double>();
      ui->ruf_argument_content_one->setValue(argument_n);
      resize(minimumSize());
    }
    else if(nodetype == NodeType::REPEATER)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("REPEATER")));
      ui->repeater_argument_one_content->show();
      ui->repeater_argument_one_label->show();
      int argument_t = yamlnode["n"].as<int>();
      ui->repeater_argument_one_content->setValue(argument_t);
      resize(minimumSize());
    }
    else if(nodetype == NodeType::QUERY)
    { 
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("QUERY")));
      ui->query_content->show();
      if(treeitem_clicked->getNodeAttributes() != "query: ")
      {
        std::string argument_query = yamlnode["query"].as<std::string>();
        ui->query_content->setText(QString::fromStdString(argument_query));
      }
      resize(minimumSize());
    }
    else if(nodetype == NodeType::ADD_BELIEF)
    { 
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("ADD BELIEF")));
      ui->add_belief_content->show();
      ui->add_belief_content->setText(QString::fromStdString(treeitem_clicked->getNodeAttributes()));
      resize(minimumSize());
    }
    else if(nodetype == NodeType::REMOVE_BELIEF)
    { 
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("REMOVE BELIEF")));
      ui->remove_belief_content->show();
      ui->remove_belief_content->setText(QString::fromStdString(treeitem_clicked->getNodeAttributes()));
      resize(minimumSize());
    }
    else if(nodetype == NodeType::SUCCEEDER)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("SUCCEEDER")));
    }
    else if(nodetype == NodeType::INVERTER)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("INVERTER")));
    }
    else if(nodetype == NodeType::SELECTOR)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("SELECTOR")));
    }
    else if(nodetype == NodeType::SEQUENCE)
    {
      ui->node_type_combobox->setCurrentIndex(ui->node_type_combobox->findText(QString::fromStdString("SEQUENCE")));
    }
  }

  resize(minimumSize());
  /*
  Connections
  */
  connect(ui->accept_button, SIGNAL(clicked()), this, SLOT(actionModify()));
  connect(ui->cancel_button, SIGNAL(clicked()), this, SLOT(actionCancel()));
  connect(ui->node_type_combobox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(nodeTypeComboBoxChanged(const QString &)));
  connect(ui->behavior_combobox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(behaviorComboBoxChanged(const QString &)));
}

void BehaviorDialog::hideAllWidgets()
{
  ui->add_belief_content->hide();
  ui->remove_belief_content->hide();
  ui->arguments_label->hide();
  ui->behavior_combobox->hide();
  ui->behavior_content->hide();
  ui->behavior_label->hide();
  ui->behavior_mode_combobox->hide();
  ui->mode_label->hide();
  ui->parallel_argument_content_one->hide();
  ui->parallel_argument_one->hide();
  ui->query_content->hide();
  ui->repeater_argument_one_content->hide();
  ui->repeater_argument_one_label->hide();
  ui->ruf_argument_content_one->hide();
  ui->ruf_argument_one->hide();
  resize(minimumSize());
}

/*
This function is called when creating a new node
*/
void BehaviorDialog::actionAccept()
{
  //If there's no node name specified an error message pops up
  if(!ui->name_content->text().isEmpty())
  {
    TreeItem *new_tree_item;
    new_tree_item = new TreeItem(padre, NodeType::SEQUENCE);
    BehaviorDialog::modifyNode(padre, new_tree_item);
  }
  else 
  {
    QMessageBox error_message;
    error_message.setWindowTitle(QString::fromStdString("Node parameters error"));
    error_message.setText(QString::fromStdString("The node name cannot be empty. Please insert a name."));
    error_message.exec();
  }
}

/*
This function is called when the user cancels a node creation/modification
*/
void BehaviorDialog::actionCancel()
{
  Q_EMIT(close());
}

/*
This function is called when modifying an existing node
*/
void BehaviorDialog::actionModify()
{
  BehaviorDialog::modifyNode(padre, treeitem_clicked);
}

void BehaviorDialog::modifyNode(TreeItem *padre_aux, TreeItem *node_to_modify)
{
  std::string combobox_content = ui->node_type_combobox->currentText().toUtf8().constData();

  std::string args;
  QString node_name;
  std::string full_node_name;
  std::string partial_node_name;

  QIcon node_icon;
  QPixmap icon_pixmap;
  node_to_modify->setActivate(false);
  node_to_modify->setRecurrent(false);

  if(combobox_content == "SEQUENCE")
  {
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::SEQUENCE);
    partial_node_name = " [Execute all actions in sequence until one fails]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name;  
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/sequence.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  } 
  else if(combobox_content == "SELECTOR")
  {
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::SELECTOR);
    partial_node_name = " [Execute all actions in sequence until one succeeds]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/selector.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  } 
  else if(combobox_content == "SUCCEEDER")
  {
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::SUCCEEDER);
    partial_node_name = " [Always returns success]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name));
    icon_pixmap = QPixmap(":/images/images/succeeder.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  } 
  else if(combobox_content == "INVERTER")
  {
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::INVERTER);
    partial_node_name = " [Inverts the result of the child node]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/inverter.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  } 
  else if(combobox_content == "QUERY")
  {
    args = "query: ";
    args =  args + this->ui->query_content->toPlainText().toUtf8().constData();
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::QUERY);
    partial_node_name = " [Query to the belief memory]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/query.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "ADD BELIEF")
  {
    args = this->ui->add_belief_content->toPlainText().toUtf8().constData();
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::ADD_BELIEF);
    partial_node_name = " [Add belief to the belief memory]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/tree_action.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "REMOVE BELIEF")
  {
    args = this->ui->remove_belief_content->toPlainText().toUtf8().constData();
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::REMOVE_BELIEF);
    partial_node_name = " [Remove belief from the belief memory]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/tree_action.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "PARALLEL")
  {
    args = "n: ";
    args = args + asString(this->ui->parallel_argument_content_one);
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::PARALLEL);
    partial_node_name = " [Execute actions in parallel. Node succeeds if ";
    partial_node_name = partial_node_name + std::to_string(this->ui->parallel_argument_content_one->value());
    partial_node_name = partial_node_name + " child node(s) succeed(s). Fails otherwise]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() +  partial_node_name;
    node_to_modify->setText(0, QString::fromStdString(full_node_name));
    icon_pixmap = QPixmap(":/images/images/parallel.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "REPEAT UNTIL FAIL")
  {
    args = "t: ";
    args = args + asString(ui->ruf_argument_content_one);
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::REPEAT_UNTIL_FAIL);
    partial_node_name = " [Execute all actions in sequence in a loop until one fails (max frequency ";
    partial_node_name = partial_node_name + (QString::number(this->ui->ruf_argument_content_one->value(), 'f', 2)).toUtf8().constData();
    partial_node_name = partial_node_name + " Hz)]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name;
    node_to_modify->setText(0, QString::fromStdString(full_node_name)); 
    icon_pixmap = QPixmap(":/images/images/loop.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "REPEATER")
  {
    args = "n: ";
    args = args + asString(ui->repeater_argument_one_content);
    node_to_modify->setNodeAttributes(args);
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    node_to_modify->setNodeType(NodeType::REPEATER);
    partial_node_name = " [Execute all actions in sequence in a loop until ";
    partial_node_name = partial_node_name + (QString::number(this->ui->repeater_argument_one_content->value(), 'f', 2)).toUtf8().constData();
    partial_node_name = partial_node_name + " iterations are executed]";
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name;
    node_to_modify->setText(0, QString::fromStdString(full_node_name));
    icon_pixmap = QPixmap(":/images/images/loop.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  else if(combobox_content == "BEHAVIOR")
  {
    node_name = this->ui->name_content->text();
    node_to_modify->setNodeName(node_name.toUtf8().constData());
    std::string behavior_combobox_content = ui->behavior_combobox->currentText().toUtf8().constData();
    std::string behavior_mode_combobox_content = ui->behavior_mode_combobox->currentText().toUtf8().constData();
    node_to_modify->setNodeType(NodeType::TASK);
    node_to_modify->setBehaviorType(behavior_combobox_content);
    partial_node_name = " [" + behavior_combobox_content + "]";
    node_to_modify->setNodeAttributes(this->ui->behavior_content->toPlainText().toUtf8().constData());
    node_to_modify->setPartialNodeName(partial_node_name);
    full_node_name = node_name.toUtf8().constData() + partial_node_name; 
    node_to_modify->setText(0, QString::fromStdString(full_node_name));
    
    if(behavior_mode_combobox_content == "ACTIVATE BEHAVIOR")
    {
      node_to_modify->setActivate(true);
      node_to_modify->setRecurrent(true);
    } 
    else if (behavior_mode_combobox_content == "DEACTIVATE BEHAVIOR") 
    {
      node_to_modify->setActivate(false);
      node_to_modify->setRecurrent(true);
    }
    icon_pixmap = QPixmap(":/images/images/action.png");
    node_icon = QIcon(icon_pixmap);
    node_to_modify->setIcon(0,node_icon);
  }
  if(!node_name.isEmpty())
  { 
    if(!is_modifying)
    {
      padre_aux->addChild(node_to_modify); 
    }
    Q_EMIT(BehaviorDialog::windowAccepted(node_to_modify));
    Q_EMIT(close());
  }
  else
  {
    QMessageBox error_message;
    error_message.setWindowTitle(QString::fromStdString("Node parameters error"));
    error_message.setText(QString::fromStdString("The node name cannot be empty. Please insert a name."));
    error_message.exec();
  }
}

void BehaviorDialog::nodeTypeComboBoxChanged(const QString &text)
{
  std::string texto = text.toUtf8().constData();
  NodeType nodetype = treeitem_clicked->stringToNodeType(texto);
  int nodeAux;

  hideAllWidgets();
  resize(minimumSize());

  switch(nodetype)
  {
    case NodeType::ADD_BELIEF:
    {
      ui->add_belief_content->show();
      resize(minimumSize());
      break;
    }
    case NodeType::QUERY:
    {
      ui->query_content->show();
      resize(minimumSize());
      break;
    }
    case NodeType::SEQUENCE:
    {
      ui->name_label->hide();
      ui->name_content->hide();
      ui->type_label->hide();
      ui->node_type_combobox->hide();
      ui->name_label->show();
      ui->name_content->show();
      ui->type_label->show();
      ui->node_type_combobox->show();
      resize(minimumSize());
      break;
    }
    case NodeType::PARALLEL:
    {
      ui->parallel_argument_content_one->show();
      ui->parallel_argument_one->show();
      resize(minimumSize());
      break;
    }
    case NodeType::REMOVE_BELIEF:
    {
      ui->remove_belief_content->show();
      resize(minimumSize());
      break;
    }
    case NodeType::REPEATER:
    {
      ui->repeater_argument_one_content->show();
      ui->repeater_argument_one_label->show();
      resize(minimumSize());
      break;
    }
    case NodeType::REPEAT_UNTIL_FAIL:
    {
      ui->ruf_argument_content_one->show();
      ui->ruf_argument_one->show();
      resize(minimumSize());
      break;
    }
    case NodeType::SUCCEEDER:
    {
      ui->name_label->hide();
      ui->name_content->hide();
      ui->type_label->hide();
      ui->node_type_combobox->hide();
      ui->name_label->show();
      ui->name_content->show();
      ui->type_label->show();
      ui->node_type_combobox->show();
      resize(minimumSize());
      break;
    }
    case NodeType::INVERTER:
    {
      ui->name_label->hide();
      ui->name_content->hide();
      ui->type_label->hide();
      ui->node_type_combobox->hide();
      ui->name_label->show();
      ui->name_content->show();
      ui->type_label->show();
      ui->node_type_combobox->show();
      resize(minimumSize());
      break;
    }
    case NodeType::SELECTOR:
    {
      ui->name_label->hide();
      ui->name_content->hide();
      ui->type_label->hide();
      ui->node_type_combobox->hide();
      ui->name_label->show();
      ui->name_content->show();
      ui->type_label->show();
      ui->node_type_combobox->show();
      resize(minimumSize());
      break;
    }  
    case NodeType::TASK:
    {
      ui->arguments_label->show();
      ui->behavior_combobox->show();
      ui->behavior_content->show();
      ui->behavior_label->show();
      ui->mode_label->show();
      ui->behavior_mode_combobox->show();
      resize(minimumSize());
      break;
    }  
  }
}
  /*
  This function detects any change in the BehaviorType ComboBox to show the proper windows
  */
void BehaviorDialog::behaviorComboBoxChanged(const QString &text)
{
  //Placeholder if different behaviors need different selection screens
}

std::string BehaviorDialog::asString(QAbstractSpinBox * widget) 
{
  QString str = widget->text();
  return str.replace(QLocale().decimalPoint(), QLatin1Char('.')).toUtf8().constData();
}
