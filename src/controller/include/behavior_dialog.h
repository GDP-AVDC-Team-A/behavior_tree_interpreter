/*!*******************************************************************************************
 *  \file       behavior_dialog.h
 *  \brief      BehaviorDialog definition file.
 *  \details    The BehaviorDialog, along with behavior_tree, behavior_tree_visualizer, tree_item and execution_tree allows a user to create
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
#ifndef BEHAVIORDIALOG_H
#define BEHAVIORDIALOG_H


#include <ros/ros.h>
#include <droneMsgsROS/ConsultAvailableBehaviors.h>
#include <droneMsgsROS/ListOfBehaviors.h>
#include <QWidget>
#include <QDialog>
#include <QString>
#include <QStringList>
#include <QList>
#include <QSize>
#include <QPixmap>
#include <QIcon>
#include <QPushButton>
#include <QModelIndex>
#include <QVariant>
#include <QSizePolicy>
#include <QMessageBox>
#include <iostream>
#include "ui_behavior_dialog.h"
#include "tree_item.h"
#include "yaml-cpp/yaml.h"

namespace Ui {
class BehaviorDialog;
}

class BehaviorDialog : public QDialog
{
    Q_OBJECT

public:
  explicit BehaviorDialog(QWidget* parent = 0, TreeItem *padre = 0);
  BehaviorDialog(QWidget* parent, TreeItem* padre, TreeItem* treeitem_clicked);
  ~BehaviorDialog();   

protected:
  void modifyNode(TreeItem* padre, TreeItem* node_to_modify);

private:
  Ui::BehaviorDialog *ui;
  QGridLayout *my_layout;
  QPushButton *acceptButton;
  QPushButton *cancelButton;
  TreeItem *padre;
  TreeItem *treeitem_clicked;

  ros::NodeHandle n;
  std::vector<std::string> available_behaviors;
  std::string consult_available_behaviors;
  ros::ServiceClient consult_available_behaviors_srv;
  bool is_modifying;

  /*!********************************************************************************************************************
  *  \brief      This method converts the contents of a SpinBox to a string.
  **********************************************************************************************************************/
  std::string asString(QAbstractSpinBox* widget);

  /*!********************************************************************************************************************
  *  \brief      This method hides all the dialog's widgets.
  **********************************************************************************************************************/
  void hideAllWidgets();

public Q_SLOTS:

  /*!********************************************************************************************************************
  *  \brief  This slot is executed when the accept button is pressed    
  **********************************************************************************************************************/
  void actionAccept();

  /*!********************************************************************************************************************
  *  \brief  This slot is executed when the cancel button is pressed
  **********************************************************************************************************************/
  void actionCancel();

  /*!********************************************************************************************************************
  *  \brief  This slot is executed when the user wants to modify a node
  **********************************************************************************************************************/
  void actionModify();

  /*!********************************************************************************************************************
  *  \brief  This slot is executed when the user selects a node type
  **********************************************************************************************************************/
  void nodeTypeComboBoxChanged(const QString &);

  /*!********************************************************************************************************************
  *  \brief  This slot is executed when the user selects a behavior
  **********************************************************************************************************************/
  void behaviorComboBoxChanged(const QString &);
  
Q_SIGNALS:

  /*!********************************************************************************************************************
  *  \brief    This signal is sent when the dialog window is accepted.  
  **********************************************************************************************************************/
  void windowAccepted(TreeItem*);
};

#endif // BEHAVIORDIALOG
