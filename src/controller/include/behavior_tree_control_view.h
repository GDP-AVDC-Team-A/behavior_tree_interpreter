/*!*******************************************************************************************
 *  \file      behavior_tree_control_view.h
 *  \brief     Behavior Tree Control View definition file.
 *  \details   Layout for the Control Panel component. Widgets can be dynamically added to it.
 *  \author    Abraham Carrera, Daniel Del Olmo
********************************************************************************************/

#ifndef BEHAVIOR_TREE_CONTROL_VIEW_H
#define BEHAVIOR_TREE_CONTROL_VIEW_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include <droneMsgsROS/openMissionFile.h>
#include <aerostack_msgs/ListOfBeliefs.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
 
#include <QWidget>
#include <QRect>
#include <QGuiApplication>
#include <QScreen>
#include <QProcess>
#include <QKeyEvent>
#include <QMap>
#include <QCloseEvent>

#include "behavior_tree_control.h"
#include "ui_behavior_tree_control_view.h"


#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "fstream"

namespace Ui {
class BehaviorTreeControlView;
}

class BehaviorTreeControl;

class BehaviorTreeControlView : public QWidget
{
  Q_OBJECT

public:
  //Constructor & Destructor
  explicit BehaviorTreeControlView(int argc, char** argv, QWidget *parent = 0);
  ~BehaviorTreeControlView();

  /*!********************************************************************************************************************
   *  \brief      This method kills the process when mission load is failed
   *********************************************************************************************************************/
  //void loadTreeError();

  BehaviorTreeControl* getBehaviorTreeControl();

private:
  Ui::BehaviorTreeControlView *ui;

  ros::NodeHandle n;
  ros::ServiceClient activate_task_srv;

  ros::Publisher window_event_pub;
  ros::Subscriber window_event_sub;

  std::string window_event_topic; 
  std::string start_task;



  BehaviorTreeControl *behavior_tree_control;
  
  std::string drone_id_namespace;
  std::string activate_behavior;


  boost::property_tree::ptree root;

  /*!************************************************************************
   *  \brief  Kills the process
   ***************************************************************************/
  void killMe();
  
  /*!********************************************************************************************************************
  *  \brief      This method is the responsible for seting up connections.
  *********************************************************************************************************************/
  void setUp();

  void setWidgetDimensions();
  int heightV= 790;
  int widthV=500;
  int position_x=-845;
  int position_y=-395;
public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief      This method notifies main window that the widget was closed
   *********************************************************************************************************************/
  //void closeEvent (QCloseEvent *event);

  /*!************************************************************************
  *  \brief   Activated when a window is closed.
  ***************************************************************************/
  //void windowOpenCallback(const aerostack_msgs::WindowEvent &msg);


Q_SIGNALS:

};

#endif // BEHAVIOR_TREE_CONTROL_VIEW_H
