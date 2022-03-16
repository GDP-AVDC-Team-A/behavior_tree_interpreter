#include <gtest/gtest.h>
#include <QMessageBox>
#include <ros/ros.h>
#include "behavior_tree_control_view.h"
#include <cstdio>
#include <QApplication>


#include <iostream>



QApplication* app;
BehaviorTreeControlView* w;
int ar;
bool finished=false;
int total_subtests=0;
int passed_subtests=0;

void spinnerThread(){
    while(!finished){
        ros::spinOnce();
    }

}

void displaySubtestResults(){
    std::cout << "\033[1;33m TOTAL SUBTESTS: "<<total_subtests<<"\033[0m"<<std::endl;
    std::cout << "\033[1;33m SUBTESTS PASSED: "<<passed_subtests<<"\033[0m"<<std::endl;
    std::cout << "\033[1;33m % PASSED: "<<(passed_subtests*100.0)/(total_subtests*1.0)<<"\033[0m"<<std::endl;

}

void test(){
    total_subtests++;
    std::string response;
    std::cout << "\033[1;34m Does a Widget called Behavior Tree Control appear in the left half o the screen (centered in vertical axis)? (Y/N) \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in position or appearance of the widget\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }

    total_subtests++;
    std::cout << "\033[1;34m Does 'drone1' appear as vehicle name? (Y/N)\033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in receiving vehicle name\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Is 100% the battery charge? (Y/N)\033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in receiving battery charge\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Click on 'Colapse text' check box . Does the text between brackets in tree nodes desapear?(Y/N)\033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Not collapsing text correctly \033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Press the 'Start mission' button. Does 'Flight time' counter start counting time?(Y/N) \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in 'Flight time' counter\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }

    total_subtests++;

    std::cout << "\033[1;34m Does each node of the tree execute correctly (each one first turned blue and then green)?(Y/N) \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in the execution of nodes\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m When the execution of the tree has finished, has 'Flight time' counter stopped? \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in stopping flight timer \033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Click  the 'Start mission' button again, wait until 'GO TO POINT A' node is executing and then press 'Emergency land' button. Has the execution of the tree stopped (current and root node turned red and flight timer stopped)? (Y/N)\033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in performing emergency land\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Click  the 'Start mission' button again, wait until 'GO TO POINT A' node is executing and then press key 'y' in your keyboard (Behavior Tree Control window must be active). Has the execution of the tree stopped (current and root node turned red and flight timer stopped)? \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in keyboard teleoperation \033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;

    std::cout << "\033[1;34m Right click on the 'TAKE OFF' node. Does a context dialog with the option 'Execute tree from this node appear'? \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem creating context dialog to execute the from current node \033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Click  on 'Execute tree from this node' . Does the tree start executing and it finishes correctly? \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in starting the execution from a specific node\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }
    total_subtests++;
    std::cout << "\033[1;34m Click  the 'Start mission' button again, wait until 'GO TO POINT A' node is executing and then press 'Abort mission' button. Has the execution of the tree stopped (current and root node turned red and flight timer stopped)? (Y/N) \033[0m"<<std::endl;
    getline(std::cin, response);
    EXPECT_TRUE(response=="Y" ||response=="y");
    if(response!="Y" && response!="y"){
        std::cout << "\033[1;31m Problem in aborting the mission\033[0m\n"<<std::endl;
    }
    else{
        passed_subtests++;
    }


    displaySubtestResults();

    app->exit();
    finished=true;

}


TEST(GraphicalUserInterfaceTests, behaviorTreeControlTest)
{

    std::thread thr(&test);
    app->exec();
    thr.join();
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, ros::this_node::getName());
    system("cp -f $AEROSTACK_STACK/stack/ground_control_system/graphical_user_interface/behavior_tree_keyboard_control_process/src/controller/test/behavior_tree_mission_file.yaml $AEROSTACK_STACK/configs/drone1 ");
    system("bash $AEROSTACK_STACK/launchers/launcher_simulated_quadrotor_basic_3.0.sh");
    system("xfce4-terminal  \--tab --title \"Behavior Coordinator\"  --command \"bash -c 'roslaunch behavior_coordinator_process behavior_coordinator_process.launch --wait \
           my_stack_directory:=${AEROSTACK_STACK};exec bash'\" &");
    ar=argc;
    app= new QApplication (argc, nullptr);
    w= new BehaviorTreeControlView (argc, nullptr);
    w->show();

    std::thread thr(&spinnerThread);

    return RUN_ALL_TESTS();
    thr.join();


}
