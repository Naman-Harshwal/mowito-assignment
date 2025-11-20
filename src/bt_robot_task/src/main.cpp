#include "behaviortree_cpp/bt_factory.h"
#include "bt_robot_task/robot_actions.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<NavigateToEnterRoom>("NavigateToEnterRoom");
    factory.registerNodeType<OpenFridge>("OpenFridge");
    factory.registerNodeType<PickApple>("PickApple");
    factory.registerNodeType<ExitRoom>("ExitRoom");
    factory.registerNodeType<IsFridgeOpen>("IsFridgeOpen");
    
    auto tree = factory.createTreeFromFile("/home/darkfate/ros2_bt_test_ws/src/bt_robot_task/src/robot_task.xml");
    
    std::cout << "\n=== Robot Apple Picking Task ===\n" << std::endl;
    tree.tickWhileRunning();
    std::cout << "\n=== TASK COMPLETE ===\n" << std::endl;
    
    return 0;
}
