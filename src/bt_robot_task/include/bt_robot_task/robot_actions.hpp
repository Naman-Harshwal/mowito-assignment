#ifndef ROBOT_ACTIONS_HPP
#define ROBOT_ACTIONS_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include <iostream>

class NavigateToEnterRoom : public BT::SyncActionNode {
public:
    NavigateToEnterRoom(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "[NAVIGATE] Robot entering room..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }
};

class OpenFridge : public BT::SyncActionNode {
public:
    OpenFridge(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "[OPEN] Robot opening fridge..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }
};

class IsFridgeOpen : public BT::ConditionNode {
public:
    IsFridgeOpen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    BT::NodeStatus tick() override {
        static bool open = false;
        open = !open;
        std::cout << "[CHECK] Fridge open? " << (open ? "YES" : "NO") << std::endl;
        return open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    static BT::PortsList providedPorts() { return {}; }
};

class PickApple : public BT::SyncActionNode {
public:
    PickApple(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "[PICK] Robot picking apple..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }
};

class ExitRoom : public BT::SyncActionNode {
public:
    ExitRoom(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "[EXIT] Robot exiting room..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }
};

#endif
