/*
Name: NavigationBTNode.cpp
Author: Manasa
Date: 2026-06-26
Version: 1.0
Description: Behavior Tree node for Navigation State Machine. 
             Listens for start/stop triggers and enables the line follower.
*/

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;

// Custom Condition Node: Checks if Navigation has been started
class IsNavigationStarted : public BT::ConditionNode
{
public:
  IsNavigationStarted(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("is_started") }; }

  BT::NodeStatus tick() override
  {
    bool started = false;
    if (getInput("is_started", started) && started) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

// Custom Action Node: Calls /line_follower/enable to enable/disable
class SetLineFollowerState : public BT::SyncActionNode
{
public:
  SetLineFollowerState(const std::string& name, const BT::NodeConfig& config, rclcpp::Node* node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    client_ = node_->create_client<std_srvs::srv::SetBool>("/line_follower/enable");
  }

  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("enable") }; }

  BT::NodeStatus tick() override
  {
    bool enable_state = false;
    if (!getInput("enable", enable_state)) {
      throw BT::RuntimeError("missing required input [enable]");
    }

    if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /line_follower/enable service...");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable_state;
    
    // We send request asynchronously but return SUCCESS immediately because 
    // we don't want to block the tree.
    client_->async_send_request(request);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node* node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

// Custom Action Node: Publish Nav State string
class PublishNavState : public BT::SyncActionNode
{
public:
  PublishNavState(const std::string& name, const BT::NodeConfig& config, rclcpp::Node* node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    pub_ = node_->create_publisher<std_msgs::msg::String>("/navigation/state", 10);
  }

  static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("state_str") }; }

  BT::NodeStatus tick() override
  {
    std::string state_str;
    if (!getInput("state_str", state_str)) {
      throw BT::RuntimeError("missing required input [state_str]");
    }
    std_msgs::msg::String msg;
    msg.data = state_str;
    pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class NavigationBTNode : public rclcpp::Node
{
public:
  NavigationBTNode() : Node("navigation_bt_node"), is_started_(false)
  {
    this->declare_parameter<std::string>("bt_xml_file", "");
    std::string xml_file;
    this->get_parameter("bt_xml_file", xml_file);

    srv_start_ = this->create_service<std_srvs::srv::Trigger>(
      "/navigation/start", std::bind(&NavigationBTNode::srvStart, this, std::placeholders::_1, std::placeholders::_2));
    srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
      "/navigation/stop", std::bind(&NavigationBTNode::srvStop, this, std::placeholders::_1, std::placeholders::_2));
    srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "/navigation/reset", std::bind(&NavigationBTNode::srvReset, this, std::placeholders::_1, std::placeholders::_2));

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<IsNavigationStarted>("IsNavigationStarted");
    
    // Register actions passing the ROS node pointer
    factory.registerBuilder<SetLineFollowerState>("SetLineFollowerState",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<SetLineFollowerState>(name, config, this);
      });

    factory.registerBuilder<PublishNavState>("PublishNavState",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<PublishNavState>(name, config, this);
      });

    if (xml_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), "bt_xml_file parameter is empty!");
      return;
    }

    tree_ = factory.createTreeFromFile(xml_file);

    // Provide the blackboard with the initial state
    tree_.rootBlackboard()->set<bool>("is_started", is_started_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10Hz is plenty for high level state
      std::bind(&NavigationBTNode::tickTree, this));

    RCLCPP_INFO(this->get_logger(), "Navigation BT Node started with tree: %s", xml_file.c_str());
  }

private:
  void tickTree()
  {
    // Update blackboard with current state before ticking
    tree_.rootBlackboard()->set<bool>("is_started", is_started_);
    tree_.tickExactlyOnce();
  }

  void srvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    is_started_ = true;
    res->success = true;
    res->message = "Navigation Started";
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  void srvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    is_started_ = false;
    res->success = true;
    res->message = "Navigation Stopped";
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  void srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    is_started_ = false;
    res->success = true;
    res->message = "Navigation Reset";
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  BT::Tree tree_;
  bool is_started_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationBTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
