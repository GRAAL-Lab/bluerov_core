// terminal_node.cpp
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"

class TerminalNode : public rclcpp::Node
{
public:
  
  TerminalNode()
  : Node("terminal_node")
  {
    dispatch_pub_ = this->create_publisher<std_msgs::msg::Empty>("/dispatch_request", 10);
    tbm_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/tbm_id", 10);
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dispatch_pub_;
    std::thread([this]() { MenuLoop(); }).detach();
    RCLCPP_INFO(this->get_logger(), "Terminal node started, publishing TBM IDs on /tbm_id");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tbm_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dispatch_pub_;

  void MenuLoop()
  {
    while (rclcpp::ok()) {
      std::cout << "\n========== TBM MENU ==========" << std::endl;
      std::cout << "1. Select TBM ID" << std::endl;
      std::cout << "2. Send mission command" << std::endl;
      std::cout << "3. SSH into AUV (not yet merged in this node)" << std::endl;
      std::cout << "4. Exit" << std::endl;
      std::cout << "> ";

      int choice;
      if (!(std::cin >> choice)) break;

      switch (choice) {
        case 1: {
          int tbm;
          std::cout << "Insert desired TBM number id -> ";
          std::cin >> tbm;
          std_msgs::msg::Int32 msg;
          msg.data = tbm;
          tbm_id_pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Published TBM ID: %d", tbm);
          break;
        }
        
        case 2: {
	    std_msgs::msg::Empty msg;
	    dispatch_pub_->publish(msg);
	    RCLCPP_INFO(this->get_logger(), "Dispatched MissionCommand request");
	    break;
        }

        case 4:
          RCLCPP_INFO(this->get_logger(), "Exit requested");
          rclcpp::shutdown();
          return;

        default:
          std::cout << "Scelta non valida, riprova." << std::endl;
          break;
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerminalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
