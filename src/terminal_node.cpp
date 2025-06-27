// terminal_node.cpp
#include <iostream>
#include <memory>
#include <thread>
#include <cstdio>
#include <array>
#include <cstdlib>                                         // for std::system
#include <ament_index_cpp/get_package_share_directory.hpp> // to get package path

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

class TerminalNode : public rclcpp::Node
{
public:
  TerminalNode()
      : Node("terminal_node")
  {
    dispatch_pub_ = this->create_publisher<std_msgs::msg::Empty>("/dispatch_request", 10);
    tbm_id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/tbm_id", 10);
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dispatch_pub_;
    start_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("/start_logging");
    stop_trigger_client_ = this->create_client<std_srvs::srv::Trigger>("/stop_logging");
    std::thread([this]()
                { MenuLoop(); })
        .detach();
    RCLCPP_INFO(this->get_logger(), "Terminal node started, publishing TBM IDs on /tbm_id");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tbm_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dispatch_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_trigger_client_;

  void MenuLoop()
  {
    while (rclcpp::ok())
    {
      std::cout << "\n========== TBM MENU ==========" << std::endl;
      std::cout << "1. Select TBM ID" << std::endl;
      std::cout << "2. Send mission command" << std::endl;
      std::cout << "3. SSH into AUV" << std::endl;
      std::cout << "4. Start post-processing operation" << std::endl;
      std::cout << "5. Exit" << std::endl;
      std::cout << "> ";

      int choice;
      if (!(std::cin >> choice))
        break;

      switch (choice)
      {
      case 1:
      {
        int tbm;
        std::cout << "Insert desired TBM number id -> ";
        std::cin >> tbm;
        std_msgs::msg::Int32 msg;
        msg.data = tbm;
        tbm_id_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published TBM ID: %d", tbm);
        break;
      }

      case 2:
      {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  			auto future = start_trigger_client_->async_send_request(request);
        std_msgs::msg::Empty msg;
        dispatch_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Dispatched MissionCommand request");
        break;
      }

      case 3:
      {
        RCLCPP_INFO(this->get_logger(), "Starting ROV log sync script...");

        std::string pkg_path = ament_index_cpp::get_package_share_directory("ctrl_station");
        std::string script_path = pkg_path + "/scripts/download_rov_data.sh";

        int ret = std::system(script_path.c_str());

        if (ret == 0)
          RCLCPP_INFO(this->get_logger(), "ROV log sync script executed successfully.");
        else
          RCLCPP_ERROR(this->get_logger(), "ROV log sync script failed with code %d", ret);

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  	    auto future = stop_trigger_client_->async_send_request(request);

        break;
      }

      case 4:
      {
        RCLCPP_INFO(this->get_logger(), "Starting post-processing Python script...");

        std::string pkg_path = ament_index_cpp::get_package_share_directory("ctrl_station");
        std::string script_path = pkg_path + "/scripts/post_processing.py";

        // Find most recent folder inside ~/rov_logs
        std::string rov_logs_path = std::string(std::getenv("HOME")) + "/rov_logs";
        std::string find_latest_cmd = "ls -1t " + rov_logs_path + " | head -n 1";
        FILE *pipe = popen(find_latest_cmd.c_str(), "r");

        if (!pipe)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to retrieve latest mission folder.");
          break;
        }

        char buffer[128];
        std::string latest_folder;
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        {
          latest_folder = std::string(buffer);
          latest_folder.erase(std::remove(latest_folder.begin(), latest_folder.end(), '\n'), latest_folder.end());
        }
        pclose(pipe);

        if (latest_folder.empty())
        {
          RCLCPP_ERROR(this->get_logger(), "No mission folder found in %s", rov_logs_path.c_str());
          break;
        }

        std::string mission_path = rov_logs_path + "/" + latest_folder;
        std::string cmd = "python3 " + script_path + " \"" + mission_path + "\"";

        int ret = std::system(cmd.c_str());

        if (ret == 0)
          RCLCPP_INFO(this->get_logger(), "Post-processing script executed successfully.");
        else
          RCLCPP_ERROR(this->get_logger(), "Post-processing script failed with code %d", ret);
              
        break;
      }

      case 5:
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerminalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
