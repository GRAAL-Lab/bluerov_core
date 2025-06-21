#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <json_utils.hpp>
#include <memory>
#include <string>
#include <mqtt/async_client.h>

using namespace std::chrono_literals;

using namespace ctljsn;

const std::string mqttAdress = "tcp://localhost:1883";
const std::string clientId = "ros2_client";
const std::string username = "teamA";
const std::string password = "passwordA";
const std::string topicUpdate = "catl/rami25/sectorA/update";
const std::string topicStatus  = "catl/rami25/sectorA/status";
const int qos = 1;

class MqttClient : public rclcpp::Node, public virtual mqtt::callback {
public:
    MqttClient()
    : Node("mqtt_client") {
        pub_lat_ = this->create_publisher<std_msgs::msg::Float64>("latitude", 10);
        pub_long_ = this->create_publisher<std_msgs::msg::Float64>("longitude", 10);

        client_ = std::make_unique<mqtt::async_client>(mqttAdress, clientId);
        mqtt::connect_options connOpts;
        connOpts.set_user_name(username);
        connOpts.set_password(password);
        connOpts.set_clean_session(true);

        client_->set_callback(*this);

        bool connected = false;
        
        int attempts = 0;
	while(!connected) {
	    try {
		RCLCPP_INFO(this->get_logger(), "Attempt %d: Connecting to MQTT broker...", attempts + 1);
		client_->connect(connOpts)->wait();
		RCLCPP_INFO(this->get_logger(), "Connected to broker.");
		client_->subscribe(topicUpdate, qos)->wait();
		client_->subscribe(topicStatus, qos)->wait();
		connected = true;
	    } catch (const mqtt::exception& e) {
		RCLCPP_WARN(this->get_logger(), "Connection attempt %d failed: %s", attempts + 1, e.what());
		std::this_thread::sleep_for(std::chrono::seconds(2));
	    }
	}
	if (!connected) {
	    RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker after multiple attempts.");
	}
	attempts++;
    }
    
    ~MqttClient() {
	    try {
		if (client_ && client_->is_connected()) {
		    RCLCPP_INFO(this->get_logger(), "Disconnecting MQTT client...");
		    client_->disconnect()->wait();
		}
	    } catch (const mqtt::exception& e) {
		RCLCPP_WARN(this->get_logger(), "Error during MQTT disconnect: %s", e.what());
	    }
    }

private:
    std::unique_ptr<mqtt::async_client> client_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lat_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_long_;

    void message_arrived(mqtt::const_message_ptr msg) override {
            RCLCPP_INFO(this->get_logger(), "Received MQTT message: %s", msg->get_payload().c_str());
	    try {
		// Usa solo jsoncons
		std::string raw = msg->get_payload();
		jsoncons::json wm_json = jsoncons::json::parse(raw);

		std::string type = "";
		// Verifica il tipo di messaggio
		if (wm_json.contains("header") && wm_json["header"].contains("message_type")){
			type = wm_json["header"]["message_type"].as<std::string>();
		}
		
		if (type == "DYNAMIC_UPDATE"){
			double* lat_long = ctljsn::geographic::CreateLatLongPositionFromJson(wm_json);

			if (lat_long != nullptr) {
			    auto msg_lat = std_msgs::msg::Float64();
			    msg_lat.data = lat_long[0];
			    pub_lat_->publish(msg_lat);
			    RCLCPP_INFO(this->get_logger(), "Lat: %f", msg_lat.data);

			    auto msg_long = std_msgs::msg::Float64();
			    msg_long.data = lat_long[1];
			    pub_long_->publish(msg_long);
			    RCLCPP_INFO(this->get_logger(), "Lon: %f", msg_long.data);
			} else {
			    RCLCPP_WARN(this->get_logger(), "Failed to extract lat/long.");
			}
		}
		else if (type == "STATUS"){
			std::string status_msg = ctljsn::geographic::handle_status(wm_json);
			RCLCPP_INFO(this->get_logger(), "%s", status_msg.c_str());
		}
		else
			RCLCPP_WARN(this->get_logger(), "Message type not recognized");
		

	    } catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error parsing MQTT message: %s", e.what());
	    }
   }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MqttClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

