#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <json_utils.hpp>
#include <memory>
#include <string>
#include <mqtt/async_client.h>

#include "ament_index_cpp/get_package_share_directory.hpp"


using namespace std::chrono_literals;

using namespace ctljsn;

std::string mqttAddress_ = "";
std::string clientId_ = "";
std::string username_ = "";
std::string password_ = "";
std::string topicUpdate_ = "";
std::string topicStatus_ = "";
int qos_ = 0;

class MqttClient : public rclcpp::Node, public virtual mqtt::callback
{
public:
	MqttClient()
		: Node("mqtt_client")
	{
		std::string config_path = ament_index_cpp::get_package_share_directory("ctrl_station") + "/conf/mqtt_config.conf";

		std::map<std::string, std::string> config;
		
		bool mqttConfigLoaded = false;
		
		try
		{
			mqttConfigLoaded = LoadMqttConf();
			RCLCPP_INFO(this->get_logger(), "MQTT configuration data successfully loaded");
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "Error loading MQTT configuration data: %s", e.what());
		}
		
		pub_lat_ = this->create_publisher<std_msgs::msg::Float64>("latitude", 10);
		pub_long_ = this->create_publisher<std_msgs::msg::Float64>("longitude", 10);

		client_ = std::make_unique<mqtt::async_client>(mqttAddress_, clientId_);
		mqtt::connect_options connOpts;
		connOpts.set_user_name(username_);
		connOpts.set_password(password_);
		connOpts.set_clean_session(true);

		client_->set_callback(*this);

		bool connected = false;

		int attempts = 0;
		while (!connected)
		{
			try
			{
				RCLCPP_INFO(this->get_logger(), "Attempt %d: Connecting to MQTT broker...", attempts + 1);
				client_->connect(connOpts)->wait();
				RCLCPP_INFO(this->get_logger(), "Connected to broker.");
				client_->subscribe(topicUpdate_, qos_)->wait();
				client_->subscribe(topicStatus_, qos_)->wait();
				connected = true;
			}
			catch (const mqtt::exception &e)
			{
				RCLCPP_WARN(this->get_logger(), "Connection attempt %d failed: %s", attempts++, e.what());
				std::this_thread::sleep_for(std::chrono::seconds(2));
			}
		}
		if (!connected)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker after multiple attempts.");
		}
		attempts++;
	}

	~MqttClient()
	{
		try
		{
			if (client_ && client_->is_connected())
			{
				RCLCPP_INFO(this->get_logger(), "Disconnecting MQTT client...");
				client_->disconnect()->wait();
			}
		}
		catch (const mqtt::exception &e)
		{
			RCLCPP_WARN(this->get_logger(), "Error during MQTT disconnect: %s", e.what());
		}
	}

private:
	std::unique_ptr<mqtt::async_client> client_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lat_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_long_;

	void message_arrived(mqtt::const_message_ptr msg) override
	{
		RCLCPP_INFO(this->get_logger(), "Received MQTT message: %s", msg->get_payload().c_str());
		try
		{
			// Usa solo jsoncons
			std::string raw = msg->get_payload();
			jsoncons::json wm_json = jsoncons::json::parse(raw);

			std::string type = "";
			// Verifica il tipo di messaggio
			if (wm_json.contains("header") && wm_json["header"].contains("message_type"))
			{
				type = wm_json["header"]["message_type"].as<std::string>();
			}

			if (type == "DYNAMIC_UPDATE")
			{
				double *lat_long = ctljsn::geographic::CreateLatLongPositionFromJson(wm_json);

				if (lat_long != nullptr)
				{
					auto msg_lat = std_msgs::msg::Float64();
					msg_lat.data = lat_long[0];
					pub_lat_->publish(msg_lat);
					RCLCPP_INFO(this->get_logger(), "Lat: %f", msg_lat.data);

					auto msg_long = std_msgs::msg::Float64();
					msg_long.data = lat_long[1];
					pub_long_->publish(msg_long);
					RCLCPP_INFO(this->get_logger(), "Lon: %f", msg_long.data);
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Failed to extract lat/long.");
				}
			}
			else if (type == "STATUS")
			{
				std::string status_msg = ctljsn::geographic::handle_status(wm_json);
				RCLCPP_INFO(this->get_logger(), "%s", status_msg.c_str());
			}
			else
				RCLCPP_WARN(this->get_logger(), "Message type not recognized");
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "Error parsing MQTT message: %s", e.what());
		}
	}
	
	bool LoadMqttConf(){
                  
	  std::string pkg = ament_index_cpp::get_package_share_directory("ctrl_station");
	  std::string path = pkg + "/conf/mqtt_config.conf";

	  std::ifstream file(path);
	  if (!file.is_open())
	  {
	    RCLCPP_ERROR(rclcpp::get_logger("~"), "Impossibile aprire il file di configurazione: %s", path.c_str());
	    return false;
	  }

	  std::map<std::string, std::string> config;
	  std::string line;
	  while (std::getline(file, line))
	  {
	    if (line.empty() || line[0] == '#') continue;
	    std::istringstream iss(line);
	    std::string key, value;
	    if (std::getline(iss, key, '=') && std::getline(iss, value))
	    {
	      config[key] = value;
	    }
	  }

	  try {
	    mqttAddress_ = config.at("mqtt_address");
	    clientId_ = config.at("client_id");
	    username_   = config.at("username");
	    password_   = config.at("password");
	    topicUpdate_ = config.at("topic_update");
	    topicStatus_ = config.at("topic_status");
	    qos_ = std::stoi(config.at("qos"));
	  }
	  catch (const std::out_of_range &e) {
	    RCLCPP_ERROR(rclcpp::get_logger("~"), "Parametro mancante nel file conf: %s", e.what());
	    return false;
	  }
	  catch (const std::exception &e) {
	    RCLCPP_ERROR(rclcpp::get_logger("~"), "Errore elaborando conf MQTT: %s", e.what());
	    return false;
	  }

	  return true;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MqttClient>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
