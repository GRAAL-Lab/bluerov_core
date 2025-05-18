
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "auv_core_helper/msg/lat_long.hpp"
#include <json_utils.hpp>
#include <memory>

#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

#define SERVER_IP "127.0.0.1"
#define PORT 8080
#define BUFFER_SIZE 2048

using namespace ctljsn;

using namespace std::chrono_literals;

struct DataStruct {
    int msg_type;
    double latitude;
    double longitude;
    std::string time;
};

bool enableDebugPrint = false;
int sockfd;
struct sockaddr_in clientAddr;
size_t total_sent = 0;
int client_socket;
struct sockaddr_in server_addr;
char buffer[BUFFER_SIZE] = {0};
std::string json_string;
int valread;
jsoncons::json received_json;
socklen_t addrLen;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
     
    pub_lat = this->create_publisher<std_msgs::msg::Float64>("latitude", 10);
    pub_long = this->create_publisher<std_msgs::msg::Float64>("longitude", 10);
    
    // Creazione socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Errore creazione socket");
        //return 1;
    }

    clientAddr.sin_family = AF_INET;
    clientAddr.sin_addr.s_addr = INADDR_ANY;
    clientAddr.sin_port = htons(PORT);

    // Bind socket
    if (bind(sockfd, (const struct sockaddr*)&clientAddr, sizeof(clientAddr)) < 0) {
        perror("Errore bind");
        close(sockfd);
        //return 1;
    }
    
    std::cerr << "Client in ascolto sulla porta 8080" << std::endl;

    addrLen = sizeof(clientAddr);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    
    int i = 1;
    int n_parsed = 0;
    
    while ((valread = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &addrLen)) > 0) {
            n_parsed = tryDecode(buffer, valread, i, n_parsed, json_string);
 
            if(n_parsed == 1){
            	i = 0;
            	n_parsed = 0;
            	break;
            }
    	}
  }
  
  void extractInfo(jsoncons::json wm_json){
    std::cerr << "Decode:\n" << std::endl;

    std::shared_ptr<ctljsn::time::AbsoluteTime> extractedAbsTime = ctljsn::time::CreateAbsoluteTimeFromJson(wm_json);
    std::shared_ptr<ctljsn::time::DirectTime> extractedDirTime = std::dynamic_pointer_cast<ctljsn::time::DirectTime>(extractedAbsTime);
    
    double*  lat_long = ctljsn::geographic::CreateLatLongPositionFromJson(wm_json);

    if (extractedDirTime)
        std::cerr << "Extracted Absolute Time:\n" << extractedDirTime->data << std::endl;
    else
        std::cerr << "Extracted Absolute Time is not a DirectTime instance." << std::endl;
    
    if (lat_long!=nullptr)
    {	
    	auto message_lat = std_msgs::msg::Float64();
	message_lat.data = lat_long[0];
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_lat.data);
	
	auto message_long = std_msgs::msg::Float64();
	message_long.data = lat_long[1];
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_long.data);
	
	pub_lat->publish(message_lat);
	pub_long->publish(message_long);
    	
    }
        
    else
        std::cerr << "Error extracting Lat Long position" << std::endl;
  }

  int tryDecode(char buffer[2048], int valread, int& i, int n_parsed, std::string json_string){
    json_string.append(buffer, valread);

         // Inizializziamo i a 1
        while (i <= json_string.size()) {
            try {
                // parsare dalla posizione 0 fino a i
                jsoncons::json received_json = jsoncons::json::parse(json_string.substr(0, i));
                extractInfo(received_json);
                n_parsed++;
            
                //std::cerr << "i value:"<< i << std::endl;
                
                // rimuovo il JSON già elaborato dalla stringa
                json_string = json_string.substr(i);

                // reset di i per ripartire con il nuovo contenuto rimanente
                i = 1;
            } catch (const jsoncons::ser_error&) {
                // Se il parsing fallisce, proviamo con una parte più grande della stringa
                i++;
            }
        }
	
        return n_parsed;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lat;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_long;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
