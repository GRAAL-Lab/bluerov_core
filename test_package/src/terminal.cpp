#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <libconfig.h++>

#include "auv_core_helper/srv/mission_command.hpp"
#include "auv_core_helper/msg/lat_long.hpp"
#include "auv_core_helper/msg/dtc_area.hpp"
#include "auv_core_helper/msg/dtc_action_map.hpp"
#include "auv_core_helper/msg/pipeline_pipe.hpp"

#include <mission_ctrl/mission_data_structs.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"


using MissionCommand = auv_core_helper::srv::MissionCommand;
using namespace mission;

class TBMConfigLoaderNode : public rclcpp::Node
{
public:
    std::shared_ptr<TaskBenchmarkSettings> conf;
    
    TBMConfigLoaderNode() : Node("terminal")
    {
        RCLCPP_INFO(this->get_logger(), "Starting TBM Configuration Loader Node");

        
        if (LoadConfiguration(conf)) {
            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");

            // Stampare dettagli del TBM caricato
            if (auto inspection = std::dynamic_pointer_cast<Inspection>(conf)) {
                RCLCPP_INFO(this->get_logger(), "Loaded TBM: Inspection with %ld pipeline structures", inspection->pipelineStructures.size());
            } else if (auto intervention = std::dynamic_pointer_cast<Intervention>(conf)) {
                RCLCPP_INFO(this->get_logger(), "Loaded TBM: Intervention");
            } else if (auto combo = std::dynamic_pointer_cast<InspectionAndIntervention>(conf)) {
                RCLCPP_INFO(this->get_logger(), "Loaded TBM: Inspection + Intervention");
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown TBM type");
            }
            
            // Stampa i dettagli comuni
	    RCLCPP_INFO(this->get_logger(), "Selected Pipeline Structure ID: %d", conf->selectedPipelineStructureId);

	    RCLCPP_INFO(this->get_logger(), "Pipeline Structures:");
	    for (const auto& ps : conf->pipelineStructures) {
		RCLCPP_INFO(this->get_logger(), "  - ID: %d", ps.id);
		RCLCPP_INFO(this->get_logger(), "    Centroid: [%.6f, %.6f]", ps.centroid.latitude, ps.centroid.longitude);
	    }

	    RCLCPP_INFO(this->get_logger(), "Buoys Area Enabled: %s", conf->buoysArea.enabled ? "true" : "false");
	    if (conf->buoysArea.enabled) {
		RCLCPP_INFO(this->get_logger(), "  Centroid: [%.6f, %.6f]",
		            conf->buoysArea.centroid.latitude, conf->buoysArea.centroid.longitude);
		RCLCPP_INFO(this->get_logger(), "  Size: [%.2f, %.2f]",
		            conf->buoysArea.size[0], conf->buoysArea.size[1]);
		RCLCPP_INFO(this->get_logger(), "  Orientation: %.2f", conf->buoysArea.orientation);
	    }
	    
	    //Stampa dettagli specifici Inspection
	    if (auto inspection = std::dynamic_pointer_cast<Inspection>(conf)) {
	    	auto c = std::dynamic_pointer_cast<Inspection>(conf);
		RCLCPP_INFO(this->get_logger(), "UAV Waypoint: [%.6f, %.6f]", c->uavWaypoint.latitude, c->uavWaypoint.longitude);
		RCLCPP_INFO(this->get_logger(), "Number of Buoys: %d", c->numberOfBuoys);

		RCLCPP_INFO(this->get_logger(), "Buoys Actions:");
		for (const auto& kv : c->buoysActions) {
		    std::string action_str = BuoyActionToString(kv.first);
    		    RCLCPP_INFO(this->get_logger(), "  %s -> %d", action_str.c_str(), static_cast<int>(kv.second));
		}

		RCLCPP_INFO(this->get_logger(), "Pipeline Pipes:");
		for (const auto& pipe : c->pipelinePipes) {
		    RCLCPP_INFO(this->get_logger(), "  - Number: %d", pipe.number);
		    RCLCPP_INFO(this->get_logger(), "    Angle: %.2f", pipe.angleWithNorth);
		    RCLCPP_INFO(this->get_logger(), "    Centroid: [%.6f, %.6f]", pipe.position.latitude, pipe.position.longitude);
		}
	    //Stampa dettagli specifici Intervention
            } else if (auto intervention = std::dynamic_pointer_cast<Intervention>(conf)) {
                 auto c = std::dynamic_pointer_cast<Intervention>(conf);
                 RCLCPP_INFO(this->get_logger(), "Number of Main Pipe Damage Markers: %d", c->numberOfMainPipeDamageMarkers);
		 RCLCPP_INFO(this->get_logger(), "Damaged Pipe On Pipeline:");
		 RCLCPP_INFO(this->get_logger(), "  - Number: %d", c->damagedPipeOnPipeline.number);
		 RCLCPP_INFO(this->get_logger(), "  - Angle: %.2f", c->damagedPipeOnPipeline.angleWithNorth);
		 RCLCPP_INFO(this->get_logger(), "  - Position: [%.6f, %.6f]", c->damagedPipeOnPipeline.position.latitude, c->damagedPipeOnPipeline.position.longitude);
            
            //Stampa dettagli specifici Inspection + Intervention
            } else if (auto combo = std::dynamic_pointer_cast<InspectionAndIntervention>(conf)) {
            	    auto c = std::dynamic_pointer_cast<InspectionAndIntervention>(conf);
                    RCLCPP_INFO(this->get_logger(), "[TBM] Type: Inspection + Intervention");

		    RCLCPP_INFO(this->get_logger(), "Number of Main Pipe Damage Markers: %d", c->numberOfMainPipeDamageMarkers);
		    RCLCPP_INFO(this->get_logger(), "Number of Buoys: %d", c->numberOfBuoys);

		    RCLCPP_INFO(this->get_logger(), "Buoys Actions:");
		    for (const auto& kv : c->buoysActions) {
			std::string action_str = mission::BuoyActionToString(kv.first);
			std::string color_str = mission::BuoyColorToString(kv.second);
			RCLCPP_INFO(this->get_logger(), "  %s -> %s", action_str.c_str(), color_str.c_str());
		    }

		    RCLCPP_INFO(this->get_logger(), "Pipeline Pipes:");
		    for (const auto& pipe : c->pipelinePipes) {
			RCLCPP_INFO(this->get_logger(), "  - Number: %d", pipe.number);
			RCLCPP_INFO(this->get_logger(), "    Angle: %.2f", pipe.angleWithNorth);
			RCLCPP_INFO(this->get_logger(), "    Position: [%.6f, %.6f]", pipe.position.latitude, pipe.position.longitude);
		    }  
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load TBM configuration");
        }
        
        //Inizializzazione subscription
        
        latitude_sub_ = this->create_subscription<std_msgs::msg::Float64>("/latitude", 10, std::bind(&TBMConfigLoaderNode::latitudeCallback, this, std::placeholders::_1));

        longitude_sub_ = this->create_subscription<std_msgs::msg::Float64>("/longitude", 10, std::bind(&TBMConfigLoaderNode::longitudeCallback, this, std::placeholders::_1));

    }

private:
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr latitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr longitude_sub_;

    double last_latitude_ = 0.0;
    double last_longitude_ = 0.0;
    
    void latitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_latitude_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "[/latitude] Received: %.6f", last_latitude_);
    }

    void longitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_longitude_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "[/longitude] Received: %.6f", last_longitude_);
    }


    bool LoadConfiguration(std::shared_ptr<TaskBenchmarkSettings>& conf)
    {
        std::string fileName_ = "tasks.conf";
        libconfig::Config confObj;
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_package");
        std::string confPath = package_share_directory + "/conf/" + fileName_;

        try {
            RCLCPP_INFO(this->get_logger(), "Reading config from: %s", confPath.c_str());
            confObj.readFile(confPath.c_str());

            uint tbm_id;
            if (!ctb::GetParam(confObj, tbm_id, "tbm")) {
                RCLCPP_ERROR(this->get_logger(), "Missing 'tbm' parameter in config");
                return false;
            }

            switch (tbm_id) {
                case 1:
                    conf = std::make_shared<Inspection>();
                    break;
                case 2:
                    conf = std::make_shared<Intervention>();
                    break;
                case 3:
                    conf = std::make_shared<InspectionAndIntervention>();
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid tbm id: %u", tbm_id);
                    return false;
            }

            return conf->ConfigureFromFile(confObj);

        } catch (const libconfig::FileIOException& fioex) {
            RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
            RCLCPP_ERROR(this->get_logger(), "  Path: '%s'. Make sure the file exists and is readable.", confPath.c_str());
            return false;
        } catch (const libconfig::ParseException& pex) {
            RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d - %s", pex.getFile(), pex.getLine(), pex.getError());
            return false;
        }
    }
    
    std::string BuoyActionToString(mission::BuoyAction action)
    {
	    switch (action) {
		case mission::ClockWiseRotation:
		    return "ClockWiseRotation";
		case mission::CounterClockWiseRotation:
		    return "CounterClockWiseRotation";
		case mission::GoUp:
		    return "GoUp";
		case mission::GoDown:
		    return "GoDown";
		default:
		    return "Unknown";
	    }
    }


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TBMConfigLoaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
