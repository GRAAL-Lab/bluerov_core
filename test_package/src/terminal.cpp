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

#include <cstdint>


using MissionCommand = auv_core_helper::srv::MissionCommand;
using namespace mission;

class TBMConfigLoaderNode : public rclcpp::Node
{
public:
    std::shared_ptr<TaskBenchmarkSettings> conf;
    uint8_t tbm_id;
    rclcpp::Client<auv_core_helper::srv::MissionCommand>::SharedPtr client_;
   
    
    TBMConfigLoaderNode() : Node("terminal")
    {
    
    	client_ = this->create_client<auv_core_helper::srv::MissionCommand>("/auv/service/mission_cmd");
    	 
        RCLCPP_INFO(this->get_logger(), "Starting TBM Configuration Loader Node");

        //Inizializzazione configurazione
         
        if (LoadConfiguration(conf)) {
            log_configuration(conf);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load TBM configuration");
        }
        
        //Inizializzazione subscription
        
        latitude_sub_ = this->create_subscription<std_msgs::msg::Float64>("/latitude", 10, std::bind(&TBMConfigLoaderNode::latitudeCallback, this, std::placeholders::_1));

        longitude_sub_ = this->create_subscription<std_msgs::msg::Float64>("/longitude", 10, std::bind(&TBMConfigLoaderNode::longitudeCallback, this, std::placeholders::_1));
        
        //Lancio del terminale in un thread separato
        /*
        std::thread([this]() {
            this->menuLoop();
        }).detach();*/
        

    }
    
    void menuLoop() {
	    while (rclcpp::ok()) {
		std::cout << "\n========== TBM MENU ==========\n";
		std::cout << "1. Seleziona TBM e carica configurazione\n";
		std::cout << "2. Invia MissionCommand\n";
		std::cout << "3. Esci\n";
		std::cout << "Scelta: ";

		
		int choice;
		std::cin >> choice;
		
		switch (choice) {
		    case 1:
		    	tbm_id = 0;
		        std::cout<<"Insert desired TBM number id-> "<<std::endl;
		        std::cin>>tbm_id;
		        break;

		    case 2:
		    	char c_choice;
		        std::cout<<"If the TBM selected is the 1st make sure you've already seny waypoints latlong by communication, otherways you'll have default latlong waypoint, do you wanna proceed? [y/n]"<<std::endl;
		        std::cin>>c_choice;
		        if(c_choice == 'y'){
		        	
		        	

				// Inizializzazione nel costruttore della classe
				

	        		auto request = std::make_shared<MissionCommand::Request>();
	        		
	        		SendMissionCommand(request);
	        		
	        		if (!client_->wait_for_service(std::chrono::seconds(5))) {
				   RCLCPP_ERROR(this->get_logger(), "Service 'mission_command' not available.");
		    		    return;
				}
				
				auto result_callback = [this](rclcpp::Client<MissionCommand>::SharedFuture future) {
				
					if (future.valid()) {
						RCLCPP_ERROR(this->get_logger(), "Risposta ricevuta: %s - Messaggio: %s", future.get()->res ? "true" : "false", future.get()->text.c_str());
					} 
					else {
						RCLCPP_ERROR(this->get_logger(), "Errore nella chiamata al servizio");
					}
				};
				
				client_->async_send_request(request, result_callback);

				//auto future = client_->async_send_request(request);

				/*if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
				    rclcpp::FutureReturnCode::SUCCESS)
				{
				    RCLCPP_INFO(this->get_logger(), "Service response: %s", future.get()->res ? "true" : "false");
				} else {
				    RCLCPP_ERROR(this->get_logger(), "Failed to call service 'mission_command'");
				}*/
				
		
		        }
		        break;

		    case 3:
		        RCLCPP_INFO(this->get_logger(), "Uscita richiesta.");
		        rclcpp::shutdown();
		        return;

		    default:
		        std::cout << "Scelta non valida, riprova.\n";
		        break;
		}
		
	    }
    }

private:
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr latitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr longitude_sub_;

    
    void latitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        auto c = std::dynamic_pointer_cast<mission::Inspection>(conf);
        if (c) {
             c->uavWaypoint.latitude = msg->data;
        }
        RCLCPP_INFO(this->get_logger(), "[/latitude] Received: %.6f", c->uavWaypoint.latitude);
    }
    
    void longitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        auto c = std::dynamic_pointer_cast<mission::Inspection>(conf);
        if (c) {
             c->uavWaypoint.longitude = msg->data;
        }
        RCLCPP_INFO(this->get_logger(), "[/longitude] Received: %.6f", c->uavWaypoint.longitude);
        
        log_configuration(conf);
    }
    
    void log_configuration(std::shared_ptr<TaskBenchmarkSettings> conf){
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
    }
    
    void SendMissionCommand(std::shared_ptr<MissionCommand::Request> request)
    {  	 	 
		// --- 1) Campi comuni ---
    request->tbm_id = 1;
    request->selected_pipeline_structure_id = conf->selectedPipelineStructureId;

    // pipeline_structures è std::array di dimensione fissa 2
    if (conf->pipelineStructures.size() > 0) {
        request->pipeline_structures[0].latitude  = conf->pipelineStructures[0].centroid.latitude;
        request->pipeline_structures[0].longitude = conf->pipelineStructures[0].centroid.longitude;
    }
    if (conf->pipelineStructures.size() > 1) {
        request->pipeline_structures[1].latitude  = conf->pipelineStructures[1].centroid.latitude;
        request->pipeline_structures[1].longitude = conf->pipelineStructures[1].centroid.longitude;
    }

    // buoys_area (DtcArea)
    
    if (conf->buoysArea.enabled) {
        request->buoys_area.centroid.latitude  = conf->buoysArea.centroid.latitude;
        request->buoys_area.centroid.longitude = conf->buoysArea.centroid.longitude;
        // ricopio la lista size
        //request->buoys_area.clear();
        request->buoys_area.size[0] = conf->buoysArea.size[0];
        request->buoys_area.size[1] = conf->buoysArea.size[1];
        request->buoys_area.orientation = conf->buoysArea.orientation;
    }

    // inizializzo i restanti campi a default
    request->uav_wp.latitude    = 0.0;
    request->uav_wp.longitude   = 0.0;
    request->n_buoys            = 0;
    request->n_damage_markers   = 0;
    request->pipes.clear();
    // request->damaged_pipe rimane con valori di default (0)

    // --- 2) Campi specifici per ciascun TBM ---

    // TBM 1: Inspection
    if (auto insp = std::dynamic_pointer_cast<Inspection>(conf)) {
        // UAV waypoint
        request->uav_wp.latitude  = insp->uavWaypoint.latitude;
        request->uav_wp.longitude = insp->uavWaypoint.longitude;

        // numero di boe
        request->n_buoys = insp->numberOfBuoys;

        // lista di PipelinePipe
        for (const auto &p : insp->pipelinePipes) {
            auv_core_helper::msg::PipelinePipe pipe_msg;
            pipe_msg.number                = p.number;
            pipe_msg.pipeline_structure_id = p.number;      // se diverso, usa p.pipeline_structure_id
            pipe_msg.orientation           = p.angleWithNorth;
            pipe_msg.centroid.latitude     = p.position.latitude;
            pipe_msg.centroid.longitude    = p.position.longitude;
            request->pipes.push_back(pipe_msg);
        }
    }
    // TBM 2: Intervention
    else if (auto inter = std::dynamic_pointer_cast<Intervention>(conf)) {
        // numero di markers
        request->n_damage_markers = inter->numberOfMainPipeDamageMarkers;

        // damaged_pipe
        const auto &d = inter->damagedPipeOnPipeline;
        request->damaged_pipe.number                = d.number;
        request->damaged_pipe.pipeline_structure_id = d.number;  // o d.pipeline_structure_id
        request->damaged_pipe.orientation           = d.angleWithNorth;
        request->damaged_pipe.centroid.latitude     = d.position.latitude;
        request->damaged_pipe.centroid.longitude    = d.position.longitude;
    }
    // TBM 3: Inspection + Intervention
    else if (auto combo = std::dynamic_pointer_cast<InspectionAndIntervention>(conf)) {
        // numero boe e markers
        request->n_buoys          = combo->numberOfBuoys;
        request->n_damage_markers = combo->numberOfMainPipeDamageMarkers;

        // lista di PipelinePipe (solo inspect)
        for (const auto &p : combo->pipelinePipes) {
            auv_core_helper::msg::PipelinePipe pipe_msg;
            pipe_msg.number                = p.number;
            pipe_msg.pipeline_structure_id = p.number;
            pipe_msg.orientation           = p.angleWithNorth;
            pipe_msg.centroid.latitude     = p.position.latitude;
            pipe_msg.centroid.longitude    = p.position.longitude;
            request->pipes.push_back(pipe_msg);
        }
        // NOTA: per TBM3 non impostiamo uav_wp né damaged_pipe
    }
    
        RCLCPP_INFO(this->get_logger(), "Invio MissionCommand con i seguenti dati:");

	// ? tbm_id: corretto
	RCLCPP_INFO(this->get_logger(), "TBM ID: %d", request->tbm_id);

	// ? selected_pipeline_structure_id: corretto
	RCLCPP_INFO(this->get_logger(), "Selected Pipeline Structure ID: %d", request->selected_pipeline_structure_id);

	// ? uav_wp: corretto
	RCLCPP_INFO(this->get_logger(), "UAV WP: [%.6f, %.6f]", request->uav_wp.latitude, request->uav_wp.longitude);

	// ? n_buoys e n_damage_markers: corretto
	RCLCPP_INFO(this->get_logger(), "Num Buoys: %d", request->n_buoys);
	RCLCPP_INFO(this->get_logger(), "Num Damage Markers: %d", request->n_damage_markers);

	// ? pipes[]: correttamente iterato
	RCLCPP_INFO(this->get_logger(), "Pipes:");
	for (const auto& pipe : request->pipes) {
	    RCLCPP_INFO(this->get_logger(), "  - Number: %d, Orientation: %.2f, Position: [%.6f, %.6f]",
		pipe.number, pipe.orientation, pipe.centroid.latitude, pipe.centroid.longitude);
	}
	
	RCLCPP_INFO(this->get_logger(), "Pipeline Structures:");
	for (int i = 0; i < 2; ++i) {
	    RCLCPP_INFO(this->get_logger(), "  [%d] = [%.6f, %.6f]",
		i,
		request->pipeline_structures[i].latitude,
		request->pipeline_structures[i].longitude
	    );
	}

	// ? buoys_area: corretto
	RCLCPP_INFO(this->get_logger(), "Buoys Area:");
	RCLCPP_INFO(this->get_logger(), "  Centroid: [%.6f, %.6f]", request->buoys_area.centroid.latitude, request->buoys_area.centroid.longitude);
	RCLCPP_INFO(this->get_logger(), "  Size: [%.2f, %.2f], Orientation: %.2f",
        request->buoys_area.size[0], request->buoys_area.size[1], request->buoys_area.orientation);
        
        

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

    // Lancia menuLoop in un thread separato
    std::thread menu_thread([&node]() {
        node->menuLoop();
    });

    // Main thread fa solo spin
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // resta qui fino a shutdown()

    menu_thread.join();
    rclcpp::shutdown();
    return 0;
}
