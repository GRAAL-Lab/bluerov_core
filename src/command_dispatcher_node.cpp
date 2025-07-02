// command_dispatcher_node.cpp
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "auv_core_helper/srv/mission_command.hpp"
#include <libconfig.h++>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mission_ctrl/mission_data_structs.hpp"
#include "std_msgs/msg/empty.hpp"

using MissionCommand = auv_core_helper::srv::MissionCommand;
using namespace mission;

class CommandDispatcherNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dispatch_sub_;
  int tbm_id_{0};
  double wpLatLong[2] = {0.0, 0.0};
  bool tbm_done = false;
  bool lat_done = false;
  bool long_done = false;

  std::ofstream log_file_;
  std::mutex log_mutex_;

  CommandDispatcherNode()
      : Node("command_dispatcher")
  {
    tbm_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/tbm_id", 10,
        std::bind(&CommandDispatcherNode::TbmIdCallback, this, std::placeholders::_1));

    latitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/latitude", 10,
        std::bind(&CommandDispatcherNode::LatitudeCallback, this, std::placeholders::_1));

    longitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/longitude", 10,
        std::bind(&CommandDispatcherNode::LongitudeCallback, this, std::placeholders::_1));

    dispatch_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/dispatch_request", 10,
        std::bind(&CommandDispatcherNode::DispatchCallback, this, std::placeholders::_1));

    client_ = this->create_client<MissionCommand>("/auv/service/mission_cmd");
    
    std::string log_path = std::string(std::getenv("HOME")) + "/robot_ctrlstation_communication.log";
    log_file_.open(log_path, std::ios_base::app);
    if (!log_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
    }
  }

  ~CommandDispatcherNode()
  {
    if (log_file_.is_open())
    {
      log_file_.close();
    }
  }

private:
  std::shared_ptr<TaskBenchmarkSettings> conf_;
  rclcpp::Client<MissionCommand>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tbm_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr latitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr longitude_sub_;

  void TbmIdCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    tbm_id_ = msg->data;

    if (tbm_id_)
    {
      RCLCPP_INFO(this->get_logger(), "Received new TBM ID: %d", tbm_id_);
      tbm_done = true;
    }

    else
      RCLCPP_INFO(this->get_logger(), "Not valid tbm_id received");
  }

  void LatitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    /*if (auto insp = std::dynamic_pointer_cast<Inspection>(conf_)) {
      insp->uavWaypoint.latitude = msg->data;
      RCLCPP_INFO(this->get_logger(), "[/latitude] %f", insp->uavWaypoint.latitude);
    }*/
    wpLatLong[0] = msg->data;

    if (wpLatLong[0])
    {
      RCLCPP_INFO(this->get_logger(), "[/latitude] %f", wpLatLong[0]);
      lat_done = true;
    }

    else
      RCLCPP_INFO(this->get_logger(), "Not valid latitude received");
  }

  void LongitudeCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    /*if (auto insp = std::dynamic_pointer_cast<Inspection>(conf_)) {
      insp->uavWaypoint.longitude = msg->data;
      RCLCPP_INFO(this->get_logger(), "[/longitude] %f", insp->uavWaypoint.longitude);
    }*/

    wpLatLong[1] = msg->data;

    if (wpLatLong[1])
    {
      RCLCPP_INFO(this->get_logger(), "[/longitude] %f", wpLatLong[1]);
      long_done = true;
    }

    else
      RCLCPP_INFO(this->get_logger(), "Not valid longitude received");
  }

  void DispatchCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    RCLCPP_INFO(this->get_logger(), "Trigger ricevuto: invio MissionCommand");

    if (!LoadConfiguration(conf_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load initial configuration");
      return;
    }

    if (tbm_done && conf_)
    {
      //if (conf_ && lat_done && long_done)
      //{
        auto request = std::make_shared<MissionCommand::Request>();
        BuildMissionRequest(request);

        std::ostringstream req_log;
        req_log << "Sending MissionCommand Request: TBM ID: " << request->tbm_id
                << ", UAV WP: (" << request->uav_wp.latitude << ", " << request->uav_wp.longitude << ")";
        logToFile(req_log.str());

        if (!client_->wait_for_service(std::chrono::seconds(5)))
        {
          RCLCPP_ERROR(this->get_logger(), "Service non disponibile");
          return;
        }

        // corretta firma del callback:
        client_->async_send_request(
            request,
            [this](rclcpp::Client<MissionCommand>::SharedFuture future)
            {
              if (future.valid())
              {
                RCLCPP_INFO(this->get_logger(),
                            "Service response: res=%s, text='%s'",
                            future.get()->res ? "true" : "false",
                            future.get()->text.c_str());
                std::ostringstream resp_log;
                resp_log << "Received MissionCommand Response: res=" << (future.get()->res ? "true" : "false")
                         << ", text='" << future.get()->text.c_str() << "'";
                logToFile(resp_log.str());
              }
              else
              {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
              }
            });
        tbm_done = false;
        lat_done = false;
        long_done = false;
      //}
    }
    else
      RCLCPP_ERROR(this->get_logger(), "tbm_id not selected");
  }

  bool LoadConfiguration(std::shared_ptr<TaskBenchmarkSettings> &conf)
  {
    std::string pkg = ament_index_cpp::get_package_share_directory("ctrl_station");
    std::string path = pkg + "/conf/tasks.conf";
    libconfig::Config cfg;
    try
    {
      cfg.readFile(path.c_str());
      uint cfg_tbm;
      if (!ctb::GetParam(cfg, cfg_tbm, "tbm") || cfg_tbm != static_cast<uint>(tbm_id_))
      {
        cfg_tbm = static_cast<uint>(tbm_id_);
      }
      switch (cfg_tbm)
      {
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
        RCLCPP_ERROR(this->get_logger(), "Invalid TBM id %u", cfg_tbm);
        return false;
      }
      return conf->ConfigureFromFile(cfg);
    }
    catch (const libconfig::FileIOException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "I/O error reading config: %s", e.what());
      return false;
    }
    catch (const libconfig::ParseException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d: %s", e.getFile(), e.getLine(), e.getError());
      return false;
    }
  }

  void BuildMissionRequest(std::shared_ptr<MissionCommand::Request> request)
  {
    if (!conf_)
    {
      RCLCPP_ERROR(this->get_logger(), "Configuration pointer is null.");
      return;
    }
    // Common fields
    request->tbm_id = tbm_id_;
    request->selected_pipeline_structure_id = conf_->selectedPipelineStructureId;
    // Pipeline structures (fixed array size 2)
    if (conf_->pipelineStructures.size() > 0)
    {
      request->pipeline_structures[0].latitude = conf_->pipelineStructures[0].centroid.latitude;
      request->pipeline_structures[0].longitude = conf_->pipelineStructures[0].centroid.longitude;
    }
    if (conf_->pipelineStructures.size() > 1)
    {
      request->pipeline_structures[1].latitude = conf_->pipelineStructures[1].centroid.latitude;
      request->pipeline_structures[1].longitude = conf_->pipelineStructures[1].centroid.longitude;
    }
    // Buoys area
    if (conf_->buoysArea.points.size() >= 4)
    {
      for (size_t i = 0; i < 4; ++i)
      {
        request->buoys_area_points[i].latitude = conf_->buoysArea.points[i].latitude;
        request->buoys_area_points[i].longitude = conf_->buoysArea.points[i].longitude;
      }
    }
    // Defaults
    // request->uav_wp.latitude    = 0.0;
    // request->uav_wp.longitude   = 0.0;
    request->n_buoys = 0;
    request->n_damage_markers = 0;
    request->pipes.clear();
    // TBM-specific
    if (auto insp = std::dynamic_pointer_cast<Inspection>(conf_))
    {
      request->uav_wp.latitude = wpLatLong[0];
      request->uav_wp.longitude = wpLatLong[1];
      request->n_buoys = insp->numberOfBuoys;
      for (const auto &p : insp->pipelinePipes)
      {
        auv_core_helper::msg::PipelinePipe pipe_msg;
        pipe_msg.number = p.number;
        pipe_msg.pipeline_structure_id = p.number;
        pipe_msg.orientation = p.orientation;
        pipe_msg.centroid.latitude = p.position.latlong.latitude;
        pipe_msg.centroid.longitude = p.position.latlong.longitude;
        request->pipes.push_back(pipe_msg);
      }
    }
    else if (auto inter = std::dynamic_pointer_cast<Intervention>(conf_))
    {
      request->n_damage_markers = inter->numberOfMainPipeDamageMarkers;
      const auto &d = inter->damagedPipeOnPipeline;
      request->damaged_pipe.number = d.number;
      request->damaged_pipe.pipeline_structure_id = d.number;
      request->damaged_pipe.orientation = d.orientation;
      request->damaged_pipe.centroid.latitude = d.position.latlong.latitude;
      request->damaged_pipe.centroid.longitude = d.position.latlong.longitude;
    }
    else if (auto combo = std::dynamic_pointer_cast<InspectionAndIntervention>(conf_))
    {
      request->n_buoys = combo->numberOfBuoys;
      request->n_damage_markers = combo->numberOfMainPipeDamageMarkers;
      for (const auto &p : combo->pipelinePipes)
      {
        auv_core_helper::msg::PipelinePipe pipe_msg;
        pipe_msg.number = p.number;
        pipe_msg.pipeline_structure_id = p.number;
        pipe_msg.orientation = p.orientation;
        pipe_msg.centroid.latitude = p.position.latlong.latitude;
        pipe_msg.centroid.longitude = p.position.latlong.longitude;
        request->pipes.push_back(pipe_msg);
      }
    }
  }

  void logToFile(const std::string &message)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    log_file_ << "[" << std::put_time(std::localtime(&now), "%F %T") << "] " << message << std::endl;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandDispatcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
