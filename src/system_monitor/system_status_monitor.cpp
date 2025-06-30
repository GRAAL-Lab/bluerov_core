#include "system_monitor/system_status_monitor.hpp"

namespace mission
{
    SystemStatusMonitor::SystemStatusMonitor()
        : Node("system_status_monitor_node")
    {
        LoadConfiguration(); // REQUIRES SYSTEM STATUS TO BE INITIALIZED

        lastMissionCtrlTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        lastBridgeTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        lastArdusubTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        lastKCLTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        lastPerceptionTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        rcvMissionCmdTime = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        missionCtrlSub_ = this->create_subscription<auv_core_helper::msg::MissionStatus>(
            auv_core_helper::topicnames::mission_status, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::MissionCtrlCB, this, std::placeholders::_1));

        bridgeHeartBeathSub_ = this->create_subscription<auv_core_helper::msg::HeartBeat>(
            auv_core_helper::topicnames::bridge_heartbeat, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::BridgeCB, this, std::placeholders::_1));

        ardusubHeartBeathSub_ = this->create_subscription<auv_core_helper::msg::HeartBeat>(
            auv_core_helper::topicnames::ardusub_heartbeat, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::ArdusubCB, this, std::placeholders::_1));

        kclSub_ = this->create_subscription<auv_core_helper::msg::KclStatus>(
            auv_core_helper::topicnames::kcl_state, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::KclCB, this, std::placeholders::_1));
        serverKclClient_ = rclcpp_action::create_client<auv_core_helper::action::SetKCL>(
            this, auv_core_helper::topicnames::kcl_setter_action);

        perceptionSub_ = this->create_subscription<std_msgs::msg::Bool>(
            auv_core_helper::topicnames::perception_heartbeat, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::PerceptionCB, this, std::placeholders::_1));

        poseSub_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
            auv_core_helper::topicnames::pose_actual_global_, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::PoseCB, this, std::placeholders::_1));

        safetySwitchSub_ = this->create_subscription<std_msgs::msg::Bool>(
            auv_core_helper::topicnames::safety_switch, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::SafetySwitchCB, this, std::placeholders::_1));

        customSwitchSub_ = this->create_subscription<std_msgs::msg::Bool>(
            auv_core_helper::topicnames::custom_switch, rclcpp::SystemDefaultsQoS(),
            std::bind(&SystemStatusMonitor::CustomSwitchCB, this, std::placeholders::_1));

        int pub_rate = 1; // Default rateù
        std::chrono::milliseconds pub_duration(1000 / pub_rate);
        runTimer_ = this->create_wall_timer(pub_duration, std::bind(&SystemStatusMonitor::StatusPub, this));
        systemStatusPub_ = this->create_publisher<auv_core_helper::msg::SystemStatus>(
            auv_core_helper::topicnames::system_status, rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "System Status Monitor is running.");
    };

    void SystemStatusMonitor::StatusPub()
    {
        auv_core_helper::msg::SystemStatus status;
        status.stamp = this->get_clock()->now();
        status.mission_ctrl = true;
        status.kcl = true;
        status.perception = true;
        status.bridge = true;

        auto timeSinceLastMissionCtrl = this->get_clock()->now() - lastMissionCtrlTime;
        auto timeSinceLastBridge = this->get_clock()->now() - lastBridgeTime;
        auto timeSinceLastArdusub = this->get_clock()->now() - lastArdusubTime;
        auto timeSinceLastKCL = this->get_clock()->now() - lastKCLTime;
        auto timeSinceLastPerception = this->get_clock()->now() - lastPerceptionTime;

        if (timeSinceLastMissionCtrl.seconds() > missionCtrlTimeout_)
        {
            status.mission_ctrl = false;
        }

        if (timeSinceLastBridge.seconds() > bridgeTimeout_ || !rcvFirstPose_ || timeSinceLastArdusub.seconds() > bridgeTimeout_)
        {
            status.bridge = false;
        }

        if (timeSinceLastKCL.seconds() > kclTimeout_)
        {
            status.kcl = false;
        }
        if (!serverKclClient_->wait_for_action_server(std::chrono::seconds(1)))
        {
            status.kcl = false;
        }

        if (timeSinceLastPerception.seconds() > perceptionTimeout_)
        {
            status.perception = false;
        }

        status.vehicle_is_armed = safetySwitchIsOff_;

        if (missionUnderExecution)
        {
            if (vehicleReachedSafetyArea_)
            {
                status.vehicle_is_armed = status.vehicle_is_armed && vehicleIsInSafetyArea_;
            }
            else
            {
                auto timeLeftToReachSafetyArea = this->get_clock()->now() - rcvMissionCmdTime;
                if (timeLeftToReachSafetyArea.seconds() > vehicleMovingToSafetyAreaTimeout_)
                {
                    status.vehicle_is_armed = false;
                    RCLCPP_ERROR(get_logger(), "Vehicle did not reach safety area soon enough.");
                }
            }
            status.vehicle_is_armed = status.vehicle_is_armed && vehicleIsFreeToMove_;
        }

        systemStatusPub_->publish(status);
        std::cout << "[M_Ctrl: " << (status.mission_ctrl ? "On" : "Off")
                  << "] [KCL: " << (status.kcl ? "On" : "Off")
                  << "] [Percept: " << (status.perception ? "On" : "Off")
                  << "] [Bridge: " << (status.bridge ? "On" : "Off")
                  << "] [Vh armed: " << (status.vehicle_is_armed ? "On" : "Off")
                  << "\n      ->(   vehicle looks free to move:  " << (vehicleIsFreeToMove_ ? "Yes" : "No") << ",\n"
                  << "            safety switch is Off:        " << (safetySwitchIsOff_ ? "Yes" : "No") << ",\n"
                  << "            vehicle reached safety area: " << (vehicleReachedSafetyArea_ ? "Yes" : "No") << ",\n"
                  << "            vehicle is in safety area:   " << (vehicleIsInSafetyArea_ ? "Yes" : "No") << ",\n"
                  << "            mission under execution:     " << (missionUnderExecution ? "Yes" : "No") << "\n       )]\n";
    }

    void SystemStatusMonitor::MissionCtrlCB(const auv_core_helper::msg::MissionStatus::SharedPtr msg)
    {
        if (missionCtrlStatus_ == "WAITING FOR CMD" && msg->state != missionCtrlStatus_)
        {
            RCLCPP_INFO(get_logger(), "Mission Control status changed from 'WAITING FOR CMD' to '%s'.", msg->state.c_str());
            missionUnderExecution = true;
            rcvMissionCmdTime = this->get_clock()->now();
        }
        else if (missionCtrlStatus_ != "WAITING FOR CMD" && msg->state == "WAITING FOR CMD")
        {
            RCLCPP_INFO(get_logger(), "Mission Control status changed to 'WAITING FOR CMD'.");
            vehicleReachedSafetyArea_ = false;
            missionUnderExecution = false;
        }

        missionCtrlStatus_ = msg->state;
        lastMissionCtrlTime = this->get_clock()->now();
    }

    void SystemStatusMonitor::BridgeCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg)
    {
        (void)msg; // Unused parameter
        lastBridgeTime = this->get_clock()->now();
    }

    void SystemStatusMonitor::ArdusubCB(const auv_core_helper::msg::HeartBeat::SharedPtr msg)
    {
        (void)msg; // Unused parameter
        lastArdusubTime = this->get_clock()->now();
    }

    void SystemStatusMonitor::KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg)
    {
        (void)msg;
        lastKCLTime = this->get_clock()->now();
    }

    void SystemStatusMonitor::PerceptionCB(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) // Unused parameter
            lastPerceptionTime = this->get_clock()->now();
    }

    void SystemStatusMonitor::SafetySwitchCB(const std_msgs::msg::Bool::SharedPtr msg)
    {
        safetySwitchIsOff_ = msg->data;
    }

    void SystemStatusMonitor::CustomSwitchCB(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Custom switch logic can be added here if needed
        (void)msg; // Unused parameter
    }

    void SystemStatusMonitor::PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
    {
        if (!rcvFirstPose_)
        {
            if (msg->position.latitude != 0.0 && msg->position.longitude != 0.0)
            {
                rcvFirstPose_ = true;
                RCLCPP_INFO(get_logger(), "Received first pose, starting system status monitor.");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Received pose with zero coordinates, waiting for valid pose.");
            }
        }

        if (IsPointWithinBoundaries(ctb::LatLong(msg->position.latitude, msg->position.longitude)))
        {
            if (missionUnderExecution)
                vehicleReachedSafetyArea_ = true;

            vehicleIsInSafetyArea_ = true;
        }
        else
        {
            vehicleIsInSafetyArea_ = false;
        }
    }

    bool SystemStatusMonitor::IsPointWithinBoundaries(const ctb::LatLong &point)
    {
        if (safetyBoundary_.size() < 3)
        {
            throw std::runtime_error("Safety boundary must have at least 3 points.");
        }

        std::vector<Eigen::Vector3d> polygon;
        for (auto &latlong : safetyBoundary_)
        {
            Eigen::Vector3d bodyF_point;
            ctb::LatLong2LocalNED(latlong, 0, point, bodyF_point);
            polygon.push_back(bodyF_point);
        }

        bool pos = false, neg = false;
        int i = 0;
        for (auto point : polygon)
        {
            auto nextPoint = polygon[(i + 1) % polygon.size()];
            double cross = (nextPoint.x() - point.x()) * (-point.y()) - (nextPoint.y() - point.y()) * (-point.x());
            if (cross < 0)
                neg = true;
            if (cross > 0)
                pos = true;
            if (pos && neg)
                return false; // point is outside
            i++;
        }

        return true;
    }

    void SystemStatusMonitor::LoadConfiguration()
    {

        // Load configuration from file
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("auv_core_helper");
        std::string confPath = package_share_directory + "/param/" + "system_monitor.conf";
        libconfig::Config confObj;

        try
        {
            confObj.readFile(confPath.c_str());
            ctb::GetParam(confObj, runRate_, "check_rate");
            ctb::GetParam(confObj, missionCtrlTimeout_, "mission_ctrl_timeout");
            ctb::GetParam(confObj, bridgeTimeout_, "bridge_timeout");
            ctb::GetParam(confObj, kclTimeout_, "kcl_timeout");
            ctb::GetParam(confObj, perceptionTimeout_, "perception_timeout");
            ctb::GetParam(confObj, vehicleMovingToSafetyAreaTimeout_, "vehicle_to_safety_area_timeout");

            const libconfig::Setting &root = confObj.getRoot();
            const libconfig::Setting &safetyBoundarySetting = root["safetyBoundary"];
            std::cerr << "Safety boundary point: " << std::endl;
            for (int i = 0; i < safetyBoundarySetting.getLength(); ++i)
            {
                const libconfig::Setting &point = safetyBoundarySetting[i];
                Eigen::VectorXd latLongTmp;
                ctb::GetParamVector(point, latLongTmp, "point");
                ctb::LatLong stonefishCentroid(44.095952330602564, 9.865115308770484); // from -update pose stonefish- in stonefish utils
                // print local position
                Eigen::Vector3d localTmp3d;
                ctb::LatLong globalPosition(latLongTmp[0], latLongTmp[1]);
                double alt = 0;
                ctb::LatLong2LocalNED(globalPosition, alt, stonefishCentroid, localTmp3d);
                std::cerr << "  - global: [" << latLongTmp[0] << ", " << latLongTmp[1] << "]" << std::endl;
                std::cerr << "  - ned local: [" << localTmp3d[0] << ", " << localTmp3d[1] << "]" << std::endl;
                safetyBoundary_.push_back(ctb::LatLong(latLongTmp[0], latLongTmp[1]));
            }
        }
        catch (const libconfig::FileIOException &fioex)
        {
            RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
            RCLCPP_ERROR(this->get_logger(), "  Path: '%s'. Make sure the file exists and is readable.", confPath.c_str());
        }
        catch (const libconfig::ParseException &pex)
        {
            RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d - %s", pex.getFile(), pex.getLine(), pex.getError());
        }
    }

} // namespace mission