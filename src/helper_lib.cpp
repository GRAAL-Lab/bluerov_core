#include "auv_core_helper/helper_lib.hpp"

namespace fs = std::filesystem;

void LoadParamsFromConf(const std::string& config_name, Eigen::VectorXd* thruster_upper_limits,
                        Eigen::VectorXd* thruster_lower_limits, Eigen::VectorXd* thruster_allocation_weights,
                        Eigen::VectorXd* gainsX, Eigen::VectorXd* gainsY, Eigen::VectorXd* gainsZ,
                        Eigen::VectorXd* gainsRoll, Eigen::VectorXd* gainsPitch, Eigen::VectorXd* gainsYaw,
                        Eigen::VectorXd* max_linear_angular_velocities, Eigen::VectorXd* min_linear_angular_velocities){

    libconfig::Config cfg;
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("auv_core_helper");
        std::string conf_file_path = package_path + "/param/ctrl/" + config_name + ".conf";

        // Check if the configuration file exists
        if (!fs::exists(conf_file_path)) {
            std::cerr << "Configuration file not found: " << conf_file_path << std::endl;
            return;  // Exit the function gracefully
        }

        cfg.readFile(conf_file_path.c_str());

        // helper function to load a vector from the configuration file
        auto loadVector = [](const libconfig::Setting& setting, Eigen::VectorXd& vector) {
            try {
                int length = setting.getLength();
                vector.resize(length);
                for (int i = 0; i < length; ++i) {
                    vector[i] = static_cast<double>(setting[i]);
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load vector: " << nfex.what() << std::endl;
            }
        };

        if (thruster_upper_limits) {
            try {
                const libconfig::Setting& upperLimits = cfg.lookup(config_name + ".thruster_upper_limits");
                loadVector(upperLimits, *thruster_upper_limits);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_upper_limits': " << nfex.what() << std::endl;
            }
        }

        if (thruster_lower_limits) {
            try {
                const libconfig::Setting& lowerLimits = cfg.lookup(config_name + ".thruster_lower_limits");
                loadVector(lowerLimits, *thruster_lower_limits);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_lower_limits': " << nfex.what() << std::endl;
            }
        }

        if (thruster_allocation_weights) {
            try {
                const libconfig::Setting& allocationWeights = cfg.lookup(config_name + ".thruster_allocation_weights");
                loadVector(allocationWeights, *thruster_allocation_weights);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_allocation_weights': " << nfex.what() << std::endl;
            }
        }

        // Load gains
        if (gainsX) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsX)[i] = cfg.lookup(config_name + ".gainsX")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsX': " << nfex.what() << std::endl;
            }
        }

        if (gainsY) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsY)[i] = cfg.lookup(config_name + ".gainsY")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsY': " << nfex.what() << std::endl;
            }
        }

        if (gainsZ) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsZ)[i] = cfg.lookup(config_name + ".gainsZ")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsZ': " << nfex.what() << std::endl;
            }
        }

        if (gainsRoll) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsRoll)[i] = cfg.lookup(config_name + ".gainsRoll")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsRoll': " << nfex.what() << std::endl;
            }
        }

        if (gainsPitch) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsPitch)[i] = cfg.lookup(config_name + ".gainsPitch")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsPitch': " << nfex.what() << std::endl;
            }
        }

        if (gainsYaw) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsYaw)[i] = cfg.lookup(config_name + ".gainsYaw")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsYaw': " << nfex.what() << std::endl;
            }
        }

        if (max_linear_angular_velocities) {
            try {
                const libconfig::Setting& maxVelocities = cfg.lookup(config_name + ".max_linear_angular_velocities");
                loadVector(maxVelocities, *max_linear_angular_velocities);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'max_linear_angular_velocities': " << nfex.what() << std::endl;
            }
        }

        if (min_linear_angular_velocities) {
            try {
                const libconfig::Setting& minVelocities = cfg.lookup(config_name + ".min_linear_angular_velocities");
                loadVector(minVelocities, *min_linear_angular_velocities);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'min_linear_angular_velocities': " << nfex.what() << std::endl;
            }
        }

    } catch (const libconfig::FileIOException &fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
    } catch (const libconfig::ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
    } catch (const libconfig::ConfigException &cex) {
        std::cerr << "Configuration Error: " << cex.what() << std::endl;
    }
}

void LoadBridgeParamsFromConf(const std::string& config_name, bool* simulation_mode, std::string* remote_addr, int* system_id, int* component_id, int* port,
                              bool* camera_enabled, std::string* camera_source_uri, std::string* camera_topic,
                              std::string* camera_info_topic, std::string* camera_frame_id, bool* camera_use_hw_decoder,
                              bool* camera_qos_reliable, int* camera_preview_width, int* camera_preview_height,
                              double* camera_preview_max_fps, int* camera_rtp_latency_ms,
                              std::string* camera_rtp_caps, bool* camera_enable_max_performance)
{
    LoadBridgeParamsFromConf(config_name, simulation_mode, remote_addr, system_id, component_id, port,
                             camera_enabled, camera_source_uri, camera_topic, camera_info_topic, camera_frame_id,
                             camera_use_hw_decoder, camera_qos_reliable, camera_preview_width, camera_preview_height,
                             camera_preview_max_fps, nullptr, nullptr, camera_rtp_latency_ms, camera_rtp_caps,
                             camera_enable_max_performance);
}

void LoadBridgeParamsFromConf(const std::string& config_name, bool* simulation_mode, std::string* remote_addr, int* system_id, int* component_id, int* port,
                              bool* camera_enabled, std::string* camera_source_uri, std::string* camera_topic,
                              std::string* camera_info_topic, std::string* camera_frame_id, bool* camera_use_hw_decoder,
                              bool* camera_qos_reliable, int* camera_preview_width, int* camera_preview_height,
                              double* camera_preview_max_fps, std::string* camera_output_encoding,
                              double* camera_info_publish_rate_hz, int* camera_rtp_latency_ms,
                              std::string* camera_rtp_caps, bool* camera_enable_max_performance)
 {
    libconfig::Config cfg;
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("auv_core_helper");
        std::string conf_file_path = package_path + "/param/ctrl/" + config_name + ".conf";

        if (!std::filesystem::exists(conf_file_path)) {
            std::cerr << "Bridge config file not found: " << conf_file_path << std::endl;
            return;
        }

        cfg.readFile(conf_file_path.c_str());
        const libconfig::Setting& bridge = cfg.lookup(config_name);

        if(simulation_mode)   *simulation_mode  = bridge["simulation_mode"];
        if (remote_addr)      *remote_addr      = (std::string)bridge["remote_addr"];
        if (system_id)        *system_id        = bridge["system_id"];
        if (component_id)     *component_id     = bridge["component_id"];
        if (port)             *port             = bridge["port"];

        bool camera_enabled_value = false;
        if (camera_enabled && bridge.lookupValue("camera_enabled", camera_enabled_value)) {
            *camera_enabled = camera_enabled_value;
        }

        std::string camera_source_uri_value;
        if (camera_source_uri && bridge.lookupValue("camera_source_uri", camera_source_uri_value)) {
            *camera_source_uri = camera_source_uri_value;
        }

        std::string camera_topic_value;
        if (camera_topic && bridge.lookupValue("camera_topic", camera_topic_value)) {
            *camera_topic = camera_topic_value;
        }

        std::string camera_info_topic_value;
        if (camera_info_topic && bridge.lookupValue("camera_info_topic", camera_info_topic_value)) {
            *camera_info_topic = camera_info_topic_value;
        }

        std::string camera_frame_id_value;
        if (camera_frame_id && bridge.lookupValue("camera_frame_id", camera_frame_id_value)) {
            *camera_frame_id = camera_frame_id_value;
        }

        bool camera_use_hw_decoder_value = false;
        if (camera_use_hw_decoder && bridge.lookupValue("camera_use_hw_decoder", camera_use_hw_decoder_value)) {
            *camera_use_hw_decoder = camera_use_hw_decoder_value;
        }

        bool camera_qos_reliable_value = true;
        if (camera_qos_reliable && bridge.lookupValue("camera_qos_reliable", camera_qos_reliable_value)) {
            *camera_qos_reliable = camera_qos_reliable_value;
        }

        int camera_preview_width_value = 0;
        if (camera_preview_width && bridge.lookupValue("camera_preview_width", camera_preview_width_value)) {
            *camera_preview_width = camera_preview_width_value;
        }

        int camera_preview_height_value = 0;
        if (camera_preview_height && bridge.lookupValue("camera_preview_height", camera_preview_height_value)) {
            *camera_preview_height = camera_preview_height_value;
        }

        double camera_preview_max_fps_value = 0.0;
        if (camera_preview_max_fps && bridge.lookupValue("camera_preview_max_fps", camera_preview_max_fps_value)) {
            *camera_preview_max_fps = camera_preview_max_fps_value;
        }

        std::string camera_output_encoding_value;
        if (camera_output_encoding && bridge.lookupValue("camera_output_encoding", camera_output_encoding_value)) {
            *camera_output_encoding = camera_output_encoding_value;
        }

        double camera_info_publish_rate_hz_value = 0.0;
        if (camera_info_publish_rate_hz && bridge.lookupValue("camera_info_publish_rate_hz", camera_info_publish_rate_hz_value)) {
            *camera_info_publish_rate_hz = camera_info_publish_rate_hz_value;
        }

        int camera_rtp_latency_ms_value = 0;
        if (camera_rtp_latency_ms && bridge.lookupValue("camera_rtp_latency_ms", camera_rtp_latency_ms_value)) {
            *camera_rtp_latency_ms = camera_rtp_latency_ms_value;
        }

        std::string camera_rtp_caps_value;
        if (camera_rtp_caps && bridge.lookupValue("camera_rtp_caps", camera_rtp_caps_value)) {
            *camera_rtp_caps = camera_rtp_caps_value;
        }

        bool camera_enable_max_performance_value = false;
        if (camera_enable_max_performance && bridge.lookupValue("camera_enable_max_performance", camera_enable_max_performance_value)) {
            *camera_enable_max_performance = camera_enable_max_performance_value;
        }

    } catch (const libconfig::FileIOException &fioex) {
        std::cerr << "I/O error while reading bridge config file: " << fioex.what() << std::endl;
    } catch (const libconfig::ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
    } catch (const libconfig::SettingNotFoundException &nfex) {
        std::cerr << "Bridge config setting not found: " << nfex.what() << std::endl;
    } catch (const libconfig::ConfigException &cex) {
        std::cerr << "Bridge config error: " << cex.what() << std::endl;
    }
}

void PublishEigenPose(const rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time) {
    auto message = std::make_unique<auv_core_helper::msg::PoseStamped>();
    message->header.stamp = time;
    message->position.latitude = pose(0);
    message->position.longitude = pose(1);
    message->depth = pose(2);
    message->roll = pose(3);
    message->pitch = pose(4);
    message->yaw = pose(5);

    publisher->publish(std::move(message));
}
void PublishEigenVelocity(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& velocity) {
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = velocity(0);
    message->linear.y = velocity(1);
    message->linear.z = velocity(2);
    message->angular.x = velocity(3);
    message->angular.y = velocity(4);
    message->angular.z = velocity(5);
    publisher->publish(std::move(message));
}
void PublishEigenAcceleration(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& acceleration) {   
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = acceleration(0);
    message->linear.y = acceleration(1);
    message->linear.z = acceleration(2);
    message->angular.x = acceleration(3);
    message->angular.y = acceleration(4);
    message->angular.z = acceleration(5);
    publisher->publish(std::move(message));
}

// Function to convert body angular velocities to Euler angle rates
Eigen::Vector3d ConvertAngularVelocitiesToEulerRates(double rollActual, double pitchActual, const Eigen::Vector3d& bOmegaDesired) {
    // Check for singularity (cos(pitchActual) == 0)
    if (std::abs(std::cos(pitchActual)) < 1e-6) {
        std::cerr << "Singularity detected: cos(pitchActual) is too close to zero. Returning zero Euler rates." << std::endl;
        // Return zero Euler rates as a fallback
        return Eigen::Vector3d::Zero();
    }

    // Construct the inverse Jacobian matrix directly
    Eigen::Matrix3d Jinv;
    Jinv << 1, std::sin(rollActual) * std::tan(pitchActual), std::cos(rollActual) * std::tan(pitchActual),
            0, std::cos(rollActual),                       -std::sin(rollActual),
            0, std::sin(rollActual) / std::cos(pitchActual), std::cos(rollActual) / std::cos(pitchActual);

    // Compute the desired Euler angle rates
    Eigen::Vector3d eulerRatesDesired = Jinv * bOmegaDesired;

    return eulerRatesDesired;
}
