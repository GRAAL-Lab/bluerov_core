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




void PublishEigenPose(const rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time) {
    auto message = std::make_unique<auv_core_helper::msg::PoseStamped>();
    message->header.stamp = time;
    message->x = pose(0);
    message->y = pose(1);
    message->z = pose(2);
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