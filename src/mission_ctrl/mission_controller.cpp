#include "mission_ctrl/mission_controller.hpp"

namespace rami {
MissionController::MissionController(std::string conf_filename)
    : Node("mission_control_node")
{

    fileName_ = conf_filename;
    if (!LoadConfiguration(conf_)) {
        std::cerr << "Failed to load configuration from file" << std::endl;
        return;
    }
    std::cerr << *conf_ << std::endl;

};

bool MissionController::LoadConfiguration(std::shared_ptr<TaskBenchMarkSettings>& conf)
{
    libconfig::Config confObj;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("mission_ctrl");
    std::string confPath = package_share_directory + "/conf/" + fileName_;

    try {
        confObj.readFile(confPath.c_str());
        uint tbm_id;
        if (!ctb::GetParam(confObj, tbm_id, "tbm"))
            return false;

        std::cerr << "tbm_id: " << tbm_id << std::endl;
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
                std::cerr << "Invalid tbm id: " << tbm_id << std::endl;
                return false;
        }
        return conf->ConfigureFromFile(confObj);
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        std::cerr << "  Path: '" << confPath << "'. Make sure the file exists and is readable." << std::endl;
        return false;   
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return false;   
    }
}

}
