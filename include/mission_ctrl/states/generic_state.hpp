// #ifndef MISSION_CTRL_GENERIC_STATE_HPP
// #define MISSION_CTRL_GENERIC_STATE_HPP

// #include <fsm/fsm.h>
// #include <libconfig.h++>
// #include "mission_ctrl/ctrl_data_structs.hpp"

// namespace rami {

// namespace states {

//     class GenericState : public fsm::BaseState {
//     protected:
//         // std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;
//         // std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask_;

//         double minHeadingError_, maxHeadingError_;

//     public:
//         // std::shared_ptr<ControlData> ctrlData; // [x y]
//         // std::shared_ptr<tpik::ActionManager> actionManager;
//         std::shared_ptr<rml::RobotModel> robotModel;
//         std::unordered_map<std::string, TasksInfo> tasksMap;

//         GenericState();
//         virtual ~GenericState(void);

//         void CheckRadioController();
//         virtual bool ConfigureStateFromFile(libconfig::Config& confObj) = 0;
//     };
// }
// }

// #endif // MISSION_CTRL_GENERIC_STATE_HPP
