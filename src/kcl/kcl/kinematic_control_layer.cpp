#include "kcl/kinematic_control_layer.hpp"
using namespace std::chrono_literals;


KCL::KCL()
    : Node("kcl_fsm_node") {
    // Declare and retrieve the "config_name" parameter
    this->declare_parameter<std::string>("config_name", "BlueROV");  // Default value if not set
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Initialize control data
    ctrlData_ = std::make_shared<auv::ControlData>();

    // Load configuration parameters into control data
    LoadParamsFromConf(
        configNameParam, nullptr, nullptr, nullptr,
        &ctrlData_->gainsX, &ctrlData_->gainsY, &ctrlData_->gainsZ,
        &ctrlData_->gainsRoll, &ctrlData_->gainsPitch, &ctrlData_->gainsYaw,
        &ctrlData_->maxVelocity, &ctrlData_->minVelocity);

    // Set up state transitions
    SetupTransitions();

    // Create FSM timer
     fsmTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(ctrlData_->dt * 1000)),
        std::bind(&KCL::ExecuteFSM, this));

    // Create subscriptions
    poseActualGlobalSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual_global_, 1,std::bind(&KCL::PoseActualGlobalCallback, this, std::placeholders::_1));

    // Create publishers
    statePublisher_ = this->create_publisher<auv_core_helper::msg::KclStatus>(auv_core_helper::topicnames::kcl_state, 1);
    poseGoalGlobalPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_desired_global, 1);
    velocityDesiredGlobalPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired_global, 1);
    pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);
    deisiredCtrlModePublisher_ = this->create_publisher<std_msgs::msg::String>(auv_core_helper::topicnames::desired_ctrl_mode, 1);

    // Create action server for KCL
    KCLSetter_ = rclcpp_action::create_server<auv_core_helper::action::SetKCL>(
    this,
    auv_core_helper::topicnames::kcl_setter_action,
    std::bind(&KCL::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&KCL::HandleCancel, this, std::placeholders::_1),
    std::bind(&KCL::HandleSetKCL, this, std::placeholders::_1)
    );

    // Create clients
    armingClient_ = this->create_client<std_srvs::srv::SetBool>(auv_core_helper::topicnames::arming_service);
    flightModeClient_ = this->create_client<auv_core_helper::srv::SetFlightMode>(auv_core_helper::topicnames::flight_mode_service);


}

void KCL::PoseActualGlobalCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    // Update actual pose in control data
    ctrlData_->poseActualGlobal << msg->position.latitude, msg->position.longitude, msg->depth, msg->roll, msg->pitch, msg->yaw;
    ctrlData_->timeActual = msg->header.stamp;
    //print
    // RCLCPP_INFO(this->get_logger(), "Pose Actual: %f, %f, %f, %f, %f, %f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
}

void KCL::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual velocity in control data
    ctrlData_->velocityActual << msg->linear.x, msg->linear.y, msg->linear.z,
                                   msg->angular.x, msg->angular.y, msg->angular.z;
}



rclcpp_action::GoalResponse KCL::HandleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const auv_core_helper::action::SetKCL::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with state: %s", goal->desired_state.c_str());
    std::lock_guard<std::mutex> lock(goalMutex_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KCL::HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<auv_core_helper::action::SetKCL>> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void KCL::HandleSetKCL(const std::shared_ptr<rclcpp_action::ServerGoalHandle<auv_core_helper::action::SetKCL>> goal_handle)
{

    {   // remember the goal
        std::lock_guard<std::mutex> lock(goalMutex_);
        if (activeGoal_ && activeGoal_->is_active()) {
            auto res          = std::make_shared<SetKCL::Result>();
            res->success      = false;
            res->message      = "Pre-empted by a newer goal";
            activeGoal_->abort(res);
            fsm_.SetNextState(States::HOLD);
            fsm_.SwitchState();

        }
        activeGoal_ = goal_handle;
    }

    const auto goal = goal_handle->get_goal();   

    // Store in member variables
    desiredState_ = goal->desired_state;
    ctrlData_->poseGoalGlobal(0) = goal->position.latitude;
    ctrlData_->poseGoalGlobal(1) = goal->position.longitude;
    ctrlData_->poseGoalGlobal(2) = -std::abs(goal->depth);
    ctrlData_->pathPlanningMode  = goal->path_mode;
    ctrlData_->resumePath     = goal->resume_path;

    // Sprial data
    ctrlData_->spiralDiameter    = goal->spiral_data.spiral_diameter;
    ctrlData_->spiralIncrement   = goal->spiral_data.spiral_increment;


    // Serpentine data
    ctrlData_->pathArea(0, 0) = goal->serpentine_data.origin.latitude;
    ctrlData_->pathArea(1, 0) = goal->serpentine_data.origin.longitude;

    ctrlData_->pathArea(0, 1) = goal->serpentine_data.front_left.latitude;
    ctrlData_->pathArea(1, 1) = goal->serpentine_data.front_left.longitude;

    ctrlData_->pathArea(0, 2) = goal->serpentine_data.front_right.latitude;
    ctrlData_->pathArea(1, 2) = goal->serpentine_data.front_right.longitude;

    ctrlData_->pathArea(0, 3) = goal->serpentine_data.right.latitude;
    ctrlData_->pathArea(1, 3) = goal->serpentine_data.right.longitude;

    // Circular data
    ctrlData_->circularDiameter = goal->circular_data.circular_diameter;
    ctrlData_->circularCenterLL.latitude = goal->circular_data.center_point.latitude;
    ctrlData_->circularCenterLL.longitude = goal->circular_data.center_point.longitude;
    ctrlData_->circularClockwise = goal->circular_data.clockwise;


    Eigen::Vector3d tmpCircularCenterLocal;
    ctb::LatLong2LocalNED(ctrlData_->circularCenterLL, -std::abs(1.0), ctrlData_->homeLL, tmpCircularCenterLocal);
    ctrlData_->circularCenterLocal = tmpCircularCenterLocal - ctrlData_->homeLocal.head<3>();




    ctrlData_->actionProgress = 0.0;
    ctrlData_->actionSuccess  = false;
    ctrlData_->actionFailed   = false;
    ctrlData_->actionMessage.clear();
    
    if (!(fsm_.SetNextState(desiredState_) == fsm::ok && fsm_.SwitchState() == fsm::ok)){
        //faild to switch state, can decalre failure
        ctrlData_->actionSuccess  = false;
        ctrlData_->actionFailed   = true;
        ctrlData_->actionMessage = "Failed to set state: " + desiredState_;
    }

}

void KCL::UpdateActionState()
{
    std::lock_guard<std::mutex> lock(goalMutex_);
    if (!activeGoal_) { return; }                       // nothing to do

    /* ---------- cancellation request ---------- */
    if (activeGoal_->is_canceling()) {
        auto res = std::make_shared<SetKCL::Result>();
        res->success = true;
        res->message = "Canceled by client";
        activeGoal_->canceled(res);
        activeGoal_.reset();
        return;
    }

    /* ---------- publish feedback --------------- */
    auto fb = std::make_shared<SetKCL::Feedback>();
    fb->actual_state      = fsm_.GetCurrentStateName();
    fb->action_progress = ctrlData_->actionProgress;
    activeGoal_->publish_feedback(fb);

    /* ---------- decide if we are done ---------- */
    if (ctrlData_->actionSuccess) {
        auto res = std::make_shared<SetKCL::Result>();
        res->success = true;
        res->message = ctrlData_->actionMessage;
        activeGoal_->succeed(res);
        activeGoal_.reset();
        ctrlData_->actionSuccess = false;
        return;
    }

    if (ctrlData_->actionFailed) {
        auto res = std::make_shared<SetKCL::Result>();
        res->success = false;
        res->message = ctrlData_->actionMessage;
        activeGoal_->abort(res);
        activeGoal_.reset();
        ctrlData_->actionFailed = false;
        return;
    }
}

void KCL::SetupTransitions() {
    // Create states
    idleState_ = std::make_unique<IdleState>(&fsm_);
    holdState_ = std::make_unique<HoldState>(&fsm_);
    lockDvlState_ = std::make_unique<LockDvlState>(&fsm_);
    wayPointNavigationState_ = std::make_unique<WayPointNavigationState>(&fsm_);
    pathFollowingState_ = std::make_unique<PathFollowingState>(&fsm_);

    

    // Share control data with states
    idleState_->ctrlData = ctrlData_;
    lockDvlState_->ctrlData = ctrlData_;
    holdState_->ctrlData = ctrlData_;
    wayPointNavigationState_->ctrlData = ctrlData_;
    pathFollowingState_->ctrlData = ctrlData_;

    // Add states and enable transitions
    fsm_.AddState(States::IDLE, idleState_.get());
    fsm_.AddState(States::LOCK_DVL, lockDvlState_.get());
    fsm_.AddState(States::HOLD, holdState_.get());
    fsm_.AddState(States::WAYPOINT_NAVIGATION, wayPointNavigationState_.get());
    fsm_.AddState(States::PATH_FOLLOWING, pathFollowingState_.get());


    // Enable transitions
    fsm_.EnableTransition(States::IDLE, States::HOLD, true);
    fsm_.EnableTransition(States::IDLE, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::IDLE, States::PATH_FOLLOWING, true);

    fsm_.EnableTransition(States::HOLD, States::IDLE, true);
    fsm_.EnableTransition(States::HOLD, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::HOLD, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::HOLD, States::LOCK_DVL, true);

    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::IDLE, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::HOLD, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::LOCK_DVL, true);


    fsm_.EnableTransition(States::PATH_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::LOCK_DVL, true);


    fsm_.SetInitState(States::IDLE);

    RCLCPP_INFO(this->get_logger(), "FSM transitions set up.");
}


void KCL::CallArmingService(bool arm)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = arm;

    auto future = armingClient_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Arming succeeded: %s", result.get()->message.c_str());
                ctrlData_->armed_actual = ctrlData_->armed_desired;
            } else {
                RCLCPP_WARN(this->get_logger(), "Arming failed: %s", result.get()->message.c_str());
                ctrlData_->actionFailed   = true;
                ctrlData_->actionMessage = "Unable to arm after";
            }
        });
}

void KCL::CallFlightModeService(const std::string &mode)
{
    auto request = std::make_shared<auv_core_helper::srv::SetFlightMode::Request>();
    request->mode = mode;
    //print
    RCLCPP_INFO(this->get_logger(), "Flight mode requested: %s", request->mode.c_str());

    auto future = flightModeClient_->async_send_request(request,
        [this](rclcpp::Client<auv_core_helper::srv::SetFlightMode>::SharedFuture result) {
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Flight mode set: %s", result.get()->message.c_str());
                ctrlData_->flightMode_actual = ctrlData_->flightMode_desired;
            } else {
                RCLCPP_WARN(this->get_logger(), "Flight mode failed: %s", result.get()->message.c_str());
                // TO DO, if it fails to go to gudided or poshold go to state find dvl lock
                CallFlightModeService(ctrlData_->flightMode_desired);
            }
        });
}

void KCL::ExecuteFSM() {
    // Temporary variables to hold local NED (North-East-Down) coordinates
    Eigen::Vector3d tmpHomeLocal;
    Eigen::Vector3d tmpPoseLocal;

    ctrlData_->poseActualLL = ctb::LatLong(ctrlData_->poseActualGlobal(0), ctrlData_->poseActualGlobal(1));
    ctb::LatLong2LocalNED(ctrlData_->homeLL, -std::abs(ctrlData_->homeGlobal(2)), ctrlData_->homeLL, tmpHomeLocal);
    ctb::LatLong2LocalNED(ctrlData_->poseActualLL, -std::abs(ctrlData_->poseActualGlobal(2)), ctrlData_->homeLL, tmpPoseLocal);




    ctrlData_->homeLocal.head<3>() = tmpHomeLocal;
    ctrlData_->poseActualLocal.head<3>() = tmpPoseLocal - tmpHomeLocal;
    ctrlData_->poseActualLocal.tail<3>() = ctrlData_->poseActualGlobal.tail<3>();
    
    //publish desiredctrlmode
    deisiredCtrlModePublisher_->publish(std_msgs::msg::String().set__data(ctrlData_->deisiredCtrlMode));


    // Execute the current FSM state
    // std::string previous_state = fsm_.GetCurrentStateName();
    fsm_.SwitchState();
    fsm_.ExecuteState();

    // Publish current state
    auv_core_helper::msg::KclStatus stateMsg;
    stateMsg.stamp = this->get_clock()->now();
    statePublisher_->publish(stateMsg);
    //stateMsg.state = fsm_.GetCurrentStateName();
    //statePublisher_->publish(stateMsg);

    if (ctrlData_->deisiredCtrlMode == auv_core_helper::BrigdeMode::PoseCtrl) {
        PublishEigenPose(poseGoalGlobalPublisher_, ctrlData_->poseGoalGlobal, this->get_clock()->now());
    }
    else if (ctrlData_->deisiredCtrlMode == auv_core_helper::BrigdeMode::VelCtrl) {
        PublishEigenPose(poseGoalGlobalPublisher_, ctrlData_->poseGoalGlobal, this->get_clock()->now());
        PublishEigenVelocity(velocityDesiredGlobalPublisher_, ctrlData_->velocityDesiredNED);
    }

    // Scale desired velocity within limits
    // rml::SaturateVector(ctrlData_->maxVelocity, ctrlData_->minVelocity, ctrlData_->velocityDesired);


    // Publish the planned path
    // pathPublisher_->publish(ctrlData_->plannedPath);


    //do clinet call if desired != actual
    if (ctrlData_->armed_desired != ctrlData_->armed_actual) {
        CallArmingService(ctrlData_->armed_desired);
    }
    if (ctrlData_->flightMode_desired != ctrlData_->flightMode_actual) {
        CallFlightModeService(ctrlData_->flightMode_desired);
    }
    UpdateActionState();
}
