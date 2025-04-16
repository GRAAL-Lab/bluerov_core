#include "kcl/kinematic_control_layer.hpp"
using namespace std::chrono_literals;


KCL::KCL()
    : Node("kcl_fsm_node") {
    // Declare and retrieve the "config_name" parameter
    this->declare_parameter<std::string>("config_name", "default_value");  // Default value if not set
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
    poseActualSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_actual, 1,
        std::bind(&KCL::PoseActualCallback, this, std::placeholders::_1));

    velocityActualSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_actual, 1,
        std::bind(&KCL::VelocityActualCallback, this, std::placeholders::_1));

    accelerationActualSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::acceleration_actual, 1,
        std::bind(&KCL::AccelerationActualCallback, this, std::placeholders::_1));

    // Create publishers
    poseGoalPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_goal, 1);

    velocityDesiredPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_desired, 1);

    statePublisher_ = this->create_publisher<std_msgs::msg::String>(
        auv_core_helper::topicnames::kcl_state, 1);

    pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);

    // Create service for control commands
    controlCommandService_ = this->create_service<auv_core_helper::srv::ControlCommand>(
        auv_core_helper::topicnames::control_cmd_service,
        std::bind(&KCL::HandleControlCommand, this, std::placeholders::_1, std::placeholders::_2));
}

void KCL::PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    // Update actual pose in control data
    ctrlData_->poseActual << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    ctrlData_->timeActual = msg->header.stamp;
}

void KCL::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual velocity in control data
    ctrlData_->velocityActual << msg->linear.x, msg->linear.y, msg->linear.z,
                                   msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::AccelerationActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual acceleration in control data
    ctrlData_->accelerationActual << msg->linear.x, msg->linear.y, msg->linear.z,
                                       msg->angular.x, msg->angular.y, msg->angular.z;
}
void KCL::HandleControlCommand(
    const std::shared_ptr<auv_core_helper::srv::ControlCommand::Request> request,
    std::shared_ptr<auv_core_helper::srv::ControlCommand::Response> response) {
        //TO DO!
        //print the request
        RCLCPP_INFO(this->get_logger(), "Received request to transition to state: %s", request->state.c_str());
}

void KCL::SetupTransitions() {
    // Create states
    idleState_ = std::make_unique<IdleState>(&fsm_);
    holdState_ = std::make_unique<HoldState>(&fsm_);
    wayPointNavigationState_ = std::make_unique<WayPointNavigationState>(&fsm_);
    surfaceState_ = std::make_unique<SurfaceState>(&fsm_);
    pathFollowingState_ = std::make_unique<PathFollowingState>(&fsm_);

    // Share control data with states
    idleState_->ctrlData = ctrlData_;
    holdState_->ctrlData = ctrlData_;
    wayPointNavigationState_->ctrlData = ctrlData_;
    surfaceState_->ctrlData = ctrlData_;
    pathFollowingState_->ctrlData = ctrlData_;

    // Add states and enable transitions
    fsm_.AddState(States::IDLE, idleState_.get());
    fsm_.AddState(States::HOLD, holdState_.get());
    fsm_.AddState(States::WAYPOINT_NAVIGATION, wayPointNavigationState_.get());
    fsm_.AddState(States::SURFACE, surfaceState_.get());
    fsm_.AddState(States::PATH_FOLLOWING, pathFollowingState_.get());


    //TO DO: edit the transitions
    // Enable transitions
    fsm_.EnableTransition(States::IDLE, States::HOLD, true);
    fsm_.EnableTransition(States::IDLE, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::IDLE, States::SURFACE, true);
    fsm_.EnableTransition(States::IDLE, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::HOLD, States::IDLE, true);
    fsm_.EnableTransition(States::HOLD, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::HOLD, States::SURFACE, true);
    fsm_.EnableTransition(States::HOLD, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::IDLE, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::HOLD, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::SURFACE, true);
    fsm_.EnableTransition(States::WAYPOINT_NAVIGATION, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::SURFACE, States::IDLE, true);
    fsm_.EnableTransition(States::SURFACE, States::HOLD, true);
    fsm_.EnableTransition(States::SURFACE, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::SURFACE, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::WAYPOINT_NAVIGATION, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::SURFACE, true);
    fsm_.SetInitState(States::HOLD);

    RCLCPP_INFO(this->get_logger(), "FSM transitions set up.");
}

void KCL::ExecuteFSM() {
    // Execute the current FSM state
    // std::string previous_state = fsm_.GetCurrentStateName();
    fsm_.SwitchState();
    fsm_.ExecuteState();

    // Publish current state
    std_msgs::msg::String stateMsg;
    stateMsg.data = fsm_.GetCurrentStateName();
    statePublisher_->publish(stateMsg);

    // Publish goal pose
    PublishEigenPose(poseGoalPublisher_, ctrlData_->poseGoal, this->get_clock()->now());

    // Scale desired velocity within limits
    rml::SaturateVector(ctrlData_->maxVelocity, ctrlData_->minVelocity, ctrlData_->velocityDesired);

    // Publish desired velocity
    PublishEigenVelocity(velocityDesiredPublisher_, ctrlData_->velocityDesired);

    // Publish the planned path
    pathPublisher_->publish(ctrlData_->plannedPath);
}
