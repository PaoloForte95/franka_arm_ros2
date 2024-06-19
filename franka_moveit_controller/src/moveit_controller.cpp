#include "franka_moveit_controller/moveit_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace franka_example_controllers {


MoveItContoller::MoveItContoller(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options), process_active_(false)
{

    RCLCPP_INFO(
    get_logger(),
    "\n\t%s lifecycle node launched. \n"
    "\tWaiting on external lifecycle transitions to activate\n"
    , get_name());

    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

 

}

MoveItContoller::~MoveItContoller()
{
      if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    this->deactivate();
  }

  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    this->cleanup();
  }
}


CallbackReturn
MoveItContoller::on_configure(const rclcpp_lifecycle::State & /*state*/)

{
    auto node = shared_from_this();

    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "Configuring MoveIt Contoller");

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("panda_arm_goal", rclcpp::SensorDataQoS(), std::bind(&MoveItContoller::goalCallback, this, std::placeholders::_1));
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ = rclcpp::Node::make_shared("test_trajectory", node_options);
    gripper_client_node_ = rclcpp::Node::make_shared("gripper_client_node", node_options);

    action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(gripper_client_node_,"/panda_gripper/gripper_action");
    RCLCPP_INFO(logger_, "Configured MoveIt Contoller");
  return CallbackReturn::SUCCESS;
}


CallbackReturn
MoveItContoller::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating MoveIt Contoller");

  // Activating main worker
  process_active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn
MoveItContoller::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating MoveIt Contoller");
  // Deactivating main worker
  process_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn
MoveItContoller::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up MoveIt Contoller");
  
  //Pubs
  //Subs
  goal_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
MoveItContoller::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return CallbackReturn::SUCCESS;
}


bool MoveItContoller::moveToGoal(geometry_msgs::msg::Pose goal){
    
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node_, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node_, PLANNING_GROUP_GRIPPER);
    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    RCLCPP_INFO(logger_, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger_, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    RCLCPP_INFO(logger_, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    
    auto current_state = move_group.getCurrentState(10);
    
    std::vector<double> joint_group_positions;
    std::vector<double> joint_group_positions_gripper;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_gripper);


    RCLCPP_INFO(logger_, "Moving to position Position %f,%f,%f with orientation %f,%f,%f, %f", 
    goal.position.x,goal.position.y, goal.position.z,
    goal.orientation.x,goal.orientation.y, goal.orientation.z, goal.orientation.w);
    move_group.setPlannerId("RRTstarkConfigDefault");

    move_group.setPoseTarget(goal, "panda_hand");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger_, "Planning successful, executing plan...");
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger_, "Planning failed!");
    }

    }
 
    bool MoveItContoller::moveGripper(control_msgs::action::GripperCommand::Goal goal){

        
        auto move_group_gripper = moveit::planning_interface::MoveGroupInterface(move_group_node_, PLANNING_GROUP_GRIPPER);
        auto current_state_gripper = move_group_gripper.getCurrentState(10);
        RCLCPP_INFO(logger_, "Moving Gripper!");

        auto is_action_server_ready = action_client->wait_for_action_server(std::chrono::seconds(5));
        if(!is_action_server_ready){
            RCLCPP_ERROR(logger_, "moving gripper action server not ready!");
        }
        goal.command.max_effort = 30;

        auto future_goal_handle = action_client->async_send_goal(goal);
        if(rclcpp::spin_until_future_complete(gripper_client_node_,future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(logger_, "Failed send goal");
        }

        auto send_close_handler = future_goal_handle.get();
        auto future_result = action_client->async_get_result(send_close_handler);

    }

    void MoveItContoller::goalCallback(geometry_msgs::msg::PoseStamped msg){
        RCLCPP_INFO(logger_, "Received goal! Position:(%f,%f,%f), Orientation:(%f,%f,%f,%f)",
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
        moveToGoal(msg.pose);
    }

}