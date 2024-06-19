#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/gripper_command.hpp" // Adjust this include based on the actual location
#include "rclcpp_action/rclcpp_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <rclcpp/duration.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"
namespace franka_example_controllers {


static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_GROUP_GRIPPER = "hand";

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class MoveItContoller : public rclcpp_lifecycle::LifecycleNode {
        public:

            MoveItContoller(
            const std::string & node_name,
            const std::string & ns = "",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

            virtual ~MoveItContoller();



            bool moveToGoal(geometry_msgs::msg::Pose goal);
            bool moveGripper(control_msgs::action::GripperCommand::Goal goal);
            
            /**
             * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers
             * @param state Lifecycle Node's state
             * @return Success or Failure
             */
            CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
            /**
             * @brief: Activates LifecyclePublishers and main processor, creates bond connection
             * @param state Lifecycle Node's state
             * @return Success or Failure
             */
            CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
            /**
             * @brief: Deactivates LifecyclePublishers and main processor, destroys bond connection
             * @param state Lifecycle Node's state
             * @return Success or Failure
             */
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
            /**
             * @brief: Resets all subscribers/publishers
             * @param state Lifecycle Node's state
             * @return Success or Failure
             */
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
            /**
             * @brief Called in shutdown state
             * @param state Lifecycle Node's state
             * @return Success or Failure
             */
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;


            /**
             * @brief Goal callback
             * @param msg Shared pointer to poseStamped message
            */
            void goalCallback(geometry_msgs::msg::PoseStamped msg);




        private:
            rclcpp::Logger logger_{rclcpp::get_logger("MoveItContoller")};

            // @brief Whether main routine is active
            bool process_active_;
            
            rclcpp::Node::SharedPtr move_group_node_;
            rclcpp::Node::SharedPtr gripper_client_node_;
            rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client; 
            std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> goal_sub_;

    };

}
