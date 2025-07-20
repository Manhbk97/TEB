

#ifndef LOCAL_PLANNER_ROS_H_
#define LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// classes wich are parts of this pkg
#include <local_planner/local_planner.h>
#include <local_planner/conversions_and_types.h>
#include <local_planner/visualization.h>
#include <local_planner/trajectory_controller.h>
#include <local_planner/PlannerConfig.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// transforms
#include <angles/angles.h>
#include <tf2_ros/buffer.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


namespace local_planner{

  /**
   * @class PlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */
  class PlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      PlannerROS();
    
    /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
    PlannerROS(std::string name, tf2_ros::Buffer* tf,
		    costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~PlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
    void initialize(std::string name, tf2_ros::Buffer* tf,
		    costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:

      /**
       * @brief Reconfigures node parameters
       * @param config The dynamic reconfigure node configuration
       * @param level Reconfiguration level
       */
      void reconfigureCallback(PlannerConfig& config, uint32_t level);

      typedef dynamic_reconfigure::Server<
        local_planner::PlannerConfig> drs;
      //! dynamic reconfigure server ptr
      boost::shared_ptr<drs> drs_;

      // pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
      tf2_ros::Buffer* tf_; ///<@brief pointer to Transform Listener

    // parameters
      double yaw_goal_tolerance_, xy_goal_tolerance_; ///<@brief parameters to define region in which goal is treated as reached
      double rot_stopped_vel_, trans_stopped_vel_; ///<@brief lower bound for absolute value of velocity (with respect to stick-slip behaviour)

      // Topics & Services
      ros::Publisher g_plan_pub_; ///<@brief publishes modified global plan
      ros::Publisher l_plan_pub_; ///<@brief publishes prediction for local commands
      ros::Subscriber odom_sub_; ///<@brief subscribes to the odometry topic in global namespace

      // data
      nav_msgs::Odometry base_odom_;
      std::vector<geometry_msgs::PoseStamped> global_plan_; // plan as handed over from move_base or global planner
      std::vector<geometry_msgs::PoseStamped> transformed_plan_; // plan transformed into the map frame we are working in
      std::vector<int> plan_start_end_counter_; // stores which number start and end frame of the transformed plan have inside the global plan

      // pointer to locally created objects (delete - except for smart-ptrs:)
      boost::shared_ptr<Planner> eband_;
      boost::shared_ptr<Visualization> eband_visual_;
      boost::shared_ptr<TrajectoryCtrl> eband_trj_ctrl_;

      bool goal_reached_;

      // flags
      bool initialized_;
      boost::mutex odom_mutex_; // mutex to lock odometry-callback while data is read from topic


      // methods

      /**
       * @brief Odometry-Callback: function is called whenever a new odometry msg is published on that topic 
       * @param Pointer to the received message
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  };
};
#endif


