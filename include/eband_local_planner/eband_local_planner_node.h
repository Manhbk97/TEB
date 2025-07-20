/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christian Connette
 * Modified for standalone node operation
 *********************************************************************/

#ifndef EBAND_LOCAL_PLANNER_NODE_H_
#define EBAND_LOCAL_PLANNER_NODE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// EBand planner includes
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/eband_trajectory_controller.h>
#include <eband_local_planner/eband_visualization.h>
#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/EBandPlannerConfig.h>

namespace eband_local_planner {

/**
 * @class EBandPlannerNode
 * @brief Standalone ROS node implementation of the Elastic Band local planner
 * 
 * This class converts the original eband_local_planner plugin into a standalone
 * ROS node that:
 * - Subscribes to global_plan topic
 * - Creates and manages its own local costmap
 * - Publishes velocity commands directly
 * - Handles goal reached status
 */
class EBandPlannerNode
{
public:
    /**
     * @brief Default constructor
     */
    EBandPlannerNode();
    
    /**
     * @brief Destructor
     */
    ~EBandPlannerNode();
    
    /**
     * @brief Initialize the node components
     */
    void initialize();
    
    /**
     * @brief Main execution loop
     */
    void run();

private:
    // Callback functions
    
    /**
     * @brief Callback for receiving global plan
     * @param msg Global plan message
     */
    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);
    
    /**
     * @brief Callback for receiving odometry data
     * @param msg Odometry message
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    /**
     * @brief Dynamic reconfigure callback
     * @param config Configuration parameters
     * @param level Reconfiguration level
     */
    void reconfigureCallback(EBandPlannerConfig& config, uint32_t level);
    
    // Main processing functions
    
    /**
     * @brief Process the received global plan
     * @return True if plan processed successfully
     */
    bool processPlan();
    
    /**
     * @brief Compute velocity commands from current elastic band
     * @return True if commands computed successfully
     */
    bool computeVelocityCommands();
    
    /**
     * @brief Check if goal has been reached
     * @return True if goal reached
     */
    bool isGoalReached();
    
    /**
     * @brief Main control loop timer callback
     * @param event Timer event
     */
    void controlLoop(const ros::TimerEvent& event);
    
    // ROS components
    ros::NodeHandle nh_;                ///< Public node handle
    ros::NodeHandle private_nh_;        ///< Private node handle
    
    // Subscribers
    ros::Subscriber global_plan_sub_;   ///< Global plan subscriber
    ros::Subscriber odom_sub_;          ///< Odometry subscriber
    
    // Publishers
    ros::Publisher cmd_vel_pub_;        ///< Velocity command publisher
    ros::Publisher local_plan_pub_;     ///< Local plan publisher
    ros::Publisher global_plan_pub_;    ///< Global plan publisher (for visualization)
    ros::Publisher goal_reached_pub_;   ///< Goal reached status publisher
    
    // TF components
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;              ///< TF buffer
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF listener
    
    // Costmap
    boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;   ///< Local costmap
    
    // EBand components
    boost::shared_ptr<EBandPlanner> eband_;                     ///< EBand planner
    boost::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;     ///< Trajectory controller
    boost::shared_ptr<EBandVisualization> eband_visual_;        ///< Visualization
    
    // Dynamic reconfigure
    typedef dynamic_reconfigure::Server<EBandPlannerConfig> DynamicReconfigureServer;
    boost::shared_ptr<DynamicReconfigureServer> drs_;           ///< Dynamic reconfigure server
    
    // Data storage
    std::vector<geometry_msgs::PoseStamped> global_plan_;       ///< Current global plan
    std::vector<geometry_msgs::PoseStamped> transformed_plan_;  ///< Transformed plan
    std::vector<int> plan_start_end_counter_;                   ///< Plan indexing counters
    nav_msgs::Odometry base_odom_;                              ///< Current odometry
    
    // State variables
    bool initialized_;          ///< Node initialization status
    bool goal_reached_;         ///< Goal reached flag
    bool new_plan_available_;   ///< New plan available flag
    
    // Mutexes for thread safety
    boost::mutex odom_mutex_;   ///< Odometry data mutex
    boost::mutex plan_mutex_;   ///< Plan data mutex
    
    // Parameters
    std::string global_frame_;      ///< Global coordinate frame
    std::string robot_base_frame_;  ///< Robot base coordinate frame
    double control_frequency_;      ///< Control loop frequency
    double xy_goal_tolerance_;      ///< XY goal tolerance
    double yaw_goal_tolerance_;     ///< Yaw goal tolerance
    double rot_stopped_vel_;        ///< Rotational stopped velocity threshold
    double trans_stopped_vel_;      ///< Translational stopped velocity threshold
    
    // Timer for control loop
    ros::Timer control_timer_;      ///< Control loop timer
};

} // namespace eband_local_planner

#endif // EBAND_LOCAL_PLANNER_NODE_H_