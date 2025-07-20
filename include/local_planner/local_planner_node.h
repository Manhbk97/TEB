#ifndef LOCAL_PLANNER_NODE_H_
#define LOCAL_PLANNER_NODE_H_

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

// planner includes
#include <local_planner/local_planner.h>
#include <local_planner/trajectory_controller.h>
#include <local_planner/visualization.h>
#include <local_planner/conversions_and_types.h>
#include <local_planner/PlannerConfig.h>

namespace local_planner {

/**
 * @class PlannerNode
 * @brief Standalone ROS node implementation of the Band local planner
 * 
 * This class converts the original local_planner plugin into a standalone
 * ROS node that:
 * - Subscribes to global_plan topic
 * - Creates and manages its own local costmap
 * - Publishes velocity commands directly
 * - Handles goal reached status
 */
class PlannerNode
{
public:
    /**
     * @brief Default constructor
     */
    PlannerNode();
    
    /**
     * @brief Destructor
     */
    ~PlannerNode();
    
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

} // namespace local_planner

#endif // LOCAL_PLANNER_NODE_H_