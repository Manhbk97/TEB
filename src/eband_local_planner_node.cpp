
// Include header files
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

// eband includes
#include <eband_local_planner/eband_local_planner_ros.h>
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/eband_trajectory_controller.h>
#include <eband_local_planner/eband_visualization.h>
#include <eband_local_planner/conversions_and_types.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <eband_local_planner/EBandPlannerConfig.h>

// base local planner utilities
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/costmap_model.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

using namespace eband_local_planner;

class EBandPlannerNode {
public:
    EBandPlannerNode(ros::NodeHandle& nh);
    ~EBandPlannerNode();

    /**
     * @brief Initialize the node
     */
    void initialize();
    void spin();

private:
    // ROS node handle
    ros::NodeHandle nh_;

    typedef dynamic_reconfigure::Server<eband_local_planner::EBandPlannerConfig> drs;
    boost::shared_ptr<drs> drs_;

    // TF and costmap
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    // costmap_2d::Costmap2D* costmap_;

    // Parameters
    double yaw_goal_tolerance_, xy_goal_tolerance_;
    double rot_stopped_vel_, trans_stopped_vel_;
    double controller_frequency_;

    // Topics & Services
    ros::Publisher g_plan_pub_;
    ros::Publisher l_plan_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber global_path_sub_;

    // Data
    nav_msgs::Odometry base_odom_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> transformed_plan_;
    std::vector<int> plan_start_end_counter_;

    // EBand components
    boost::shared_ptr<EBandPlanner> eband_;
    boost::shared_ptr<EBandVisualization> eband_visual_;
    boost::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;

    // Flags
    bool goal_reached_;
    bool initialized_;
    bool has_global_plan_;
    boost::mutex odom_mutex_;

    // Timer for main control loop
    ros::Timer control_timer_;

    // Methods
    void controlTimerCallback(const ros::TimerEvent& event);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void reconfigureCallback(EBandPlannerConfig& config, uint32_t level);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
};

EBandPlannerNode::EBandPlannerNode(ros::NodeHandle& nh) : 
    nh_(nh),
    costmap_ros_(nullptr),
    // costmap_(nullptr),
    initialized_(false),
    goal_reached_(false),
    has_global_plan_(false),
    controller_frequency_(10.0)
{
    initialize();
}

EBandPlannerNode::~EBandPlannerNode() {
    if (costmap_ros_) {
        delete costmap_ros_;
    }
}

void EBandPlannerNode::initialize() {
    // Initialize TF
    tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.0));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    // Node parameters
    // std::string robot_name;
    // nh_.param<std::string>("robot_name", robot_name, "sirbot1");
    nh_.param<double>("controller_frequency", controller_frequency_, 10.0);
    nh_.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.05);
    nh_.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    nh_.param<double>("rot_stopped_vel", rot_stopped_vel_, 0.01);
    nh_.param<double>("trans_stopped_vel", trans_stopped_vel_, 0.01);

    // Create local costmap
    ROS_INFO("Creating local costmap...");
    costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
    // costmap_ros_->pause();
    
    // Start costmap after a brief delay to ensure proper initialization
    // ros::Duration(1.0).sleep();
    costmap_ros_->start();
    
    // costmap_ = costmap_ros_->getCostmap();

    // Setup publishers
    ros::NodeHandle pn("~");
    g_plan_pub_ = pn.advertise<nav_msgs::Path>("/global_plan", 1);
    l_plan_pub_ = pn.advertise<nav_msgs::Path>("/local_plan", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Setup subscribers
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&EBandPlannerNode::odomCallback, this, _1));
    global_path_sub_ = nh_.subscribe<nav_msgs::Path>("/global_path", 1, boost::bind(&EBandPlannerNode::globalPathCallback, this, _1));

    // Create EBand components
    std::string planner_name = "eband_planner";
    eband_ = boost::shared_ptr<EBandPlanner>(new EBandPlanner(planner_name, costmap_ros_));
    eband_trj_ctrl_ = boost::shared_ptr<EBandTrajectoryCtrl>(new EBandTrajectoryCtrl(planner_name, costmap_ros_));
    eband_visual_ = boost::shared_ptr<EBandVisualization>(new EBandVisualization);

    // costmap_ros_->start();

    // Connect components
    eband_->setVisualization(eband_visual_);
    eband_trj_ctrl_->setVisualization(eband_visual_);
    eband_visual_->initialize(pn, costmap_ros_);

    // Setup dynamic reconfigure
    drs_.reset(new drs(pn));
    drs::CallbackType cb = boost::bind(&EBandPlannerNode::reconfigureCallback, this, _1, _2);
    drs_->setCallback(cb);

    // Setup control timer
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_frequency_), 
                                     &EBandPlannerNode::controlTimerCallback, this);

    initialized_ = true;
    ROS_INFO("EBand Local Planner Node initialized successfully");
}

void EBandPlannerNode::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!initialized_) return;
    
    global_plan_ = msg->poses;
    // ROS_INFO("Received global plan with %zu poses", global_plan_.size());
    has_global_plan_ = !global_plan_.empty();
    // Add check for empty plan
    if (global_plan_.empty()) {
        ROS_WARN("Received empty global plan");
        // return;
    }
    if (has_global_plan_) {
        setPlan(global_plan_);
        ROS_DEBUG("Received global plan with %zu poses", global_plan_.size());
    }
}

void EBandPlannerNode::reconfigureCallback(EBandPlannerConfig& config, uint32_t level) {
    xy_goal_tolerance_ = config.xy_goal_tolerance;
    yaw_goal_tolerance_ = config.yaw_goal_tolerance;
    rot_stopped_vel_ = config.rot_stopped_vel;
    trans_stopped_vel_ = config.trans_stopped_vel;

    if (eband_)
        eband_->reconfigure(config);
    else
        ROS_ERROR("Reconfigure CB called before eband planner initialization");

    if (eband_trj_ctrl_)
        eband_trj_ctrl_->reconfigure(config);
    else
        ROS_ERROR("Reconfigure CB called before trajectory controller initialization!");

    if (eband_visual_)
        eband_visual_->reconfigure(config);
    else
        ROS_ERROR("Reconfigure CB called before eband visualizer initialization");
}

void EBandPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
}

bool EBandPlannerNode::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // Reset the global plan
    // global_plan_.clear();
    global_plan_ = orig_global_plan;

    if (orig_global_plan.empty()) {
        ROS_ERROR("Received empty global plan in setPlan");
        return false;
    }
    // Transform global plan to the map frame
    std::vector<int> start_end_counts(2, (int) global_plan_.size());
    if (!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, 
                                                   costmap_ros_->getGlobalFrameID(), 
                                                   transformed_plan_, start_end_counts)) {
        ROS_WARN("Plan frame: %s, Target frame: %s", 
                 global_plan_[0].header.frame_id.c_str(),
                 costmap_ros_->getGlobalFrameID().c_str());
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
    }

    if (transformed_plan_.empty()) {
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
    }

    // Set plan to elastic band
    if (!eband_->setPlan(transformed_plan_)) {
        costmap_ros_->resetLayers();
        if (!eband_->setPlan(transformed_plan_)) {
            ROS_ERROR("Setting plan to Elastic Band method failed!");
            return false;
        }
    }

    plan_start_end_counter_ = start_end_counts;
    eband_->optimizeBand();

    // Visualize initial band
    std::vector<eband_local_planner::Bubble> current_band;
    if (eband_->getBand(current_band))
        eband_visual_->publishBand("bubbles", current_band);

    goal_reached_ = false;
    return true;
}

void EBandPlannerNode::controlTimerCallback(const ros::TimerEvent& event) {
    if (!initialized_ || !has_global_plan_) {
        return;
    }


    geometry_msgs::Twist cmd_vel;
    if (computeVelocityCommands(cmd_vel)) {
        cmd_vel_pub_.publish(cmd_vel);
    } else {
        // Publish zero velocity if planning fails
        geometry_msgs::Twist zero_vel;
        cmd_vel_pub_.publish(zero_vel);
    }
}

bool EBandPlannerNode::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    geometry_msgs::PoseStamped global_pose;
    std::vector<geometry_msgs::PoseStamped> tmp_plan;

    // Get current robot position
    if (!costmap_ros_->getRobotPose(global_pose)) {
        ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
        return false;
    }

    // Add current robot pose to elastic band
    tmp_plan.assign(1, global_pose);
    if (!eband_->addFrames(tmp_plan, add_front)) {
        ROS_WARN("Could not connect robot pose to existing elastic band.");
        return false;
    }

    // Check for new path frames in moving window
    std::vector<int> plan_start_end_counter = plan_start_end_counter_;
    std::vector<geometry_msgs::PoseStamped> append_transformed_plan;
    
    if (!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, 
                                                   costmap_ros_->getGlobalFrameID(), 
                                                   transformed_plan_, plan_start_end_counter)) {
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
    }

    if (transformed_plan_.empty()) {
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
    }

    // Add new frames if available
    append_transformed_plan.clear();
    if (plan_start_end_counter_.at(1) > plan_start_end_counter.at(1)) {
        if (plan_start_end_counter_.at(1) > plan_start_end_counter.at(0)) {
            append_transformed_plan = transformed_plan_;
        } else {
            int discarded_frames = plan_start_end_counter.at(0) - plan_start_end_counter_.at(1);
            if (transformed_plan_.begin() + discarded_frames + 1 < transformed_plan_.end()) {
                append_transformed_plan.assign(transformed_plan_.begin() + discarded_frames + 1, 
                                             transformed_plan_.end());
            }
        }

        if (!append_transformed_plan.empty() && eband_->addFrames(append_transformed_plan, add_back)) {
            plan_start_end_counter_ = plan_start_end_counter;
        }
    }

    // Optimize elastic band
    std::vector<eband_local_planner::Bubble> current_band;
    if (!eband_->optimizeBand()) {
        ROS_WARN("Optimization failed - Band invalid - No controls available");
        if (eband_->getBand(current_band))
            eband_visual_->publishBand("bubbles", current_band);
        return false;
    }

    // Get optimized band and set to controller
    eband_->getBand(current_band);
    if (!eband_trj_ctrl_->setBand(current_band)) {
        ROS_DEBUG("Failed to set current band to Trajectory Controller");
        return false;
    }

    // Set odometry to controller
    if (!eband_trj_ctrl_->setOdometry(base_odom_)) {
        ROS_DEBUG("Failed to set current odometry to Trajectory Controller");
        return false;
    }

    // Get velocity commands
    geometry_msgs::Twist cmd_twist;
    if (!eband_trj_ctrl_->getTwist(cmd_twist, goal_reached_)) {
        ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
        return false;
    }

    cmd_vel = cmd_twist;

    // Publish plans and visualization
    std::vector<geometry_msgs::PoseStamped> refined_plan;
    if (eband_->getPlan(refined_plan)) {
        base_local_planner::publishPlan(refined_plan, g_plan_pub_);
    }

    if (eband_->getBand(current_band)) {
        eband_visual_->publishBand("bubbles", current_band);
    }

    return true;
}

bool EBandPlannerNode::isGoalReached() {
    return goal_reached_;
}

void EBandPlannerNode::spin() {
    ros::spin();
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "eband_local_planner_node");
    ros::NodeHandle nh("~");
    
    try {
        EBandPlannerNode node(nh);
        node.spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in EBand planner node: %s", e.what());
        return 1;
    }
    
    return 0;
}