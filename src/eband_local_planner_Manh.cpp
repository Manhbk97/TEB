// Include header files
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <planner_cspace_msgs/PlannerStatus.h>



// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>



// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// #include <obstacle.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <std_msgs/Int16MultiArray.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <queue> 
#include <costmap_2d/obstacle_layer.h>




#include <eband_local_planner/eband_local_planner_ros.h>


// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>




using namespace eband_local_planner;

class TebOptimNode {

enum class NavInd {
  SYS_STATE,
  NAVI_STATE,
  TASK_TYPE,
  REQUEST_STATE,
  REPLAN_COUNT,
  GOAL_STATE,
  DIST_REMAIN,
  DIST_GLOBAL,
  DURATION,
  DOCK_STATE,
  SAFETY_ERROR,
  PLAN_ERROR,
  TRACK_ERROR,
  TASKS_PENDING
};

enum class NavState {
  IDLE,
  PAUSED,
  PLANNING,
  NAVIGATING,
  OBSTACLE_AVOIDING,
  GOAL_POINT,
  DOCKING,
  HALT
};

enum class GoalState {
  NO_GOAL,
  HAVE_GOAL,
  NEAR_GOAL,
  BULLSEYE_GOAL,
  TOLERANT_GOAL,
  NEAR_DOCK_GOAL,
  ALIGN_DOCK_GOAL,
  DOCK_GOAL,
  REACHED_GOAL
};

public:
    TebOptimNode(ros::NodeHandle& nh);

    ~TebOptimNode();

    /**
     * @brief Initialize the node
     */
    void initialize();
    void spin();

private:
    // ROS node handle
    ros::NodeHandle nh_;

    typedef dynamic_reconfigure::Server<
    eband_local_planner::EBandPlannerConfig> drs;
    //! dynamic reconfigure server ptr
    boost::shared_ptr<drs> drs_;

    // pointer to external objects (do NOT delete object)
    costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
    costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
    // base_local_planner::CostmapModel costmap_model_;



    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;    

  // parameters
    double yaw_goal_tolerance_, xy_goal_tolerance_; ///<@brief parameters to define region in which goal is treated as reached
    double rot_stopped_vel_, trans_stopped_vel_; ///<@brief lower bound for absolute value of velocity (with respect to stick-slip behaviour)

    // Topics & Services
    ros::Publisher g_plan_pub_; ///<@brief publishes modified global plan
    ros::Publisher l_plan_pub_; ///<@brief publishes prediction for local commands
    ros::Subscriber odom_sub_; ///<@brief subscribes to the odometry topic in global namespace
    ros::Subscriber global_path_sub_; ///<@brief subscribes to the global path topic 

    // data
    nav_msgs::Odometry base_odom_;
    std::vector<geometry_msgs::PoseStamped> global_plan_; // plan as handed over from move_base or global planner
    std::vector<geometry_msgs::PoseStamped> transformed_plan_; // plan transformed into the map frame we are working in
    std::vector<int> plan_start_end_counter_; // stores which number start and end frame of the transformed plan have inside the global plan

    // pointer to locally created objects (delete - except for smart-ptrs:)
    boost::shared_ptr<EBandPlanner> eband_;
    boost::shared_ptr<EBandVisualization> eband_visual_;
    boost::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;

    bool goal_reached_;

    // flags
    bool initialized_;
    boost::mutex odom_mutex_; // mutex to lock odometry-callback while data is read from topic

    // methods
    void mainCycleCallback();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void reconfigureCallback(EBandPlannerConfig& config, uint32_t level);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
  
};

// Constructor implementation
TebOptimNode::TebOptimNode(ros::NodeHandle& nh) : 
    nh_(nh),
    costmap_ros_(nullptr),
    initialized_(false),
    goal_reached_(false){
        initialize();
}


// Destructor implementation
TebOptimNode::~TebOptimNode() {

}
void TebOptimNode::initialize() {

    // copy adress of costmap and Transform Listener (handed over from move_base)
    // costmap_ros_ = costmap_ros;
    // tf_ = tf;
    tf_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    std::string name = "sirbot1" ;

    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle pn("~/");


    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
    // costmap_ros_->pause();
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    // costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);


    // advertise topics (adapted global plan and predicted local trajectory)
    g_plan_pub_ = pn.advertise<nav_msgs::Path>("track_path", 1);
    l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);


    // subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
    ros::NodeHandle gn;
    odom_sub_ = gn.subscribe<nav_msgs::Odometry>("/sirbot1/tf_odom", 1, boost::bind(&TebOptimNode::odomCallback, this, _1));


    // create the actual planner that we'll use. Pass Name of plugin and pointer to global costmap to it.
    // (configuration is done via parameter server)
    eband_ = boost::shared_ptr<EBandPlanner>(new EBandPlanner(name, costmap_ros_));

    // create the according controller
    eband_trj_ctrl_ = boost::shared_ptr<EBandTrajectoryCtrl>(new EBandTrajectoryCtrl(name, costmap_ros_));

    // create object for visualization
    eband_visual_ = boost::shared_ptr<EBandVisualization>(new EBandVisualization);

    // pass visualization object to elastic band
    eband_->setVisualization(eband_visual_);

    // pass visualization object to controller
    eband_trj_ctrl_->setVisualization(eband_visual_);

    // initialize visualization - set node handle and pointer to costmap
    eband_visual_->initialize(pn, costmap_ros_);

    // create and initialize dynamic reconfigure
    drs_.reset(new drs(pn));
    drs::CallbackType cb = boost::bind(&TebOptimNode::reconfigureCallback, this, _1, _2);
    drs_->setCallback(cb);



    // this is only here to make this process visible in the rxlogger right from the start
    ROS_DEBUG("Elastic Band plugin initialized.");


    // planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));

    // Setup subscribers
    // std::string cmd_vel_topic_out = simulation ? "/sirbot1/key_vel" : "/sirbot1/filtered_key_vel";
    // ROS_INFO_STREAM("cmd_vel_topic_out is: "  << cmd_vel_topic_out);    
    global_path_sub_ = nh_.subscribe("/sirbot1/track_path", 1, &TebOptimNode::globalPathCallback, this);
    // // subgoal_sub_ = nh_.subscribe("/sirbot1/subgoal", 1, &TebOptimNode::subgoalCallback, this);
    // navi_status_sub_ = nh_.subscribe("/sirbot1/navi_status", 1, &TebOptimNode::Callback_navistate, this);


    // Setup publishers
    // cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_out, 1, true);
    // pub_status_ = nh_.advertise<planner_cspace_msgs::PlannerStatus>("sirbot1/planner_3d/status", 1);

    // set initialized flag

    initialized_ = true;
    ROS_INFO("TebOptimNode initialized successfully");


}


void TebOptimNode::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {

// Desired frequency: 5 Hz (0.2 seconds per message)
  ros::Time current_time = ros::Time::now();
  global_plan_ = msg->poses; // Update global_plan_ with new poses

  // Check if enough time has elapsed since the last processed message
  // if ((current_time - last_processed_time_).toSec() >= 5.0) {

  //   global_plan_ = msg->poses; // Update global_plan_ with new poses
 
  //   last_processed_time_ = current_time;
  //   // ROS_INFO("Processed global plan with %zu poses at time %f", global_plan_.size(), current_time.toSec());
  //   // }
  // } 
  setPlan(global_plan_);
}

void TebOptimNode::reconfigureCallback(EBandPlannerConfig& config,
  uint32_t level)
{
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

void TebOptimNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // lock Callback while reading data from topic
  boost::mutex::scoped_lock lock(odom_mutex_);

  // get odometry and write it to member variable (we assume that the odometry is published in the frame of the base)
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
}

    // set global plan to wrapper and pass it to eband
bool TebOptimNode::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{

  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  //reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // transform global plan to the map frame we are working in
  // this also cuts the plan off (reduces it to local window)
  std::vector<int> start_end_counts (2, (int) global_plan_.size()); // counts from the end() of the plan
  if(!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, start_end_counts))
  {
    // if plan could not be tranformed abort control and local planning
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  // also check if there really is a plan
  if(transformed_plan_.empty())
  {
    // if global plan passed in is empty... we won't do anything
    ROS_WARN("Transformed plan is empty. Aborting local planner!");
    return false;
  }

  // set plan - as this is fresh from the global planner robot pose should be identical to start frame
  if(!eband_->setPlan(transformed_plan_))
  {
    // We've had some difficulty where the global planner keeps returning a valid path that runs through an obstacle
    // in the local costmap. See issue #5. Here we clear the local costmap and try one more time.
    costmap_ros_->resetLayers();
    if (!eband_->setPlan(transformed_plan_)) {
      ROS_ERROR("Setting plan to Elastic Band method failed!");
      return false;
    }
  }
  ROS_DEBUG("Global plan set to elastic band for optimization");

  // plan transformed and set to elastic band successfully - set counters to global variable
  plan_start_end_counter_ = start_end_counts;

  // let eband refine the plan before starting continuous operation (to smooth sampling based plans)
  eband_->optimizeBand();


  // display result
  std::vector<eband_local_planner::Bubble> current_band;
  if(eband_->getBand(current_band))
    eband_visual_->publishBand("bubbles", current_band);

  // set goal as not reached
  goal_reached_ = false;

  return true;
}

      
void TebOptimNode::mainCycleCallback() 
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return;
  }
  geometry_msgs::Twist cmd_vel;
  // instantiate local variables
  //std::vector<geometry_msgs::PoseStamped> local_plan;

  geometry_msgs::PoseStamped global_pose;
  std::vector<geometry_msgs::PoseStamped> tmp_plan;

  // get curent robot position
  ROS_DEBUG("Reading current robot Position from costmap and appending it to elastic band.");
  if(!costmap_ros_->getRobotPose(global_pose))
  {
    ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
    return;
  }

  // convert robot pose to frame in plan and set position in band at which to append
  tmp_plan.assign(1, global_pose);
  eband_local_planner::AddAtPosition add_frames_at = add_front;

  // set it to elastic band and let eband connect it
  if(!eband_->addFrames(tmp_plan, add_frames_at))
  {
    ROS_WARN("Could not connect robot pose to existing elastic band.");
    return ;
  }

  // get additional path-frames which are now in moving window
  ROS_DEBUG("Checking for new path frames in moving window");
  std::vector<int> plan_start_end_counter = plan_start_end_counter_;
  std::vector<geometry_msgs::PoseStamped> append_transformed_plan;
  // transform global plan to the map frame we are working in - careful this also cuts the plan off (reduces it to local window)
  if(!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, plan_start_end_counter))
  {
    // if plan could not be tranformed abort control and local planning
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return;
  }

  // also check if there really is a plan
  if(transformed_plan_.empty())
  {
    // if global plan passed in is empty... we won't do anything
    ROS_WARN("Transformed plan is empty. Aborting local planner!");
    return;
  }

  ROS_DEBUG("Retrieved start-end-counts are: (%d, %d)", plan_start_end_counter.at(0), plan_start_end_counter.at(1));
  ROS_DEBUG("Current start-end-counts are: (%d, %d)", plan_start_end_counter_.at(0), plan_start_end_counter_.at(1));

  // identify new frames - if there are any
  append_transformed_plan.clear();
  // did last transformed plan end futher away from end of complete plan than this transformed plan?
  if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(1)) // counting from the back (as start might be pruned)
  {
    // new frames in moving window
    if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(0)) // counting from the back (as start might be pruned)
    {
      // append everything
      append_transformed_plan = transformed_plan_;
    }
    else
    {
      // append only the new portion of the plan
      int discarded_frames = plan_start_end_counter.at(0) - plan_start_end_counter_.at(1);
      ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 >= transformed_plan_.begin());
      ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 < transformed_plan_.end());
      append_transformed_plan.assign(transformed_plan_.begin() + discarded_frames + 1, transformed_plan_.end());
    }

    // set it to elastic band and let eband connect it
    ROS_DEBUG("Adding %d new frames to current band", (int) append_transformed_plan.size());
    if(eband_->addFrames(append_transformed_plan, add_back))
    {
      // appended frames succesfully to global plan - set new start-end counts
      ROS_DEBUG("Sucessfully added frames to band");
      plan_start_end_counter_ = plan_start_end_counter;
    }
        else {
      ROS_WARN("Failed to add frames to existing band");
      return;
    }
  }
  else
    ROS_DEBUG("Nothing to add");

  // update Elastic Band (react on obstacle from costmap, ...)
  ROS_DEBUG("Calling optimization method for elastic band");
  std::vector<eband_local_planner::Bubble> current_band;
  if(!eband_->optimizeBand())
  {
    ROS_WARN("Optimization failed - Band invalid - No controls availlable");
    // display current band
    if(eband_->getBand(current_band))
      eband_visual_->publishBand("bubbles", current_band);
    return ;
  }

  // get current Elastic Band and
  eband_->getBand(current_band);
  // set it to the controller
  if(!eband_trj_ctrl_->setBand(current_band))
  {
    ROS_DEBUG("Failed to to set current band to Trajectory Controller");
    return ;
  }

  // set Odometry to controller
  if(!eband_trj_ctrl_->setOdometry(base_odom_))
  {
    ROS_DEBUG("Failed to to set current odometry to Trajectory Controller");
    return;
  }

  // get resulting commands from the controller
  geometry_msgs::Twist cmd_twist;
  if(!eband_trj_ctrl_->getTwist(cmd_twist, goal_reached_))
  {
    ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
    return;
  }


  // set retrieved commands to reference variable
  ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd_twist.linear.x, cmd_twist.linear.y, cmd_twist.angular.z);
  cmd_vel = cmd_twist;


  // publish plan
  std::vector<geometry_msgs::PoseStamped> refined_plan;
  if(eband_->getPlan(refined_plan))
    // TODO publish local and current gloabl plan
    base_local_planner::publishPlan(refined_plan, g_plan_pub_);
  //base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

  // display current band
  if(eband_->getBand(current_band))
    eband_visual_->publishBand("bubbles", current_band);

  // return;

}



void TebOptimNode::spin(){
  ros::Rate rate(10.0); // Adjust frequency as needed
  while (ros::ok()) {

      // ROS_INFO_STREAM("controller_frequency_: " << controller_frequency_);

      mainCycleCallback();
      ros::spinOnce();
      rate.sleep();

  }
}   

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "rgt_teb_planner");
    ros::NodeHandle nh("~");
    TebOptimNode node(nh);
    node.spin();
    // ros::spin();
    return 0;
}