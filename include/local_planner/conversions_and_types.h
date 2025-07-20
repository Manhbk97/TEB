

#ifndef CONVERSIONS_AND_TYPES_H_
#define CONVERSIONS_AND_TYPES_H_

#include <ros/ros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#include <geometry_msgs/TransformStamped.h>

// transforms
#include <angles/angles.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace local_planner{


  // typedefs

  ///<@brief defines a bubble - pose of center & radius of according hypersphere (expansion)
  struct Bubble
  {
    geometry_msgs::PoseStamped center;
    double expansion;
  };

  enum AddAtPosition {add_front, add_back};

  // functions

  // pose - Quaternions,pose2D - euler angles
  /**
   * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from quaternions to euler angles)
   * @param Pose which shall be converted
   * @param References to converted ROS Pose2D frmae
   */
  void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D);


  /**
   * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from euler angles to quaternions, -> z-coordinate is set to zero)
   * @param References to converted ROS Pose2D frame
   * @param Pose2D which shall be converted
   */
  void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D);


  /**
   * @brief  Transforms the global plan of the robot from the planner frame to the local frame. This replaces the transformGlobalPlan as defined in the base_local_planner/goal_functions.h main difference is that it additionally outputs counter indicating which part of the plan has been transformed.
   * @param tf A reference to a transform listener
   * @param global_plan The plan to be transformed
   * @param costmap A reference to the costmap being used so the window size for transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param transformed_plan Populated with the transformed plan
   * @param number of start and end frame counted from the end of the global plan
   */
  bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
			   costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts_from_end);

  /**
   * @brief Gets the footprint of the robot and computes the circumscribed radius for the eband approach
   * @param costmap A reference to the costmap from which the radius is computed
   * @return radius in meters
   */
  double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap);

};
#endif

