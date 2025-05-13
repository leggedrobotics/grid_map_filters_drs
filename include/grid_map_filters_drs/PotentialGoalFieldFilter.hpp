/*
 * PotentialGoalFieldFilter.hpp
 *
 *  Author: Julia Richter
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>


#include <pluginlib/class_list_macros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/core/core.hpp>

namespace grid_map {

/*!
 * Creates a potential field with a linear gradient towards a goal position.
 */
template <typename T>
class PotentialGoalFieldFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  PotentialGoalFieldFilter();

  /*!
   * Destructor.
   */
  virtual ~PotentialGoalFieldFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Updates the grid map with a potential field.
   * @param mapIn GridMap with the input layers.
   * @param mapOut GridMap with the potential field applied.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  /*!
   * Callback for receiving the path and determining the goal position.
   */
  void pathCallback(const nav_msgs::PathConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);

  /*!
   * Helper to add a cv::Mat as a layer to the grid map.
   */
  void addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap, double resolution = 1.0);

  //! Layer to be processed.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  // ! Output layers
  std::string obstacleLayer_;
  std::string freeSpaceLayer_;
  std::string gradientXLayer_;
  std::string gradientYLayer_;
  std::string gradientZLayer_;

  //! Traversability threshold
  double threshold_;

  //! Topic for receiving the path.
  std::string pathTopic_;

//   //! Goal position.
//   grid_map::Position goalPosition_;

  //! Attractor path
  nav_msgs::Path attractorPath_;
  grid_map::GridMap map_;

  //! Smooth field options
  double fieldSmoothingRadius_;
  bool fieldSmoothing_;

//   //! Flag to normalize output gradients
//   bool normalizeGradients_;

//   //! Attractor position
//   grid_map::Position attractorDirection_;

  //! Attractor stamp
  ros::Time attractorStamp_;

  //! Map frame.
  std::string mapFrame_;

  //! ROS helpers
  //! TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  //! ROS Node handle.
  ros::NodeHandle nodeHandle_;

  //! Path subscriber.
  ros::Subscriber pathSubscriber_;

  //! Marker publisher
  ros::Publisher markerPublisher_;

  //! Timer for updating the path
  ros::Timer timer_;

  //! Profiler for performance measurement.
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map
