/*
 * GeodesicDistanceField2dFilter.hpp
 *
 *  Author: Matias Mattamala
 */

#pragma once

#include <filters/filter_base.hpp>
#include <grid_map_filters_drs/utils/profiler.hpp>
#include <string>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_filters_drs/thirdparty/GeodesicDistanceTransform.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace grid_map {

/*!
 * Computes a signed distance field by applying a threshold on a layer
 */
template <typename T>
class GeodesicDistanceField2dFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  GeodesicDistanceField2dFilter();

  /*!
   * Destructor.
   */
  virtual ~GeodesicDistanceField2dFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Uses either an upper or lower threshold. If the threshold is exceeded
   * the cell value is set to the predefined value setTo_.
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 protected:
  /*!
   * Finds a valid attractor in the current grid map
   */
  virtual grid_map::Index getAttractorIndex(const T& gridMap, const grid_map::Position& attractorPosition);

  /*!
   * Helper to fill layers with cv::Mats
   */
  void addMatAsLayer(const cv::Mat& m, const std::string& layerName, grid_map::GridMap& gridMap, double resolution = 1.0);

  /*!
   * Subscribers the attractor from a pose message
   */
  void attractorCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  //! Layer the threshold will be evaluated.
  std::string inputLayer_;

  //! Output layer prefix
  std::string outputLayer_;

  // ! Output layers
  std::string obstacleLayer_;
  std::string freeSpaceLayer_;
  std::string gradientXLayer_;
  std::string gradientYLayer_;
  std::string gradientZLayer_;

  //! Traversability threshold
  double threshold_;

  //! Smooth field options
  double fieldSmoothingRadius_;
  bool fieldSmoothing_;

  //! Attractor position
  grid_map::Position attractorPosition_;

  //! Attractor topic
  std::string attractorTopic_;

  //! Target frame
  std::string attractorFrame_;
  //! Map frame
  std::string mapFrame_;
  //! Goal id
  ros::Time attractorStamp_;

  //! Flag to normalize output gradients
  bool normalizeGradients_;

  //! ROS helpers
  //! TF listener
  std::shared_ptr<tf::TransformListener> tfListener_;

  //! Node handle
  ros::NodeHandle nodeHandle_;

  //! Attractor subscriber
  ros::Subscriber attractorSubscriber_;
  ros::Subscriber attractorSubscriberRviz_;
  ros::Subscriber attractorSubscriberRviz2_;

  //! Profiler
  std::shared_ptr<Profiler> profiler_ptr_;
};

}  // namespace grid_map