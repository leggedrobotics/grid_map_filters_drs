/*
 * PotentialGoalFieldFilter.cpp
 *
 *  Implements a Potential Goal Field using a path to create a linear gradient towards a corner.
 *
 *  Author: Julia Richter
 */

#include <grid_map_filters_drs/PotentialGoalFieldFilter.hpp>

using namespace filters;

namespace grid_map
{

    template <typename T>
    PotentialGoalFieldFilter<T>::PotentialGoalFieldFilter() : attractorStamp_(0), mapFrame_("not_set") {}

    template <typename T>
    PotentialGoalFieldFilter<T>::~PotentialGoalFieldFilter() {}

    template <typename T>
    bool PotentialGoalFieldFilter<T>::configure()
    {
        // Setup profiler
        profiler_ptr_ = std::make_shared<Profiler>("PotentialGoalFieldFilter");

        // Initialize node handle
        nodeHandle_ = ros::NodeHandle("~potential_goal_field");

        // Load Parameters
        // Input layer to be processed
        if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `input_layer`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] input_layer = %s.", inputLayer_.c_str());

        // Read output_layers_prefix, to define output grid map layers prefix.
        if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `output_layer`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] output_layer = %s.", outputLayer_.c_str());

        // Read threshold, to define the untraversable areas
        if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `threshold`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] threshold = %f.", threshold_);

        // Read path topic
        if (!FilterBase<T>::getParam(std::string("path_topic"), pathTopic_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `path_topic`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] path_topic = %s.", pathTopic_.c_str());

        // Option to smooth the field
        // Read flag to binarize defining traversable and non traversable areas
        if (!filters::FilterBase<T>::getParam(std::string("use_field_smoothing"), fieldSmoothing_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `use_field_smoothing`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] use_field_smoothing = %s.", (fieldSmoothing_ ? "true" : "false"));

        // Smoothing radius
        if (!filters::FilterBase<T>::getParam(std::string("field_smoothing_radius"), fieldSmoothingRadius_))
        {
            ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `field_smoothing_radius`.");
            return false;
        }
        ROS_INFO("[PotentialGoalFieldFilter] field_smoothing_radius = %f.", fieldSmoothingRadius_);

        //   // Read option to normalize gradients
        //   if (!filters::FilterBase<T>::getParam(std::string("normalize_gradients"), normalizeGradients_)) {
        //     ROS_ERROR("[PotentialGoalFieldFilter] Did not find parameter `normalize_gradients`.");
        //     return false;
        //   }
        //   ROS_INFO("[PotentialGoalFieldFilter] normalize_gradients = %s.", (normalizeGradients_ ? "true" : "false"));

        // Initialize TF listener
        tfListener_ = std::make_shared<tf::TransformListener>();

        // Initialize subscriber
        pathSubscriber_ = nodeHandle_.subscribe(std::string(pathTopic_), 1, &PotentialGoalFieldFilter::pathCallback, this);

        // Initialize output layer variables
        obstacleLayer_ = outputLayer_ + "_obstacle_space";
        freeSpaceLayer_ = outputLayer_ + "_free_space";
        gradientXLayer_ = outputLayer_ + "_gradient_x";
        gradientYLayer_ = outputLayer_ + "_gradient_y";
        gradientZLayer_ = outputLayer_ + "_gradient_z";

        // Initialize marker publisher
        markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/field_local_planner/path_heading_direction", 1);
        timer_ = nodeHandle_.createTimer(ros::Duration(1.0), &PotentialGoalFieldFilter::timerCallback, this);

        return true;
    }

    template <typename T>
    void PotentialGoalFieldFilter<T>::pathCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (mapFrame_ == "not_set")
        {
            return;
        }

        // If the timestamps are the same, skip (to avoid TF_REPEATED_DATA issue)
        //   if (msg->header.stamp == attractorStamp_) {
        //     ROS_ERROR("Checkpoint 3");
        //     return;
        //   }
        //   attractorStamp_ = msg->header.stamp;

        attractorPath_ = *msg;
        ROS_ERROR("[PotentialGoalFieldFilter] Received attractor path.");
    }

    template <typename T>
    bool PotentialGoalFieldFilter<T>::update(const T &mapIn, T &mapOut)
    {
        profiler_ptr_->startEvent("0.update");

        // Copy and fix indexing
        map_ = mapIn;
        mapOut = mapIn;
        mapOut.convertToDefaultStartIndex(); // TODO: what is this doing?

        // Check if layer exists.
        if (!mapOut.exists(inputLayer_))
        {
            ROS_ERROR("Check your layers! Layer %s does not exist", inputLayer_.c_str());
            return false;
        }

        // Get resolution
        double resolution = mapOut.getResolution();

        if (mapFrame_ == "not_set")
        {
            // Set standard goal at the center
            grid_map::Position goalPosition = mapOut.getPosition();

            // Create a path containing only the current position
            nav_msgs::Path currentPath;
            currentPath.header.frame_id = mapFrame_;
            currentPath.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped currentPose;
            currentPose.header = currentPath.header;
            currentPose.pose.position.x = goalPosition.x();
            currentPose.pose.position.y = goalPosition.y();
            currentPose.pose.position.z = 0.0;    // Assuming a 2D plane
            currentPose.pose.orientation.w = 1.0; // Neutral orientation

            currentPath.poses.push_back(currentPose);

            // Optionally, store this path if needed
            attractorPath_ = currentPath;
        }

        // Get frame of elevation map
        mapFrame_ = mapOut.getFrameId();

        // Convert selected layer to OpenCV image
        profiler_ptr_->startEvent("1.preprocess");
        cv::Mat cvLayer;
        const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
        const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
        grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, minValue, maxValue, cvLayer);
        cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
        cvLayer.convertTo(cvLayer, CV_32F);
        cvLayer *= 255;

        // Apply threshold and compute obstacle masks
        cv::Mat cvFreeSpaceMask = cvLayer;
        cv::Mat cvObstacleSpaceMask;

        // Binarize input layer
        cv::threshold(cvLayer, cvFreeSpaceMask, 255 * threshold_, 255, cv::THRESH_BINARY);
        cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_8UC1);
        cv::bitwise_not(cvFreeSpaceMask, cvObstacleSpaceMask);
        cvObstacleSpaceMask.convertTo(cvObstacleSpaceMask, CV_32F);
        cvFreeSpaceMask.convertTo(cvFreeSpaceMask, CV_32F);

        // Preallocate output layer
        cv::Mat cvGeodesicDistance(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
        //   cv::Mat maskedGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
        //   cv::Mat maskedGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
        cv::Mat cvGradientsX(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
        cv::Mat cvGradientsY(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));
        cv::Mat cvGradientsZ(cvObstacleSpaceMask.size(), cvObstacleSpaceMask.type(), cv::Scalar(0.0));

        // Add free and occupied space as layers TODO: add for debugging purposes
        //   addMatAsLayer(cvObstacleSpaceMask / 255, obstacleLayer_, mapOut);
        //   addMatAsLayer(cvFreeSpaceMask / 255, freeSpaceLayer_, mapOut);

        // This little hack makes the fast marching method work
        cvObstacleSpaceMask *= 10; // This is to enforce the difference between obstacles and free space
        cvObstacleSpaceMask += 1;  // This adds a baseline layer to start the propagation

        // Smooth field by applying Gaussian Smoothing
        // This is similar to the 'saturation' method used in
        // FM2 by Javier V. Gomez: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6582543
        if (fieldSmoothing_)
        {
            int radiusInPixels = std::max((int)std::ceil(fieldSmoothingRadius_ / mapIn.getResolution()), 3); // Minimum kernel of size 3
            radiusInPixels = (radiusInPixels % 2 == 0) ? radiusInPixels + 1 : radiusInPixels;

            cv::GaussianBlur(cvObstacleSpaceMask, cvObstacleSpaceMask, cv::Size(radiusInPixels, radiusInPixels), 0);
        }
        profiler_ptr_->endEvent("1.preprocess");

        // ###############################################################################
        // ################################ key algorithm ################################
        // ###############################################################################

        // Create a gradient field towards the path
        profiler_ptr_->startEvent("2.potential_calc");
        grid_map::Position currentPosition = mapOut.getPosition();

        Eigen::Vector2d totalVector(0.0, 0.0);

        for (const auto &pose : attractorPath_.poses)
        {
            grid_map::Position pathPosition(pose.pose.position.x, pose.pose.position.y);
            Eigen::Vector2d vectorToPathPoint = pathPosition - currentPosition;
            totalVector += vectorToPathPoint;
        }
        ROS_INFO("Vector: [%f, %f]", totalVector.x(), totalVector.y());

        if (totalVector.norm() > 0) {
            totalVector.normalize();
        }

        // Create a gradient in the direction of the vector
        for (int y = 0; y < cvGeodesicDistance.rows; ++y)
        {
            for (int x = 0; x < cvGeodesicDistance.cols; ++x)
            {
                // slope is calculated from a*x+b*y+c*z=d -> z = (d - a*x - b*y) / c
                double a, b, c, d, z;
                a = totalVector.x();
                b = totalVector.y();
                c = -1;
                d = 0;  // distance of slope, aka size of map
                z = (d - a * x - b * y) / c;

                cvGeodesicDistance.at<float>(x, y) = static_cast<float>(z);
            }
        }

        double minVal, maxVal;
        cv::minMaxLoc(cvGeodesicDistance, &minVal, &maxVal);

        // Normalize the cvGeodesicDistance to ensure values are within a specific range
        double scale = 100.0;
        cv::normalize(cvGeodesicDistance, cvGeodesicDistance, 0, scale, cv::NORM_MINMAX);

        // ###############################################################################
        // ################################ key algorithm ################################
        // ###############################################################################
        profiler_ptr_->endEvent("2.potential_calc");

        profiler_ptr_->startEvent("3.gradients");
        // Blur before computing gradients to smooth the image
        cv::GaussianBlur(cvGeodesicDistance, cvGeodesicDistance, cv::Size(5, 5), 0);

        // Compute gradients for path planning
        cv::Sobel(cvGeodesicDistance, cvGradientsX, -1, 0, 1, 3, resolution);
        cv::Sobel(cvGeodesicDistance, cvGradientsY, -1, 1, 0, 3, resolution);

        profiler_ptr_->endEvent("3.gradients");

        // Add layers
        profiler_ptr_->startEvent("4.gdf_add_layers");
        addMatAsLayer(cvGeodesicDistance, outputLayer_, mapOut, resolution);
        addMatAsLayer(cvGradientsX, gradientXLayer_, mapOut);
        addMatAsLayer(cvGradientsY, gradientYLayer_, mapOut);
        addMatAsLayer(cvGradientsZ, gradientZLayer_, mapOut);
        profiler_ptr_->endEvent("4.gdf_add_layers");

        mapOut.setBasicLayers({});

        // Publish the vector as a marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = mapFrame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_heading";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1; // Shaft diameter
        marker.scale.y = 0.2; // Head diameter
        marker.scale.z = 0.2; // Head length
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0; // Red
        marker.color.g = 0.0; // Green
        marker.color.b = 0.0; // Blue

        geometry_msgs::Point start, end;
        start.x = currentPosition.x();
        start.y = currentPosition.y();
        start.z = 0.0;
        end.x = start.x + totalVector.x();
        end.y = start.y + totalVector.y();
        end.z = 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);

        markerPublisher_.publish(marker);

        profiler_ptr_->endEvent("0.update");
        return true;
    }

    template <typename T>
    void PotentialGoalFieldFilter<T>::addMatAsLayer(const cv::Mat &mat, const std::string &layerName, grid_map::GridMap &gridMap, double resolution)
    {
        double minDistance, maxDistance;
        cv::minMaxLoc(mat, &minDistance, &maxDistance);

        minDistance *= resolution;
        maxDistance *= resolution;

        cv::Mat normalized;
        cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
        normalized.convertTo(normalized, CV_32F);

        grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName, gridMap, minDistance, maxDistance);
    }

    template <typename T>
    void PotentialGoalFieldFilter<T>::timerCallback(const ros::TimerEvent &)
    {
        if (attractorPath_.poses.empty() || mapFrame_ == "not_set")
        {
            return;
        }

        // Get the current position of the robot
        grid_map::Position currentPosition = map_.getPosition();

        // Find the closest point on the path
        auto closestIt = std::min_element(attractorPath_.poses.begin(), attractorPath_.poses.end(),
                                          [&currentPosition](const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
                                          {
                                              grid_map::Position posA(a.pose.position.x, a.pose.position.y);
                                              grid_map::Position posB(b.pose.position.x, b.pose.position.y);
                                              return (posA - currentPosition).norm() < (posB - currentPosition).norm();
                                          });

        // Erase all points up to and including the closest point if the distance is under 2 meters
        if (closestIt != attractorPath_.poses.end())
        {
            grid_map::Position closestPos(closestIt->pose.position.x, closestIt->pose.position.y);
            if ((closestPos - currentPosition).norm() < 2.0)
            {
                attractorPath_.poses.erase(attractorPath_.poses.begin(), closestIt + 1);
            }
        }
    }

} // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::PotentialGoalFieldFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::PotentialGoalFieldFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)

// rostopic pub /global_planning/path nav_msgs/Path "header:
//     seq: 0
//     stamp: {secs: 0, nsecs: 0}
//     frame_id: 'map'
// poses:
// - header:
//     seq: 0
//     stamp: {secs: 0, nsecs: 0}
//     frame_id: 'map'
//   pose:
//     position: {x: 1.0, y: 2.0, z: 0.0}
//     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
// - header:
//     seq: 0
//     stamp: {secs: 0, nsecs: 0}
//     frame_id: 'map'
//   pose:
//     position: {x: 3.0, y: 4.0, z: 0.0}
//     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"