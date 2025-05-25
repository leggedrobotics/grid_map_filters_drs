/*
 * SignedDistanceField2dFilter.cpp
 *
 *  Implements a 2D SDF layer using a binary layer. 0 means obstacles, 1 free space
 *
 *  Author: Matias Mattamala
 */

#include <time.h>
#include <grid_map_filters_drs/SignedDistanceField2dFilter.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
SignedDistanceField2dFilter<T>::SignedDistanceField2dFilter() {}

template <typename T>
SignedDistanceField2dFilter<T>::~SignedDistanceField2dFilter() {}

template <typename T>
bool SignedDistanceField2dFilter<T>::configure() {
  // Setup profiler
  profiler_ptr_ = std::make_shared<Profiler>("SignedDistanceField2dFilter");

  // Load Parameters
  // Input layer to be processed
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter output_layer = %s.", inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter output_layer = %s.", outputLayer_.c_str());

  // Read threshold, to define the untraversable areas
  if (!filters::FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `threshold`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter threshold = %f.", threshold_);

  // Read option to normalize gradients
  if (!filters::FilterBase<T>::getParam(std::string("normalize_gradients"), normalizeGradients_)) {
    ROS_ERROR("SDF 2D filter did not find parameter `normalize_gradients`.");
    return false;
  }
  ROS_DEBUG("SDF 2D filter normalize_gradients = %s.", (normalizeGradients_ ? "true" : "false"));

  return true;
}

template <typename T>
bool SignedDistanceField2dFilter<T>::update(const T& mapIn, T& mapOut) {
  // auto tic = std::chrono::high_resolution_clock::now();
  profiler_ptr_->startEvent("0.update");
  // clock_t start_t = clock();

  // Copy and fix indexing
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  // Check if layer exists.
  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("Check your threshold types! Type %s does not exist", inputLayer_.c_str());
    return false;
  }

  // Get resolution
  double resolution = mapOut.getResolution();

  profiler_ptr_->startEvent("1.preprocess");
  // Convert selected layer to OpenCV image
  cv::Mat cvLayer;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<float, 1>(mapOut, inputLayer_, CV_32F, minValue, maxValue, cvLayer);
  cv::normalize(cvLayer, cvLayer, minValue, maxValue, cv::NORM_MINMAX);
  cvLayer.convertTo(cvLayer, CV_32F);
  cvLayer *= 255;

  // Apply threshold and compute obstacle masks
  cv::Mat cvObstacleSpaceMask, cvFreeSpaceMask;
  cv::threshold(cvLayer, cvObstacleSpaceMask, 255 * threshold_, 255, cv::THRESH_BINARY);
  cvObstacleSpaceMask.convertTo(cvObstacleSpaceMask, CV_8UC1);
  cv::bitwise_not(cvObstacleSpaceMask, cvFreeSpaceMask);

  profiler_ptr_->endEvent("1.preprocess");

  // Compute SDF
  profiler_ptr_->startEvent("2.sdf_computation");
  cv::Mat cvSdfFreeSpace;
  cv::Mat cvSdfObstacleSpace;
  cv::distanceTransform(cvFreeSpaceMask, cvSdfFreeSpace, cv::DIST_L2, 3);          // TODO: parametrize kernel and distance metric
  cv::distanceTransform(cvObstacleSpaceMask, cvSdfObstacleSpace, cv::DIST_L2, 3);  // TODO: parametrize kernel and distance metric
  cv::Mat cvSdf = cvSdfObstacleSpace - cvSdfFreeSpace;
  profiler_ptr_->endEvent("2.sdf_computation");

  profiler_ptr_->startEvent("3.sdf_gradients");
  cv::Mat cvSdfGrad = cvSdf.clone();
  // Smooth SDF
  cv::GaussianBlur(cvSdf, cvSdfGrad, cv::Size(5, 5), 0);

  // Compute gradients
  cv::Mat cvGradientsX;
  cv::Mat cvGradientsY;
  cv::Mat cvGradientsZ(cvSdfGrad.size(), cvSdfGrad.type(), cv::Scalar(0.0));
  cv::Sobel(cvSdfGrad, cvGradientsX, -1, 0, 1, 3);
  cv::Sobel(cvSdfGrad, cvGradientsY, -1, 1, 0, 3);
  // cvGradientsX*=resolution;
  // cvGradientsY*=resolution;

  if (normalizeGradients_) {
    // Compute norm
    cv::Mat sqGradientsX;
    cv::Mat sqGradientsY;
    cv::pow(cvGradientsX, 2, sqGradientsX);
    cv::pow(cvGradientsY, 2, sqGradientsY);
    cv::Mat normNormal;
    cv::sqrt(sqGradientsX + sqGradientsY, normNormal);

    cvGradientsX /= (normNormal + std::numeric_limits<float>::epsilon());
    cvGradientsY /= (normNormal + std::numeric_limits<float>::epsilon());
  }

  // Some normalization to ease visualization
  cvObstacleSpaceMask /= 255.0;
  cvFreeSpaceMask /= 255.0;
  profiler_ptr_->endEvent("3.sdf_gradients");

  // Add layers
  profiler_ptr_->startEvent("4.sdf_add_layers");
  addMatAsLayer(cvSdf, outputLayer_, mapOut, resolution);
  // addMatAsLayer(cvObstacleSpaceMask, outputLayer_ + "_obstacle_space", mapOut);
  // addMatAsLayer(cvFreeSpaceMask, outputLayer_  + "_free_space", mapOut);
  addMatAsLayer(cvGradientsX, outputLayer_ + "_gradient_x", mapOut);
  addMatAsLayer(cvGradientsY, outputLayer_ + "_gradient_y", mapOut);
  addMatAsLayer(cvGradientsZ, outputLayer_ + "_gradient_z", mapOut);
  profiler_ptr_->endEvent("4.sdf_add_layers");

  mapOut.setBasicLayers({});

  // Timing
  // clock_t end_t = clock();
  // double total_t = ((double)(end_t - start_t)) / CLOCKS_PER_SEC;
  profiler_ptr_->endEvent("0.update");
  ROS_DEBUG_STREAM(profiler_ptr_->getReport());
  // auto toc = std::chrono::high_resolution_clock::now();
  // double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(toc - tic).count()/1000;
  // ROS_DEBUG_STREAM("[SignedDistanceField2dFilter] Process time: " << elapsedTime << " ms");
  // ROS_DEBUG_STREAM("[SignedDistanceField2dFilter] Process time (clock_t): " << total_t*1000 << " ms");

  return true;
}

template <typename T>
void SignedDistanceField2dFilter<T>::addMatAsLayer(const cv::Mat& mat, const std::string& layerName, grid_map::GridMap& gridMap,
                                                   double resolution) {
  // Get max and min
  double minDistance, maxDistance;
  cv::minMaxLoc(mat, &minDistance, &maxDistance);

  minDistance *= resolution;
  maxDistance *= resolution;

  // Normalize sdf to get a greyscale image
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  normalized.convertTo(normalized, CV_32F);

  // Add layer to elevation map
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(normalized, layerName, gridMap, minDistance, maxDistance);
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::SignedDistanceField2dFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::SignedDistanceField2dFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)