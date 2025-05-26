/*
 * GeodesicDistanceField2dSnapFilter.cpp
 *
 *  Extension of GeodesicDistanceField2dFilter that "repairs" an invalid
 *  goal by projecting it onto the medial axis contained in a Signed-Distance
 *  Field (SDF) layer.
 *
 *  Author: <your-name>
 */

#include <grid_map_filters_drs/GeodesicDistanceField2dFilter.hpp>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <ros/ros.h>

using namespace filters;

namespace grid_map {

template<typename T>
class GeodesicDistanceField2dSnapFilter : public GeodesicDistanceField2dFilter<T> {
public:
  GeodesicDistanceField2dSnapFilter() = default;
  ~GeodesicDistanceField2dSnapFilter() override = default;

  //-----------------------------------------
  // Parameter loading
  //-----------------------------------------
  bool configure() override {
    // Let the parent read its parameters first
    if (!GeodesicDistanceField2dFilter<T>::configure()) {
      return false;
    }

    // // ── Signed‑distance‑field layer name (mandatory) ──
    // if (!FilterBase<T>::getParam(std::string("sdf_layer"), sdfLayer_)) {
    //   ROS_ERROR("GDF-Snap filter: parameter `sdf_layer` missing");
    //   return false;
    // }

    // // ── Optional search radius ──
    // if (!FilterBase<T>::getParam(std::string("sdf_search_radius"), sdfSearchRadius_)) {
    //   sdfSearchRadius_ = 3.0;   // [m] default
    // }

    // ── Derive the free‑space layer name from the same YAML param set that
    // the parent already used.  We fetch it again rather than access the
    // private member in the base class (cannot be accessed from here).
    std::string outputLayerName;
    if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayerName)) {
      ROS_ERROR("GDF-Snap filter: unable to read `output_layer` parameter");
      return false;
    }
    freeSpaceLayerName_ = outputLayerName + "_free_space";

    // ── Optional number of samples for line-of-sight check ──
    if (!FilterBase<T>::getParam(std::string("los_num_samples"), losNumSamples_)) {
      losNumSamples_ = 20;   // default
    }

    return true;
  }

protected:
  //-----------------------------------------
  // Attractor correction
  //-----------------------------------------
  grid_map::Index getAttractorIndex(const T& gridMap,
                                    const grid_map::Position& attractorPosition) override {
    // Init stuff
    Position attractor = attractorPosition;

    grid_map::Index attractorIdx;
    grid_map::Index centerIdx;

    gridMap.getIndex(gridMap.getPosition(), centerIdx);
    gridMap.getIndex(attractor, attractorIdx);

    // Check if the attractor is outside the map to snap it to the edge
    if (!gridMap.isInside(attractor)) {
        // Get closest points
        attractor = gridMap.getClosestPositionInMap(attractor);
        // Get index
        gridMap.getIndex(attractor, attractorIdx);
        // Enforce the index to be valid
        attractorIdx.x() = std::max(std::min(attractorIdx.x(), gridMap.getSize().x() - 1), 0);
        attractorIdx.y() = std::max(std::min(attractorIdx.y(), gridMap.getSize().y() - 1), 0);
    }


    // ****************************************
    // GET BEST IDX USING SIGNED DISTANCE FIELD
    // ****************************************
    // // Convert search radius to cells
    // const double res = gridMap.getResolution();

    // float bestSdf = -std::numeric_limits<float>::infinity();
    // grid_map::Index bestIdx = attractorIdx;

    // for (grid_map::SpiralIterator it(gridMap, attractor, sdfSearchRadius_); !it.isPastEnd(); ++it) {
    //     const grid_map::Index& idx = *it;

    //     // Only consider cells that are valid in SDF layer and traversable
    //     if (!gridMap.isValid(idx, sdfLayer_)) continue;
    //     if (gridMap.at(freeSpaceLayerName_, idx) == 0) continue;

    //     float val = gridMap.at(sdfLayer_, idx);
    //     if (val > bestSdf) {
    //         bestSdf = val;
    //         bestIdx = idx;
    //         ROS_INFO_STREAM("Best SDF value: " << bestSdf << ", at index: " << bestIdx.x() << ", " << bestIdx.y());
    //     }
    // }

    // if (bestSdf > 0) {
    //     // Get index from position
    //     grid_map::Position snappedPos;
    //     gridMap.getPosition(bestIdx, snappedPos);
    //     return bestIdx;
    // }

    // // Fallback: publish best index from parent
    // ROS_ERROR_STREAM("GDF-Snap filter: Could not bet GDF-snap, falling back to pure GDF.");
    // grid_map::Index baseIdx = GeodesicDistanceField2dFilter<T>::getAttractorIndex(gridMap, attractorPosition);
    // return bestIdx;
    // ****************************************

    // ****************************************
    // GET BEST IDX USING LINE OF SIGHT
    // ****************************************
    double distanceToCenter = (gridMap.getPosition() - attractor).norm();
    for (grid_map::SpiralIterator it(gridMap, attractor, distanceToCenter); !it.isPastEnd(); ++it) {
        const grid_map::Index& idx = *it;

        // Only consider cells that are valid and traversable
        if (!gridMap.isValid(idx, freeSpaceLayerName_)) continue;
        if (gridMap.at(freeSpaceLayerName_, idx) == 0) continue;

        // Check if idx cell has a clear line of sight to center cell
        bool lineOfSight = true;
        Eigen::Vector2f end(centerIdx.x(), centerIdx.y());
        Eigen::Vector2f start(idx.x(), idx.y());
        for (int i = 1; i < losNumSamples_; ++i) {
            float t = static_cast<float>(i) / losNumSamples_;
            Eigen::Vector2f interp = start + t * (end - start);
            grid_map::Index sampleIdx(std::round(interp.x()), std::round(interp.y()));
            if (!gridMap.isValid(sampleIdx, freeSpaceLayerName_) ||
                gridMap.at(freeSpaceLayerName_, sampleIdx) == 0) {
                lineOfSight = false;
                break;
            }
        }

        if (lineOfSight) {
          attractorIdx = idx;
          break;
        }
    }
    return attractorIdx;
  }

private:
  //-----------------------------------------
  // Members added by the snap filter
  //-----------------------------------------
//   std::string sdfLayer_;             //!< Name of Signed‑Distance‑Field layer
//   double      sdfSearchRadius_{3.0}; //!< [m] search radius around attractor
  std::string freeSpaceLayerName_;   //!< Derived layer name <output_layer>_free_space
  int         losNumSamples_{8};     //!< Number of samples for line-of-sight check
};

} // namespace grid_map

//────────── Plugin export ──────────

template class grid_map::GeodesicDistanceField2dSnapFilter<grid_map::GridMap>;
PLUGINLIB_EXPORT_CLASS(grid_map::GeodesicDistanceField2dSnapFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)


