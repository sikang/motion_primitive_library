/**
 * @file map_planner.h
 * @brief motion planning using voxel map for collision checking
 */

#ifndef MPL_MAP_PLANNER_H
#define MPL_MAP_PLANNER_H

#include <mpl_planner/env/env_map.h>
#include <mpl_planner/common/planner_base.h>

namespace MPL {

using linkedHashMap =
    std::unordered_map<int, std::vector<std::pair<Key, int>>>;
/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim> class MapPlanner: public PlannerBase<Dim, Waypoint<Dim>> {
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug messages
   */
  MapPlanner(bool verbose);
  /// Set map util
  virtual void setMapUtil(const std::shared_ptr<MapUtil<Dim>> &map_util);
  /**
   * @brief Set valid region
   * @param path a sequence of waypoints from a path or trajectory
   * @param search_radius the search distance in each axis
   * @param dense if true, do ray cast between two consecutive points in path
   */
  void setValidRegion(const vec_Vecf<Dim>& path, const Vecf<Dim>& search_radius, bool dense = false);

  /// Get search region
  vec_Vecf<Dim> getSearchRegion() const;
  /// Get linked voxels
  vec_Vecf<Dim> getLinkedNodes() const;
  /**
   * @brief Update edge costs according to the new blocked nodes
   * @param pns the new occupied voxels
   *
   * The function returns affected primitives for debug purpose
   */
  vec_E<Primitive<Dim>> updateBlockedNodes(const vec_Veci<Dim> &pns);
  /**
   * @brief Update edge costs according to the new cleared nodes
   * @param pns the new cleared voxels
   *
   * The function returns affected primitives for debug purpose
   */
  vec_E<Primitive<Dim>> updateClearedNodes(const vec_Veci<Dim> &pns);

  /// Set potential radius
  void setPotentialRadius(const Vecf<Dim>& radius);
  /// Set potential map size
  void setPotentialMapRange(const Vecf<Dim>& range);
  /// Set gradient weight
  void setGradientWeight(decimal_t w);
  /// Set potential weight
  void setPotentialWeight(decimal_t w);

  /// Get the potential cloud, works for 2D and 3D
  vec_Vec3f getPotentialCloud(decimal_t h_max = 1.0);
  /// Get the gradient cloud, works for 2D
  vec_Vec3f getGradientCloud(decimal_t h_max = 1.0, int i = 0);

  /**
   * @brief Generate potential map
   * @param pos center of the potential map range is zero, do global generation
   * @param pow power of potential field
   */
  void updatePotentialMap(const Vecf<Dim>& pos, int pow = 1);

protected:
  /// Create mask for potential
  void createMask(int pow);

  /// Calculate local gradient map
  vec_E<Vecf<Dim>> calculateGradient(const Veci<Dim>& coord1,
                                     const Veci<Dim>& coord2);
  /// Map util
  std::shared_ptr<MapUtil<Dim>> map_util_;

  /// Linked table that records voxel and corresponding primitives passed through
  mutable linkedHashMap lhm_;

  /// Max value for potential
  int8_t H_MAX{100};

  /// Radius of potential for each voxel
  Vecf<Dim> potential_radius_{Vecf<Dim>::Zero()};
  /// Potential map size, centered at the given pos
  Vecf<Dim> potential_map_range_{Vecf<Dim>::Zero()};
  /// Mask for generating potential field around obstacle
  vec_E<std::pair<Veci<Dim>, int8_t>> potential_mask_;
  /// Gradient map
  vec_E<Vecf<Dim>> gradient_map_;
};

/// Planner for 2D OccMap
typedef MapPlanner<2> OccMapPlanner;

/// Planner for 3D VoxelMap
typedef MapPlanner<3> VoxelMapPlanner;
}

#endif
