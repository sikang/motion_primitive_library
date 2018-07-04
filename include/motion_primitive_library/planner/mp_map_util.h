/**
 * @file mp_map_util.h
 * @brief motion planning map util
 */
#include <motion_primitive_library/planner/env_map.h>
#include <motion_primitive_library/planner/mp_base_util.h>

using linkedHashMap =
    std::unordered_map<int, std::vector<std::pair<MPL::Key, int>>>;
/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim> class MPMapUtil : public MPBaseUtil<Dim> {
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug messages
   */
  MPMapUtil(bool verbose);
  /// Set map util
  virtual void setMapUtil(std::shared_ptr<MPL::MapUtil<Dim>> &map_util);
  /**
   * @brief Set valid region
   * @param path a sequence of waypoints from a path or trajectory
   * @param search_radius the search distance in each axis
   * @param dense if true, do ray cast between two consecutive points in path
   */
  void setValidRegion(const vec_Vecf<Dim>& path, const Vecf<Dim>& search_radius, bool dense = false);

  /// Get valid region
  vec_Vecf<Dim> getValidRegion() const;
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
   */
  void updatePotentialMap(const Vecf<Dim>& pos);

protected:
  /// Create mask for potential
  void createMask(int pow);

  /// Calculate local gradient map
  vec_E<Vecf<Dim>> calculateGradient(const Veci<Dim>& coord1,
                                     const Veci<Dim>& coord2);
  /// Map util
  std::shared_ptr<MPL::MapUtil<Dim>> map_util_;
  /// Linked table that records voxel and corresponding primitives passed through
  mutable linkedHashMap lhm_;
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
typedef MPMapUtil<2> MPMap2DUtil;

/// Planner for 3D VoxelMap
typedef MPMapUtil<3> MPMap3DUtil;
