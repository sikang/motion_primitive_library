/**
 * @file mp_distancec_map_util.h
 * @brief motion planning map util
 */
#include <motion_primitive_library/planner/env_distance_map.h>
#include <motion_primitive_library/planner/mp_map_util.h>

/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim> class MPDistanceMapUtil : public MPMapUtil<Dim> {
 public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug messages
   */
  MPDistanceMapUtil(bool verbose);
  void setMapUtil(std::shared_ptr<MPL::MapUtil<Dim>> &map_util);
  void setDistanceRadius(const Vecf<Dim>& radius);
  void setMapRange(const Vecf<Dim>& range);
  void setGradientWeight(decimal_t w);
  void setPotentialWeight(decimal_t w);

  vec_Vec3f getPotentialCloud(decimal_t h_max = 1.0);
  vec_Vec3f getGradientCloud(decimal_t h_max = 1.0, int i = 0);

  void createMask(int pow);
  /**
   * @brief Generate distance map
   * @param pos center of the distance map
   range is zero, do global generation
   */
  void updateDistanceMap(const Vecf<Dim>& pos);
 protected:
  vec_E<Vecf<Dim>> calculateGradient(const Veci<Dim>& coord1, const Veci<Dim>& coord2);
  int8_t H_MAX{100};

  Vecf<Dim> distance_radius_;
  Vecf<Dim> map_range_;
  /// Mask for generating potential field around obstacle
  vec_E<std::pair<Veci<Dim>, int8_t>> distance_mask_;
  vec_E<Vecf<Dim>> gradient_map_;


};

/// Planner for 2D OccMap
typedef MPDistanceMapUtil<2> MPDistanceMap2DUtil;

/// Planner for 3D VoxelMap
typedef MPDistanceMapUtil<3> MPDistanceMap3DUtil;
