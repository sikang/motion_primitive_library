/**
 * @file voxel_map_util.h
 * @brief VoxelMapUtil classes
 */

#ifndef MPL_VOXEL_MAP_UTIL_H
#define MPL_VOXEL_MAP_UTIL_H

#include <motion_primitive_library/collision_checking/map_util_base.h>

namespace MPL {
class VoxelMapUtil : public MapUtilBase<3, std::vector<signed char>> {
public:
  VoxelMapUtil() : MapUtilBase() {}

  /*
   void dilate(decimal_t r, decimal_t h) {
    int dilate_xy = std::ceil(r / res_);
    int dilate_z = std::ceil(h / res_); // assum robot is 0.2 m high
    dilate_neighbor_.clear();
    Vec3i n;
    for (n(0) = -dilate_xy; n(0) <= dilate_xy; n(0)++) {
      for (n(1) = -dilate_xy; n(1) <= dilate_xy; n(1)++) {
        for (n(2) = -dilate_z; n(2) <= dilate_z; n(2)++) {
          decimal_t d = res_ * n.topRows(2).norm() - r;
          // if (d >= -0.1 && d <= res_ * 0.71)
          if (d <= res_ * 0.71)
            dilate_neighbor_.push_back(n);
        }
      }
    }

 }
 */

  vec_Vec3f getFreeCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isFree(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  vec_Vec3f getUnknownCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnKnown(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  vec_Vec3f getCloud() {
    vec_Vec3f cloud;
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(n))
            cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  void dilate(const vec_Vec3i& dilate_neighbor) {
    std::vector<signed char> map = map_;
    Vec3i n = Vec3i::Zero();
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(n)) {
            for (const auto &it : dilate_neighbor) {
              if (!isOutSide(n + it))
                map[getIndex(n + it)] = val_occ;
            }
          }
        }
      }
    }
    map_ = map;
  }

  void freeUnKnown() {
    Vec3i n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnKnown(n))
            map_[getIndex(n)] = val_free;
        }
      }
    }
  }

};
  
}
#endif
