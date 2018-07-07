/**
 * @file data_utils.h
 * @brief Basic data utils
 */
#ifndef DATA_UTILS_H
#define DATA_UTILS_H
#include <motion_primitive_library/common/data_type.h>

/**
 * @brief estimate the ellipsoid in 3D
 * @param axe the length of semi-axes of the ellipsoid
 * @param pos center of ellipsoid
 * @param acc acceleration
 */
template <int Dim>
Ellipsoid generate_ellipsoid(const Vec3f& axe,
                             const Vecf<Dim>& pos,
                             const Vecf<Dim>& acc) {
  Vec3f ellipsoid_center = Dim == 2? Vec3f(pos(0), pos(1), 0) : pos;
  Vec3f ellipsoid_acc = Dim == 2? Vec3f(acc(0), acc(1), 0) : acc;

  const Vec3f b3 = (ellipsoid_acc + 9.81 * Vec3f::UnitZ()).normalized();
  const Vec3f bc(std::cos(0), std::sin(0), 0);
  const Vec3f b2 = b3.cross(bc).normalized();
  const Vec3f b1 = b2.cross(b3).normalized();
  Mat3f R;
  R << b1, b2, b3;

  Mat3f C = Mat3f::Identity();
  C(0, 0) = axe(0);
  C(1, 1) = axe(1);
  C(2, 2) = axe(2);
  C = R * C * R.transpose();
  return std::make_pair(C, ellipsoid_center);
}

#endif
