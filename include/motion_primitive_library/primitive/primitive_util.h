/**
 * @file primitive_util.h
 * @brief Simple primitive utils
 */
#ifndef PRIMITIVE_UTIL_H
#define PRIMITIVE_UTIL_H

#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/primitive/trajectory.h>

///Print all coefficients in primitive p
void print_coeffs(const Primitive& p);
///Print max dynamic infomation in primitive p
void print_max(const Primitive& p);

/**
 * @brief estimate the ellipsoid 
 * @param axe the length of semi-axes of the ellipsoid
 * @param pos center of ellipsoid 
 * @param acc acceleration
 */
Ellipsoid generate_ellipsoid(const Vec3f& axe, const Vec3f& pos, const Vec3f& acc);

///Sample N ellipsoids along the primitive
vec_Ellipsoid sample_ellipsoids(const Primitive& pr, const Vec3f& axe, int N);

///Sample N ellipsoids along the trajectory
vec_Ellipsoid sample_ellipsoids(const Trajectory& traj, const Vec3f& axe, int N);

///Approximate the maximum roll/pitch along the trajectory
void max_attitude(const Trajectory& traj, int N);
#endif

