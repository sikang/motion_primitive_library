/**
 * @file funnel_util.h
 * @brief funnel util util class
 */
#ifndef FUNNEL_UTIL_H
#define FUNNEL_UTIL_H
#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/primitive/primitive_util.h>
#include <motion_primitive_library/primitive/primitive_funnel.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/make_shared.hpp>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KDTree;

/**
 * @brief Collision checking inside a Safe Flight Corridor (SFC)
 *
 * SFC is an ordered collection of convex polyhedra that models free space
 */
class FunnelUtil {
  public:
    /**
     * @brief Simple constructor
     * @param r robot radius
     * @param h robot height, default as 0.1m
     */
    FunnelUtil(decimal_t kp, decimal_t kv, decimal_t v);
    ///Set obstacles
    void setObstacles(const vec_Vec3f& obs);
    ///Set bounding box
    void set_region(const Vec3f& ori, const Vec3f& dim);
    ///Get bounding box
    Polyhedron virtual_wall(const Vec3f& pos);

    ///Get polyhedra
    Polyhedra polyhedra();
    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive3D &pr, const Vec3f &x10, Vec3f &wf);
    ///Convert obstacle points into pcl point cloud
    PCLPointCloud toPCL(const vec_Vec3f &obs);
 private:
    ///Check if a point pt is inside the given polyhedron
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);
    ///Check if a point in O is inside the given ellipsoid
    bool insideEllipsoid(const Ellipsoid& E, const vec_Vec3f& O);

    decimal_t kp_;
    decimal_t kv_;
    decimal_t v_;
    ///obstacle points
    vec_Vec3f obs_;
    ///obstacles in kd tree form
    KDTree kdtree_;
    ///Bounding box
    Polyhedron Vs_;
};
#endif


