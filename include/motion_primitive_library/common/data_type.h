/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#ifndef BASIC_DATA_H
#define BASIC_DATA_H
#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

///Set red font in printf funtion 
#define ANSI_COLOR_RED "\x1b[1;31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[1;32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[1;34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[1;36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"

///Pre-allocated std::vector for Eigen.
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/*! \brief Rename the float type used in lib 

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;

///Pre-allocated std::vector for Eigen.
template <int N> 
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Pre-allocated std::vector for Eigen.
template <int N> 
using Veci = Eigen::Matrix<int, N, 1>;
///Pre-allocated std::vector for Eigen.
template <int M, int N> 
using Matf = Eigen::Matrix<decimal_t, M, N>;
///Pre-allocated std::vector for Eigen.
template <int N> 
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
///Pre-allocated std::vector for Eigen.
template <int N> 
using vec_Vecf = vec_E<Vecf<N>>;
///Pre-allocated std::vector for Eigen.
template <int N> 
using vec_Veci = vec_E<Veci<N>>;



///Column vector in float of size 2.
typedef Vecf<2> Vec2f;
///Column vector in int of size 2.
typedef Veci<2> Vec2i;
///Column vector in float of size 3.
typedef Vecf<3> Vec3f;
///Column vector in int of size 3.
typedef Veci<3> Vec3i;
///Column vector in float of size 4.
typedef Vecf<4> Vec4f;
///Column vector in float of size 6.
typedef Vecf<6> Vec6f;

///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;

///2x2 Matrix in float
typedef Matf<2, 2> Mat2f;
///3x3 Matrix in float
typedef Matf<3, 3> Mat3f;
///4x4 Matrix in float
typedef Matf<4, 4> Mat4f;
///6x6 Matrix in float
typedef Matf<6, 6> Mat6f;

///Column vector in float with dynamic size
typedef Vecf<Eigen::Dynamic> VecDf;
///Nx3 matrix in float 
typedef Matf<Eigen::Dynamic, 3> MatD3f;
///NxN matrix in float
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

///Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
///Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;
///std::pair of Eigen::Vector3d
typedef std::pair<Vec3f, Vec3f> pair_Vec3f;

///Ellipsoid: first is the Affine Transform, second is the center
typedef std::pair<Mat3f, Vec3f> Ellipsoid;
///Vector of Ellipsoids
typedef vec_E<Ellipsoid> vec_Ellipsoid;

///Pair of linear equality constraints in 3D <A, b> as: \f$Ax \leq b\f$
typedef std::pair<MatD3f, VecDf> LinearConstraint3f; // Ax <= b
///Vector of LinearConstraint3f
typedef vec_E<LinearConstraint3f> vec_LinearConstraint3f;


///Face class
class Face {
  public:
    Vec3f p;
    Vec3f n;
    bool pass;

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Face(Vec3f _p, Vec3f _n):
      p(_p), n(_n), pass(true) {}
    Face(Vec3f _p, Vec3f _n, bool _pass):
      p(_p), n(_n), pass(_pass) {}
};

///Polyhedron, consists of faces
typedef vec_E<Face> Polyhedron; // composed by planes with form (p, n)
///Vector of Polyhedron
typedef vec_E<Polyhedron> Polyhedra;

///Extreme points of a polyhedron
typedef vec_E<vec_Vec3f> BoundVec3f; // compose by extreme points

#endif
