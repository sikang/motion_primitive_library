#include <motion_primitive_library/collision_checking/funnel_util.h>

FunnelUtil::FunnelUtil(decimal_t kp, decimal_t kv, decimal_t v) : kp_(kp), kv_(kv), v_(v) {
}

void FunnelUtil::set_region(const Vec3f& ori, const Vec3f& dim) {
  Polyhedron Vs;
  Vs.push_back(Face(ori + Vec3f(0, dim(1)/2, dim(2)/2), -Vec3f::UnitX()));
  Vs.push_back(Face(ori + Vec3f(dim(0)/2, 0, dim(2)/2), -Vec3f::UnitY()));
  Vs.push_back(Face(ori + Vec3f(dim(0)/2, dim(2)/2, 0), -Vec3f::UnitZ()));
  Vs.push_back(Face(ori + dim - Vec3f(0, dim(1)/2, dim(2)/2), Vec3f::UnitX()));
  Vs.push_back(Face(ori + dim - Vec3f(dim(0)/2, 0, dim(2)/2), Vec3f::UnitY()));
  Vs.push_back(Face(ori + dim - Vec3f(dim(0)/2, dim(1)/2, 0), Vec3f::UnitZ()));

  Vs_ = Vs;
}

Polyhedra FunnelUtil::polyhedra() {
  Polyhedra polys;
  if(!Vs_.empty())
    polys.push_back(Vs_);
  return polys;
}

PCLPointCloud FunnelUtil::toPCL(const vec_Vec3f &obs)
{
  PCLPointCloud cloud;       
  cloud.width = obs.size();
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for(unsigned int i = 0; i < obs.size(); i++)
  {                                                           
    cloud.points[i].x = obs[i](0);
    cloud.points[i].y = obs[i](1);
    cloud.points[i].z = obs[i](2);
  }
  return cloud;
}          

void FunnelUtil::setObstacles(const vec_Vec3f& obs) { 
  obs_.clear();
  if(!Vs_.empty()) {
    for(const auto&it: obs) {
      if(insidePolyhedron(it, Vs_))
        obs_.push_back(it);
    }
  }
  else
    obs_ = obs; 
  PCLPointCloud::Ptr cloud_ptr = boost::make_shared<PCLPointCloud>(toPCL(obs_));
  kdtree_.setInputCloud (cloud_ptr);
}


bool FunnelUtil::isFree(const Primitive3& pr, const Vec3f& x10, Vec3f& x1f) {
  if(!Vs_.empty()) {
    vec_E<Waypoint3> ps = pr.sample(2);
    for(const auto& it: ps) {
      if(!insidePolyhedron(it.pos, Vs_))
        return false;
    }
  }

  //decimal_t max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
  //int n = std::ceil(max_v * pr.t() / axe_(0));
  PrimitiveFunnel<3> f(pr, kp_, kv_);
  auto Es = f.compute(x10.topRows(2), v_, 5);
  x1f.topRows(2) = f.get_wf();

  for(const auto& E: Es) {
    float radius = E.first(0, 0);
    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    searchPoint.x = E.second(0);
    searchPoint.y = E.second(1);
    searchPoint.z = E.second(2);

    if ( kdtree_.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
      return false;
    }
  }

  return true;

}


bool FunnelUtil::insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs) {
  for (const auto& v : Vs) {
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    if (a.dot(pt) - b > 1e-3)
      return false;
  }
  return true;
}

bool FunnelUtil::insideEllipsoid(const Ellipsoid& E, const vec_Vec3f& O) {
  for (const auto &it : O) {
    decimal_t d = (E.first.inverse() * (it - E.second)).norm();
    if (d < 1) return true;
  }
  return false;
}

