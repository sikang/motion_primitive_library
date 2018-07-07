/**
 * @file trajectory.h
 * @brief Trajectory class
 */


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <motion_primitive_library/common/primitive.h>
#include <motion_primitive_library/common/lambda.h>

/**
 * @brief Trajectory class
 *
 * A trajectory is composed by multiple end-to-end connected primitives, so-called piece-wise polynomials
 */
template <int Dim>
class Trajectory {
  public:
    /**
     * @brief Empty constructor
     */
    Trajectory() {}
    /**
     * @brief Construct from multiple primitives
     */
    Trajectory(const vec_E<Primitive<Dim>>& prs) : segs(prs) {
      taus.push_back(0);
      for (const auto &pr : prs)
        taus.push_back(pr.t() + taus.back());
      Ts = taus;
      total_t_ = taus.back();
    }

    /**
     * @brief Return the total duration of the trajectory
     */
    decimal_t getTotalTime() const { return total_t_; }

    /**
     * @brief Retrieve scaling factor
     */
    Lambda lambda() const { return lambda_; }

    /**
     * @brief Evaluate state at t, return false if fails to evaluate
     *
     * If t is out of scope, we set t to be the closer bound (0 or total_t_) and return the evaluation;
     * The failure case is when lambda is ill-posed such that \f$t = \lambda(\tau)^{-1}\f$ has no solution
     */
    bool evaluate(decimal_t time, Waypoint<Dim>& p) const {
      decimal_t tau = lambda_.getTau(time);
      if (tau < 0)
        tau = 0;
      if (tau > total_t_)
        tau = total_t_;

      decimal_t lambda = 1;
      decimal_t lambda_dot = 0;

      if (lambda_.exist()) {
        VirtualPoint vt = lambda_.evaluate(tau);
        lambda = vt.p;
        lambda_dot = vt.v;
      }

      for (int id = 0; id < (int)segs.size(); id++) {
        if (tau >= taus[id] && tau <= taus[id + 1]) {
          tau -= taus[id];
          for (int j = 0; j < Dim; j++) {
            const auto pr = segs[id].traj(j);
            p.pos(j) = pr.p(tau);
            p.vel(j) = pr.v(tau) / lambda;
            p.acc(j) = pr.a(tau) / lambda / lambda -
              p.vel(j) * lambda_dot / lambda / lambda / lambda;
            p.jrk(j) = pr.j(tau) / lambda / lambda -
              3 / power(lambda, 3) * p.acc(j) * p.acc(j) * lambda_dot +
              3 / power(lambda, 4) * p.vel(j) * lambda_dot *
              lambda_dot;
          }
          return true;
        }
      }

      printf("cannot find tau according to time: %f\n", time);
      return false;
    }


    /**
     * @brief Scale according to ratio at start and end (velocity only)
     */
    bool scale(decimal_t ri, decimal_t rf) {
      std::vector<VirtualPoint> vs;
      VirtualPoint vi, vf;
      vi.p = 1.0 / ri;
      vi.v = 0;
      vi.t = 0;

      vf.p = 1.0 / rf;
      vf.v = 0;
      vf.t = taus.back();

      vs.push_back(vi);
      vs.push_back(vf);
      Lambda ls(vs);
      lambda_ = ls;
      std::vector<decimal_t> ts;
      for (const auto &tau : taus)
        ts.push_back(lambda_.getT(tau));
      Ts = ts;
      total_t_ = Ts.back();
      return true;
    }


    /**
     * @brief Scale down the whole trajectory according to mv
     */
    bool scale_down(decimal_t mv, decimal_t ri, decimal_t rf) {
      std::vector<VirtualPoint> vs;
      VirtualPoint vi, vf;
      vi.p = ri;
      vi.v = 0;
      vi.t = 0;

      vf.p = rf;
      vf.v = 0;
      vf.t = taus.back();

      vs.push_back(vi);
      for (int id = 0; id < (int)segs.size(); id++) {
        for (int i = 0; i < 3; i++) {
          if (segs[id].max_vel(i) > mv) {
            std::vector<decimal_t> ts = segs[id].traj(i).extrema_vel(segs[id].t());
            if (id != 0)
              ts.push_back(0);
            ts.push_back(segs[id].t());
            for (const auto &tv : ts) {
              Vec4f p = segs[id].traj(i).evaluate(tv);
              decimal_t v = p(1);
              decimal_t lambda_v = fabs(v) / mv;
              if (lambda_v <= 1)
                continue;

              VirtualPoint vt;
              vt.p = lambda_v;
              vt.v = 0;
              vt.t = tv + taus[id];
              vs.push_back(vt);
            }
          }
        }
      }

      vs.push_back(vf);

      std::sort(
        vs.begin(), vs.end(),
        [](const VirtualPoint &i, const VirtualPoint &j) { return i.t < j.t; });
      decimal_t max_l = 1;
      for (const auto &v : vs) {
        if (v.p > max_l)
          max_l = v.p;
      }

      if (max_l <= 1)
        return false;

      // printf("max_l: %f\n", max_l);
      for (int i = 1; i < (int)vs.size() - 1; i++)
        vs[i].p = max_l;
      std::vector<VirtualPoint> vs_s;
      vs_s.push_back(vs.front());
      for (const auto &v : vs)
        if (v.t > vs_s.back().t)
          vs_s.push_back(v);

      lambda_ = Lambda(vs_s);

      std::vector<decimal_t> ts;
      for (const auto &tau : taus)
        ts.push_back(lambda_.getT(tau));
      Ts = ts;
      total_t_ = Ts.back();
      return true;
    }


    /**
     * @brief Sample N+1 states using uniformed time
     */
    vec_E<Waypoint<Dim>> sample(int N) const {
      vec_E<Waypoint<Dim>> ps(N+1);

      decimal_t dt = total_t_ / N;
      for (int i = 0; i <= N; i++) {
        Waypoint<Dim> pt;
        evaluate(i * dt, pt);
        ps[i] = pt;
      }

      return ps;
    }

    /**
     * @brief Return total efforts of primitive for the given duration: \f$J(i) = \int_0^t |p^{(i+1)}(t)|^2dt\f$
     *
     * Return J is the summation of efforts in all three dimensions
     * @param i effort is defined as \f$i\f$-th derivative of polynomial
     */
    decimal_t J(int i) const {
      decimal_t J = 0;
      for (const auto &seg : segs)
        J += seg.J(i);
      return J;
    }

    ///Get time for each segment
    std::vector<decimal_t> getSegsT() const {
      std::vector<decimal_t> dts;
      for (int i = 0; i < (int)Ts.size() - 1; i++)
        dts.push_back(Ts[i + 1] - Ts[i]);
      return dts;
    }

    ///Segments of primitives
    vec_E<Primitive<Dim>> segs;
    ///Time in virtual domain
    std::vector<decimal_t> taus;
    ///Time in actual domain
    std::vector<decimal_t> Ts;
    ///Total time of the trajectory
    decimal_t total_t_;
    ///Scaling object
    Lambda lambda_;
};

///Trajectory in 2D
typedef Trajectory<2> Trajectory2D;

///Trajectory in 3D
typedef Trajectory<3> Trajectory3D;

/**************************** Utils *************************/
///Sample N ellipsoids along the trajectory
template <int Dim>
vec_Ellipsoid sample_ellipsoids(const Trajectory<Dim>& traj, const Vec3f& axe, int N) {
  vec_Ellipsoid Es;

  decimal_t dt = traj.getTotalTime() / N;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint<Dim> pt;
    if(traj.evaluate(t, pt))
      Es.push_back(generate_ellipsoid<Dim>(axe, pt.pos, pt.acc));
  }

  return Es;
}

///Approximate the maximum roll/pitch along the trajectory
template <int Dim>
void max_attitude(const Trajectory<Dim>& traj, int N) {
  decimal_t dt = traj.getTotalTime() / N;
  decimal_t max_roll = 0;
  decimal_t max_roll_time = 0;
  decimal_t max_pitch = 0;
  decimal_t max_pitch_time = 0;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint<Dim> pt;
    if(traj.evaluate(t, pt)) {
      const Vec3f b3 = Dim == 2 ?
        (Vec3f(pt.acc(0), pt.acc(1), 0) +
         9.81 * Vec3f::UnitZ()).normalized() :
        (pt.acc + 9.81 * Vec3f::UnitZ()).normalized();
      decimal_t roll = std::atan2(b3(1), b3(2));
      decimal_t pitch= std::atan2(b3(0), b3(2));
      if(std::fabs(roll) > std::fabs(max_roll)) {
        max_roll = roll;
        max_roll_time = t;
      }
      if(std::fabs(pitch) > std::fabs(max_pitch)) {
        max_pitch = pitch;
        max_pitch_time = t;
      }

    }
  }

  printf("max roll: %f at [%f]\n", max_roll * 180/M_PI, max_roll_time);
  printf("max pitch: %f at [%f]\n", max_pitch * 180/M_PI, max_pitch_time);
}
#endif
