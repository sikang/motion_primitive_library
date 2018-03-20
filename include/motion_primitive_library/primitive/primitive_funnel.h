/**
 * @file primitive_funnel.h
 * @brief PrimitiveFunnel classes
 */

#ifndef PRIMITIVE_FUNNEL_H
#define PRIMITIVE_FUNNEL_H
#include <motion_primitive_library/primitive/primitive.h>
#include <unsupported/Eigen/MatrixFunctions>

/** 
 * @brief PrimitiveFunnel class
 *
 * Contains \f$n\f$ 1D primitives corresponding to each axis individually.
 */
template <int Dim>
class PrimitiveFunnel {
  public:
    PrimitiveFunnel(const Primitive<Dim>& _pr, double _kp, double _kv) :
      pr(_pr), kp(_kp), kv(_kv) {}


    vec_Ellipsoid compute(const Vec2f& x10, const decimal_t& v, int n = 10) {
      const decimal_t T = pr.t();
      const auto p0 = pr.evaluate(0);

      Matf<2, 2> A;
      A << 0, 1, 
        -kp, -kv;
      const Matf<2, 2> Ainv = A.inverse();
      const Matf<2, 2> I = Matf<2, 2>::Identity();
      decimal_t dt = T / n;

      vec_Ellipsoid es;

      for(decimal_t t = 0; t <= T; t+=dt) {
        const Matf<2, 2> Aexp = (A*t).exp();
        const Vec2f x = Aexp*x10+
          Ainv*(Aexp-I)*Vec2f(0, kp*p0.pos(0)+kv*p0.vel(0)+p0.acc(0)+v)+
          Ainv*Ainv*(Aexp-(A*t+I))*Vec2f(0, kp*p0.vel(0)+kv*p0.acc(0))+
          Ainv*Ainv*Ainv*(Aexp-(0.5*A*A*t*t+A*t+I))*Vec2f(0, kp*p0.acc(0));
        auto w = pr.evaluate(t);
        decimal_t r = x(0) - w.pos(0);
        Mat3f C = r*Mat3f::Identity();
        es.push_back(std::make_pair(C, w.pos));
        if(t == T) 
          wf = x;
      }

      return es;
    }

    vec_E<Waypoint<Dim>> compute(const Waypoint<Dim>& p1, const Vec2f& w, int n = 10) {
      const decimal_t T = pr.t();
      const auto p0 = pr.evaluate(0);

      Matf<2, 2> A;
      A << 0, 1, 
        -kp, -kv;
      const Matf<2, 2> Ainv = A.inverse();
      const Matf<2, 2> I = Matf<2, 2>::Identity();
      decimal_t dt = T / n;
      Vec2f x10(p1.pos(0), p1.vel(0));
      Vec2f y10(p1.pos(1), p1.vel(1));

      vec_E<Waypoint<Dim>> pts;

      for(decimal_t t = 0; t <= T; t+=dt) {
        const Matf<2, 2> Aexp = (A*t).exp();
        const Vec2f x = Aexp*x10+
          Ainv*(Aexp-I)*Vec2f(0, kp*p0.pos(0)+kv*p0.vel(0)+p0.acc(0)+w(0))+
          Ainv*Ainv*(Aexp-(A*t+I))*Vec2f(0, kp*p0.vel(0)+kv*p0.acc(0))+
          Ainv*Ainv*Ainv*(Aexp-(0.5*A*A*t*t+A*t+I))*Vec2f(0, kp*p0.acc(0));
        const Vec2f y = Aexp*y10+
          Ainv*(Aexp-I)*Vec2f(0, kp*p0.pos(1)+kv*p0.vel(1)+p0.acc(1)+w(1))+
          Ainv*Ainv*(Aexp-(A*t+I))*Vec2f(0, kp*p0.vel(1)+kv*p0.acc(1))+
          Ainv*Ainv*Ainv*(Aexp-(0.5*A*A*t*t+A*t+I))*Vec2f(0, kp*p0.acc(1));

        Waypoint<Dim> w;
        w.pos(0) = x(0), w.pos(1) = y(0);
        w.vel(0) = x(1), w.vel(1) = y(1);
        if(Dim == 3)
          w.pos(2) = 0, w.vel(2) = 0;
        pts.push_back(w);
      }

      return pts;
    }

    Vec2f get_wf() { return wf; }
  private:
    Primitive<Dim> pr;
    decimal_t kp, kv;
    Vec2f wf;
};

#endif
