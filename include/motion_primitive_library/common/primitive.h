/**
 * @file primitive.h
 * @brief Primitive classes
 */

#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include <motion_primitive_library/common/waypoint.h>
#include <motion_primitive_library/common/data_utils.h>
#include "math.h"

/**
 * @brief Primitive1D class
 *
 * Assume the 1D primitive is the n-th order polynomial with n = 5 as
 * \f$p(t) = \frac{c(0)}{120}t^5+\frac{c(1)}{24}t^4+\frac{c(2)}{6}t^3+\frac{c(3)}{2}t^2+c(4)t+c(5) = 0\f$
 */
class Primitive1D {
 public:
  /************************* Constructors *************************/
  /// Empty constructor
  Primitive1D() {}

    /**
     * @brief Construct from known coefficients
     * @param coeff[0] is the coefficient of the highest order
     */
    Primitive1D(const Vec6f &coeff) : c(coeff) {}

    /// Construct 1D primitive from an initial state (p) and an input control (u)
    Primitive1D(decimal_t p, decimal_t u) {
      c << 0, 0, 0, 0, u, p;
    }

    /// Construct 1D primitive from an initial state (p, v) and an input control (u)
    Primitive1D(Vec2f state, decimal_t u) {
      c << 0, 0, 0, u, state(1), state(0);
    }

    /// Construct 1D primitive from an initial state (p, v, a) and an input control (u)
    Primitive1D(Vec3f state, decimal_t u) {
      c << 0, 0, u, state(2), state(1), state(0);
    }

    /// Construct 1D primitive from an initial state (p, v, a, j) and an input control (u)
    Primitive1D(Vec4f state, decimal_t u) {
      c << 0, u, state(3), state(2), state(1), state(0);
    }

    /// Construct 1D primitive from an initial state (p1) to a goal state (p2), given duration t
    Primitive1D(decimal_t p1, decimal_t p2,  decimal_t t) {
      c << 0, 0, 0, 0, (p2 - p1) / t, p1;
    }

    /// Construct 1D primitive from an initial state (p1, v1) to a goal state (p2, v2), given duration t
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2, decimal_t t) {
      Mat4f A;
      A << 0, 0, 0, 1,
        0, 0, 1, 0,
        power(t, 3) / 6, t * t / 2, t, 1,
        t * t / 2, t, 1, 0;
      Vec4f b;
      b << p1, v1, p2, v2;
      Vec4f cc = A.inverse() * b;
      c << 0, 0, cc(0), cc(1), cc(2), cc(3);
    }

    /// Construct 1D primitive from an initial state (p1, v1, a1) to a goal state (p2, v2, a2), given duration t
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1,
                decimal_t p2, decimal_t v2, decimal_t a2, decimal_t t) {
      Mat6f A;
      A << 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 1, 0, 0,
        power(t, 5) / 120, power(t, 4) / 24, power(t, 3) / 6, t * t / 2, t,
        1, power(t, 4) / 24, power(t, 3) / 6, t * t / 2, t, 1,
        0, power(t, 3) / 6, t * t / 2, t, 1, 0, 0;
      Vec6f b;
      b << p1, v1, a1, p2, v2, a2;
      c = A.inverse() * b;
    }

    /*************************** Member functions **************************/
    /**
     * @brief Return total efforts of 1D primitive for the given duration: \f$J(t, i) = \int_0^t |p^{i}(t)|^2dt\f$
     * @param t assume the duration is from 0 to t
     * @param i effort is defined as \f$i\f$-th derivative of polynomial
     */
    decimal_t J(decimal_t t, int i) const {
      // i = 1, return integration of square of vel
      if (i == 1)
        return c(0) * c(0) / 5184 * power(t, 9) + c(0) * c(1) / 576 * power(t, 8) +
          (c(1) * c(1) / 252 + c(0) * c(2) / 168) * power(t, 7) +
          (c(0) * c(3) / 72 + c(1) * c(2) / 36) * power(t, 6) +
          (c(2) * c(2) / 20 + c(0) * c(4) / 60 + c(1) * c(3) / 15) * power(t, 5) +
          (c(2) * c(3) / 4 + c(1) * c(4) / 12) * power(t, 4) +
          (c(3) * c(3) / 3 + c(2) * c(4) / 3) * power(t, 3) +
          c(3) * c(4) * t * t + c(4) * c(4) * t;
      // i = 2, return integration of square of acc
      else if (i == 2)
        return c(0) * c(0) / 252 * power(t, 7) + c(0) * c(1) / 36 * power(t, 6) +
          (c(1) * c(1) / 20 + c(0) * c(2) / 15) * power(t, 5) +
          (c(0) * c(3) / 12 + c(1) * c(2) / 4) * power(t, 4) +
          (c(2) * c(2) / 3 + c(1) * c(3) / 3) * power(t, 3) +
          c(2) * c(3) * t * t + c(3) * c(3) * t;
      // i = 3, return integration of square of jerk
      else if (i == 3)
        return c(0) * c(0) / 20 * power(t, 5) + c(0) * c(1) / 4 * power(t, 4) +
          (c(1) * c(1) + c(0) * c(2)) / 3 * power(t, 3) +
          c(2) * c(2) * t * t +c(2) * c(2) * t;
      // i = 4, return integration of square of snap
      else if (i == 4)
        return c(0) * c(0) / 3 * power(t, 3) + c(0) * c(1) * t * t +
          c(1) * c(1) * t;
      else
        return 0;
    }


    /// Return coffecients
    Vec6f coeff() const {
      return c;
    }

    /// Return \f$p\f$ at time \f$t\f$
    decimal_t p(decimal_t t) const {
      return c(0) / 120 * power(t, 5) + c(1) / 24 * power(t, 4) +
        c(2) / 6 * power(t, 3) + c(3) / 2 * t * t + c(4) * t + c(5);
    }

    /// Return \f$v\f$ at time \f$t\f$
    decimal_t v(decimal_t t) const {
      return c(0) / 24 * power(t, 4) + c(1) / 6 * power(t, 3) +
        c(2) / 2 * t * t + c(3) * t + c(4);
    }

    /// Return \f$a\f$ at time \f$t\f$
    decimal_t a(decimal_t t) const {
        return c(0) / 6 * power(t, 3) + c(1) / 2 * t * t + c(2) * t + c(3);
    }

    /// Return \f$j\f$ at time \f$t\f$
    decimal_t j(decimal_t t) const {
        return c(0) / 2 * t * t + c(1) * t + c(2);
    }

    /**
     * @brief Return vector of time \f$t\f$ for velocity extrema
     *
     * Velocities at both ends (0, t) are not considered
     */
    std::vector<decimal_t> extrema_v(decimal_t t) const {
      std::vector<decimal_t> roots = solve(0, c(0) / 6, c(1) / 2, c(2), c(3));
      std::vector<decimal_t> ts;
      for (const auto &it : roots) {
        if (it > 0 && it < t)
          ts.push_back(it);
        else if(it >= t)
          break;
      }
      return ts;
    }

    /**
     * @brief Return vector of time \f$t\f$ for acceleration extrema
     *
     * Accelerations at both ends (0, t) are not considered
     */
    std::vector<decimal_t> extrema_a(decimal_t t) const {
      std::vector<decimal_t> roots = solve(0, 0, c(0) / 2, c(1), c(2));
      std::vector<decimal_t> ts;
      for (const auto &it : roots) {
        if (it > 0 && it < t)
          ts.push_back(it);
        else if(it >= t)
          break;
      }
      return ts;
    }

    /**
     * @brief Return vector of time \f$t\f$ for jerk extrema
     *
     * Jerks at both ends (0, t) are not considered
     */
    std::vector<decimal_t> extrema_j(decimal_t t) const {
      std::vector<decimal_t> ts;
      if (c(0) != 0) {
        decimal_t t = -c(1) * 2 / c(0);
        if (t > 0 && t < t)
          ts.push_back(t);
      }
      return ts;
    }

  public:
    /// Coefficients
    Vec6f c;
};


/**
 * @brief Primitive class
 *
 * Contains \f$n\f$ 1D primitives corresponding to each axis individually.
 */
template <int Dim>
class Primitive {
 public:
  /**
   * @brief Empty constructor
   */
  Primitive() {}
  /**
   * @brief Construct from an initial state p and an input control u for a given duration t
   */
  Primitive(const Waypoint<Dim>& p, const Vecf<Dim>& u, decimal_t t)
    : t_(t), control_(p.control) {
    if (p.control == Control::SNP) {
      for (int i = 0; i < Dim; i++) {
        Vec4f vec;
        vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
        prs_[i] = Primitive1D(vec, u(i));
      }
    } else if (p.control == Control::JRK) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
    } else if (p.control == Control::ACC) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
    } else if (p.control == Control::VEL) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p.pos(i), u(i));
    } else
      printf("Null Primitive, check the control set-up of the Waypoint!\n");
  }

  /**
   * @brief Construct from an initial state p1 and a goal state p2 for a given duration t
   */
  Primitive(const Waypoint<Dim>& p1, const Waypoint<Dim>& p2, decimal_t t)
    : t_(t), control_(p1.control) {
    // Use jrk control
    if (p1.control == Control::JRK && p2.control == Control::JRK) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i),
                              p2.pos(i), p2.vel(i), p2.acc(i), t_);
    }
    // Use acc control
    else if (p1.control == Control::ACC && p2.control == Control::ACC) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), t_);
    }
    // Use vel control
    else if (p1.control == Control::VEL && p2.control == Control::VEL) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
    }
    // Null
    else
      printf("Null Primitive, check the control set-up of the Waypoint!\n");
  }

  /**
   * @brief Construct from given coefficients and duration\f$t\f$
   *
   * Note: flag `use_xxx` is not set in this constructor
   */
  Primitive(const vec_E<Vec6f>& cs, decimal_t t) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(cs[i]);
  }

  /**
   * @brief Return Waypoint at time \f$t\f$
   *
   * Note: flag `use_xxx` is set in the return value and it is equal
   * to the first given Waypoint
   */
  Waypoint<Dim> evaluate(decimal_t t) const {
    Waypoint<Dim> p(control_);
    for (int k = 0; k < Dim; k++) {
      p.pos(k) = prs_[k].p(t);
      p.vel(k) = prs_[k].v(t);
      p.acc(k) = prs_[k].a(t);
      p.jrk(k) = prs_[k].j(t);
    }
    return p;
  }

  /**
   * @brief Return duration \f$t\f$
   */
  decimal_t t() const { return t_; }

  /**
   * @brief Retrieve the 1D primitive
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  Primitive1D traj(int k) const { return prs_[k]; }

  /**
   * @brief Return max velocity along one axis
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  decimal_t max_vel(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_v(t_);
    decimal_t max_v = std::max(std::abs(prs_[k].v(0)),
                               std::abs(prs_[k].v(t_)));
    for (const auto &it : ts) {
      if (it > 0 && it < t_) {
        decimal_t v = std::abs(prs_[k].v(it));
        max_v = v > max_v ? v : max_v;
      }
    }
    return max_v;
  }

  /**
   * @brief Return max accleration along one axis
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  decimal_t max_acc(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_a(t_);
    decimal_t max_a = std::max(std::abs(prs_[k].a(0)),
                               std::abs(prs_[k].a(t_)));
    for (const auto &it : ts) {
      if (it > 0 && it < t_) {
        decimal_t a = std::abs(prs_[k].a(it));
        max_a = a > max_a ? a : max_a;
      }
    }
    return max_a;
  }

  /**
   * @brief Return max jerk along k-th dimension
   */
  decimal_t max_jrk(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_j(t_);
    decimal_t max_j = std::max(std::abs(prs_[k].j(0)),
                               std::abs(prs_[k].j(t_)));
    for (const auto &it : ts) {
      if (it > 0 && it < t_) {
        decimal_t j = std::abs(prs_[k].j(it));
        max_j = j > max_j ? j : max_j;
      }
    }
    return max_j;
  }

  /**
   * @brief Check if the max velocity magnitude is within the threshold
   * @param mv is the max threshold
   *
   * Use L1 norm for the maximum
   */
  bool validate_vel(decimal_t mv) const {
    // ignore negative threshold
    if (mv < 0)
      return true;
    // check if max vel is violating the constraint
    for (int i = 0; i < Dim; i++) {
      if (max_vel(i) > mv)
        return false;
    }
    return true;
  }

  /**
   * @brief Check if the max acceleration magnitude is within the threshold
   * @param ma is the max threshold
   *
   * Use L1 norm for the maximum
   */
  bool validate_acc(decimal_t ma) const {
    // ignore negative threshold
    if (ma < 0)
      return true;
    // check if max acc is violating the constraint
    for (int i = 0; i < Dim; i++) {
      if (max_acc(i) > ma)
        return false;
    }
    return true;
  }

  /**
   * @brief Check if the max jerk magnitude is within the threshold
   * @param mj is the max threshold
   *
   * Use L1 norm for the maximum
   */
  bool validate_jrk(decimal_t mj) const {
    // ignore negative threshold
    if (mj < 0)
      return true;
    // check if max jerk is violating the constraint
    for (int i = 0; i < Dim; i++) {
      if (max_jrk(i) > mj)
        return false;
    }
    return true;
  }


  /**
   * @brief Return total efforts of primitive for the given duration
   * @param i effort is defined as \f$i\f$-th derivative of polynomial
   *
   * Return J is the summation of efforts in all three dimensions and
   * \f$J(i) = \int_0^t |p^{i}(t)|^2dt\f$
   */
  decimal_t J(int i) const {
    decimal_t j = 0;
    for (const auto &pr : prs_)
      j += pr.J(t_, i);
    return j;
  }

  /**
   * @brief Sample N+1 Waypoints using uniformed time
   */
  vec_E<Waypoint<Dim>> sample(int N) const {
    vec_E<Waypoint<Dim>> ps(N+1);
    decimal_t dt = t_ / N;
    for (int i = 0; i <= N; i++)
      ps[i] = evaluate(i * dt);
    return ps;
  }

  /**
   * @brief Retrieve coefficients
   */
  vec_E<Vec6f> coeffs() const {
    vec_E<Vec6f> cs(Dim);
    for (int k = 0; k < Dim; k++)
      cs[k] = prs_[k].coeff();
    return cs;
  }



  /************************** Public members ************************/
  ///Duration
  decimal_t t_;
  ///Control
  int control_;
  ///By default, primitive class contains `Dim` 1D primitive
  std::array<Primitive1D, Dim> prs_;
};

///Primitive for 2D
typedef Primitive<2> Primitive2D;

///Primitive for 3D
typedef Primitive<3> Primitive3D;

/************************* Utils ******************************/
///Print all coefficients in primitive p
template <int Dim>
void print_coeffs(const Primitive<Dim>& p) {
  printf("coeffs: t = %f\n", p.t());
  for(int i = 0; i < Dim; i++)
    std::cout << i << ":     " << p.traj(i).coeff().transpose() << std::endl;
}

///Print max dynamic infomation in primitive p
template <int Dim>
void print_max(const Primitive<Dim>& p) {
  Vecf<Dim> max_v, max_a, max_j;
  for(int i = 0; i < Dim; i++) {
    max_v(i) = p.max_vel(i);
    max_a(i) = p.max_acc(i);
    max_j(i) = p.max_jrk(i);
  }
  std::cout << "max_vel: ", max_v.transpose() << std::endl;;
  std::cout << "max_acc: ", max_a.transpose() << std::endl;;
  std::cout << "max_jrk: ", max_j.transpose() << std::endl;;
}

///Sample N+1 ellipsoids along the primitive
template <int Dim>
vec_Ellipsoid sample_ellipsoids(const Primitive<Dim>& pr, const Vec3f& axe, int N) {
  vec_Ellipsoid Es(N+1);
  decimal_t dt = pr.t() / N;
  for(int i = 0; i <= N; i++) {
    const auto pt = pr.evaluate(i*dt);
    Es[i] = generate_ellipsoid<Dim>(axe, pt.pos, pt.acc);
  }

  return Es;
}

#endif
