/**
 * @file waypoint.h
 * @brief Waypoint classes
 */

#ifndef MPL_WAYPOINT_H
#define MPL_WAYPOINT_H
#include <iostream>
#include <bitset>
#include <mpl_basis/control.h>
#include <mpl_basis/data_type.h>


/**
 * @brief Waypoint base class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where the dimension \f$n\f$ can be either 2 or 3.
 * Yaw is contained by default.
 * The anonymous union is used to set control flag.
 */
template <int Dim>
struct Waypoint {
  /// Empty constructor
  Waypoint() {}
  /**
   * @brief Simple constructor
   * @param c control input
   */
  Waypoint(Control::Control c) : control(c) {}
  Vecf<Dim> pos; ///<position in \f$R^{Dim}\f$
  Vecf<Dim> vel; ///<velocity in \f$R^{Dim}\f$
  Vecf<Dim> acc; ///<acceleration in \f$R^{Dim}\f$
  Vecf<Dim> jrk; ///<jerk in \f$R^{Dim}\f$
  decimal_t yaw; ///<yaw
  decimal_t t{0}; ///<time

  /**
   * @brief Control flag
   *
   * Anonymous union type that contains 5 bits as xxxxx,
   * each bit from left to right is assigned to `use_pos`, `use_vel`, `use_acc`,
   * `use_jrk` and `use_yaw`.
   */
  union {
    struct {
      bool use_yaw : 1;///<If true, yaw will be used in primitive generation
      bool use_jrk : 1;///<If true, jrk will be used in primitive generation
      bool use_acc : 1;///<If true, acc will be used in primitive generation
      bool use_vel : 1;///<If true, vel will be used in primitive generation
      bool use_pos : 1;///<If true, pos will be used in primitive generation
    };
    Control::Control control : 5;
  };

  ///Print all attributes
  virtual void print(std::string str = "") const {
    if(!str.empty())
      std::cout << str << std::endl;
    std::cout << "pos: " << pos.transpose() << std::endl;
    std::cout << "vel: " << vel.transpose() << std::endl;
    std::cout << "acc: " << acc.transpose() << std::endl;
    std::cout << "jrk: " << jrk.transpose() << std::endl;
    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "t: " << t << std::endl;
    if(control == Control::VEL)
      std::cout << "use vel!" << std::endl;
    else if(control == Control::ACC)
      std::cout << "use acc!" << std::endl;
    else if(control == Control::JRK)
      std::cout << "use jrk!" << std::endl;
    else if(control == Control::SNP)
      std::cout << "use snp!" << std::endl;
    else if(control == Control::VELxYAW)
      std::cout << "use vel & yaw!" << std::endl;
    else if(control == Control::ACCxYAW)
      std::cout << "use acc & yaw!" << std::endl;
    else if(control == Control::JRKxYAW)
      std::cout << "use jrk & yaw!" << std::endl;
    else if(control == Control::SNPxYAW)
      std::cout << "use snp & yaw!" << std::endl;
    else
      std::cout << "use null!" << std::endl;
  }

  /**
   * @brief  Check if two waypoints are equivalent
   *
   * Compare the attribute if corresponding `use_xxx` of both Waypoints is true.
   */
  virtual bool operator==(const Waypoint<Dim>& n) const {
    if((this->use_pos || n.use_pos) && this->pos != n.pos)
      return false;
    if((this->use_vel || n.use_vel) && this->vel != n.vel)
      return false;
    if((this->use_acc || n.use_acc) && this->acc != n.acc)
      return false;
    if((this->use_jrk || n.use_jrk) && this->jrk != n.jrk)
      return false;
    if((this->use_yaw || n.use_yaw) && this->yaw != n.yaw)
      return false;
    return true;
    //return this->t == n.t;
  }

  ///Check if two waypoints are not equivalent
  bool operator!=(const Waypoint<Dim>& n) {
    return !(*this == n);
  }
};

///Waypoint for 2D
typedef Waypoint<2> Waypoint2D;

///Waypoint for 3D
typedef Waypoint<3> Waypoint3D;

#endif
