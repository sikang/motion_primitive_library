/**
 *  * @file waypoint.h
 *   * @brief Waypoint classes
 *    */

#ifndef WAYPOINT_H
#define WAYPOINT_H
#include <iostream>
#include <motion_primitive_library/common/data_type.h>

/// Lookup table for control input
enum Control {
  VEL = 0b1000,
  ACC = 0b1100,
  JRK = 0b1110,
  SNP = 0b1111,
};

/**
 * @brief Waypoint base class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where the dimension \f$n\f$ can be either 2 or 3.
 */
template <int Dim>
struct Waypoint {
  /// Empty constructor
  Waypoint() {}
  /**
   * @brief Simple constructor
   * @param c control input
   */
  Waypoint(int c) : control(c) {}
  Vecf<Dim> pos; ///<position in \f$R^{Dim}\f$
  Vecf<Dim> vel; ///<velocity in \f$R^{Dim}\f$
  Vecf<Dim> acc; ///<acceleration in \f$R^{Dim}\f$
  Vecf<Dim> jrk; ///<jerk in \f$R^{Dim}\f$

  union {
    struct {
      bool use_jrk : 1;///<If true, jrk will be used in primitive generation
      bool use_acc : 1;///<If true, acc will be used in primitive generation
      bool use_vel : 1;///<If true, vel will be used in primitive generation
      bool use_pos : 1;///<If true, pos will be used in primitive generation
    };
    int control{0};
  };

  ///Print all attributes
  virtual void print(std::string str = "") const {
    if(!str.empty())
      std::cout << str << std::endl;
    std::cout << "pos: " << pos.transpose() << std::endl;
    std::cout << "vel: " << vel.transpose() << std::endl;
    std::cout << "acc: " << acc.transpose() << std::endl;
    std::cout << "jrk: " << jrk.transpose() << std::endl;
    std::cout << "use_pos: " << use_pos << std::endl;
    std::cout << "use_vel: " << use_vel << std::endl;
    std::cout << "use_acc: " << use_acc << std::endl;
    std::cout << "use_jrk: " << use_jrk << std::endl;
  }

  /**
   * @brief  Check if two waypoints are equivalent
   *
   * Compare the attribute if corresponding `use_xxx` of both Waypoints is true.
   */
  virtual bool operator==(const Waypoint<Dim>& n) const {
    /*
    return this->pos == n.pos &&
      this->vel == n.vel &&
      this->acc == n.acc &&
      this->jrk == n.jrk;
      */
    if((this->use_pos || n.use_pos) && this->pos != n.pos)
      return false;
    if((this->use_vel || n.use_vel) && this->vel != n.vel)
      return false;
    if((this->use_acc || n.use_acc) && this->acc != n.acc)
      return false;
    if((this->use_jrk || n.use_jrk) && this->jrk != n.jrk)
      return false;
    return true;
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
