/**
 * @file env_map.h
 * @biref environment for planning in voxel map
 */

#ifndef MPL_ENV_MAP_H
#define MPL_ENV_MAP_H
#include <mpl_collision/map_util.h>
#include <mpl_planner/common/env_base.h>
#include <unordered_map>
#include <memory>

namespace MPL {
/**
 * @brief Voxel map environment
 */
template <int Dim> class env_map : public env_base<Dim> {
public:
  /// Constructor with map util as input
  env_map(std::shared_ptr<MapUtil<Dim>> map_util) : map_util_(map_util) {}

  /// Check if state hit the goal region, use L-1 norm
  bool is_goal(const Waypoint<Dim> &state) const {
    bool goaled = (state.pos - this->goal_node_.pos).template lpNorm<Eigen::Infinity>() <= this->tol_dis_;
    if (goaled && this->goal_node_.use_vel && this->tol_vel_ > 0)
      goaled = (state.vel - this->goal_node_.vel).template lpNorm<Eigen::Infinity>() <= this->tol_vel_;
    if (goaled && this->goal_node_.use_acc && this->tol_acc_ > 0)
      goaled = (state.acc - this->goal_node_.acc).template lpNorm<Eigen::Infinity>() <= this->tol_acc_;
    if(goaled && this->goal_node_.use_yaw && this->tol_yaw_ > 0)
      goaled = std::abs(state.yaw- this->goal_node_.yaw) <= this->tol_yaw_;
    if (goaled) {
      auto pns = map_util_->rayTrace(state.pos, this->goal_node_.pos);
      for (const auto &it : pns) {
        if (map_util_->isOccupied(it))
          return false;
      }
    }
    return goaled;
  }

  /// Check if a point is in free space
  bool is_free(const Vecf<Dim> &pt) const {
    const auto pn = map_util_->floatToInt(pt);
    return map_util_->isFree(pn);
  }

  /**
   * @brief Check if the primitive is in free space
   *
   * Sample points along the primitive, and check each point for collision; the
   * number of sampling is calculated based on the maximum velocity and
   * resolution of the map.
   */
  bool is_free(const Primitive<Dim> &pr) const {
    decimal_t max_v = 0;
    if (Dim == 2)
      max_v = std::max(pr.max_vel(0), pr.max_vel(1));
    else if (Dim == 3)
      max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
    int n = std::ceil(max_v * pr.t() / map_util_->getRes());
    vec_E<Waypoint<Dim>> pts = pr.sample(n);
    for (const auto &pt : pts) {
      Veci<Dim> pn = map_util_->floatToInt(pt.pos);
      if (map_util_->isOccupied(pn) || map_util_->isOutside(pn))
        return false;
      if(!this->valid_region_.empty() && !this->valid_region_[map_util_->getIndex(pn)])
        return false;
    }

    return true;
  }

 /**
   * @brief Accumulate the cost along the primitive
   *
   * Sample points along the primitive, and sum up the cost of each point;
   * number of sampling is calculated based on the maximum velocity and
   * resolution of the map.
   *
   * If the potential map has been set, it also uses the potential values;
   * otherwise, the accumulated value will be zero for collision-free primitive
   * and infinity for others.
   */

  decimal_t traverse_primitive(const Primitive<Dim> &pr) const {
    decimal_t max_v = 0;
    for (int i = 0; i < Dim; i++) {
      if (pr.max_vel(i) > max_v)
        max_v = pr.max_vel(i);
    }
    int n = std::ceil(max_v * pr.t() / this->map_util_->getRes());
    decimal_t c = 0;
    vec_E<Waypoint<Dim>> pts = pr.sample(n);
    for (const auto &pt : pts) {
      const Veci<Dim> pn = this->map_util_->floatToInt(pt.pos);
      const int idx = this->map_util_->getIndex(pn);
      if (this->map_util_->isOutside(pn) ||
          (!this->valid_region_.empty() &&
          !this->valid_region_[idx]))
        return std::numeric_limits<decimal_t>::infinity();
      /*
      decimal_t v_value = gradient_map_[idx].dot(pt.vel);
      if(v_value > 0)
        v_value = 0;
      v_value = -v_value;
      */
      if(!potential_map_.empty()) {
        if(potential_map_[idx] < 100 && potential_map_[idx] > 0)
          c += potential_weight_ * potential_map_[idx] +
            gradient_weight_ * pt.vel.norm();
        else if(potential_map_[idx] >= 100)
          return std::numeric_limits<decimal_t>::infinity();
      }
      else if (map_util_->isOccupied(pn))
          return std::numeric_limits<decimal_t>::infinity();
    }

    return c;
  }



  /**
   * @brief Get successor
   *
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_idx The array stores successors' Key
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control input
   * for each successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory.
   * Only return the primitive satisfies valid dynamic constriants (include the
   * one hits obstacles).
   */
  void get_succ(const Waypoint<Dim> &curr, vec_E<Waypoint<Dim>> &succ,
                std::vector<Key> &succ_idx, std::vector<decimal_t> &succ_cost,
                std::vector<int> &action_idx) const {
    succ.clear();
    succ_idx.clear();
    succ_cost.clear();
    action_idx.clear();

    //this->expanded_nodes_.push_back(curr.pos);

    /*
    const Veci<Dim> pn = map_util_->floatToInt(curr.pos);
    if (map_util_->isOutside(pn))
      return;
      */

    for (unsigned int i = 0; i < this->U_.size(); i++) {
      Primitive<Dim> pr(curr, this->U_[i], this->dt_);
      Waypoint<Dim> tn = pr.evaluate(this->dt_);
      if (tn == curr)
        continue;
      if (pr.control() == Control::ACC && !pr.validate_vel(this->v_max_))
        continue;
      if (pr.control() == Control::JRK &&
          (!pr.validate_vel(this->v_max_) || !pr.validate_acc(this->a_max_)))
        continue;
      if (pr.control() == Control::SNP &&
          (!pr.validate_vel(this->v_max_) || !pr.validate_acc(this->a_max_) ||
           !pr.validate_jrk(this->j_max_)))
        continue;
      if (pr.control() == Control::VELxYAW && !pr.validate_yaw(this->yaw_max_))
        continue;
      if (pr.control() == Control::ACCxYAW &&
          (!pr.validate_yaw(this->yaw_max_) || !pr.validate_vel(this->v_max_)))
        continue;
      if (pr.control() == Control::JRKxYAW &&
          (!pr.validate_yaw(this->yaw_max_) || !pr.validate_vel(this->v_max_) ||
           !pr.validate_acc(this->a_max_)))
        continue;
      if (pr.control() == Control::SNPxYAW &&
          (!pr.validate_yaw(this->yaw_max_) || !pr.validate_vel(this->v_max_) ||
           !pr.validate_acc(this->a_max_) || !pr.validate_jrk(this->j_max_)))
        continue;

      succ.push_back(tn);
      succ_idx.push_back(this->state_to_idx(tn));
      //std::cout << succ_idx.back() << std::endl;
      decimal_t cost = curr.pos == tn.pos ? 0 : traverse_primitive(pr);
      if (!std::isinf(cost))
        cost += pr.J(pr.control()) + this->wyaw_ * pr.Jyaw() + this->w_ * this->dt_;

      succ_cost.push_back(cost);
      action_idx.push_back(i);
    }
  }

  /// Set gradient map
  void set_gradient_map(const vec_E<Vecf<Dim>>& map) {
    gradient_map_ = map;
  }

  /// Set gradient weight
  void set_gradient_weight(decimal_t w) {
    gradient_weight_ = w;
  }

  /// Set potential map
  void set_potential_map(const std::vector<int8_t>& map) {
    potential_map_ = map;
  }

  /// Set potential weight
  void set_potential_weight(decimal_t w) {
    potential_weight_ = w;
  }

  ///Print out params
  void info() {
      printf("++++++++++++++++++++ env_map ++++++++++++++++++\n");
      printf("+                  w: %.2f               +\n", this->w_);
      printf("+               wyaw: %.2f               +\n", this->wyaw_);
      printf("+                 dt: %.2f               +\n", this->dt_);
      printf("+                 ds: %.2f               +\n", this->ds_);
      printf("+                 dv: %.2f               +\n", this->dv_);
      printf("+                 da: %.2f               +\n", this->da_);
      printf("+                 dj: %.2f               +\n", this->dj_);
      printf("+              v_max: %.2f               +\n", this->v_max_);
      printf("+              a_max: %.2f               +\n", this->a_max_);
      printf("+              j_max: %.2f               +\n", this->j_max_);
      printf("+            yaw_max: %.2f               +\n", this->yaw_max_);
      printf("+              U num: %zu                +\n", this->U_.size());
      printf("+            tol_dis: %.2f               +\n", this->tol_dis_);
      printf("+            tol_vel: %.2f               +\n", this->tol_vel_);
      printf("+            tol_acc: %.2f               +\n", this->tol_acc_);
      printf("+            tol_yaw: %.2f               +\n", this->tol_yaw_);
      printf("+              alpha: %d                 +\n", this->alpha_);
      printf("+heur_ignore_dynamics: %d                 +\n", this->heur_ignore_dynamics_);
      if(!potential_map_.empty())
        printf("+    potential_weight: %.2f                 +\n", potential_weight_);
      if(!gradient_map_.empty())
        printf("+     gradient_weight: %.2f                 +\n", gradient_weight_);
      printf("++++++++++++++++++++ env_map ++++++++++++++++++\n");
  }


protected:
  /// Collision checking util
  std::shared_ptr<MapUtil<Dim>> map_util_;
  /// Potential map, optional
  std::vector<int8_t> potential_map_;
  /// Gradient map, optional
  vec_E<Vecf<Dim>> gradient_map_;
  /// Weight of potential value
  decimal_t potential_weight_{0.1};
  /// Weight of gradient value
  decimal_t gradient_weight_{0.1};
};
}

#endif
