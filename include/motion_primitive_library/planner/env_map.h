/**
 * @file env_map.h
 * @biref environment for planning in voxel map
 */

#ifndef ENV_MAP_H
#define ENV_MAP_H
#include <motion_primitive_library/collision_checking/map_util.h>
#include <motion_primitive_library/planner/env_base.h>
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
    bool goaled =
        (state.pos - this->goal_node_.pos).template lpNorm<Eigen::Infinity>() <=
        this->tol_dis;
    if (goaled && this->goal_node_.use_vel && this->tol_vel > 0)
      goaled = (state.vel - this->goal_node_.vel)
                   .template lpNorm<Eigen::Infinity>() <= this->tol_vel;
    if (goaled && this->goal_node_.use_acc && this->tol_acc > 0)
      goaled = (state.acc - this->goal_node_.acc)
                   .template lpNorm<Eigen::Infinity>() <= this->tol_acc;
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
      if (pr.validate_vel(this->v_max_) &&
          pr.validate_acc(this->a_max_) &&
          pr.validate_jrk(this->j_max_)) {

        succ.push_back(tn);
        succ_idx.push_back(this->state_to_idx(tn));
        /*
        decimal_t cost = is_free(pr)
                             ? pr.J(this->wi_) + this->w_ * this->dt_
                             : std::numeric_limits<decimal_t>::infinity();
                             */
        decimal_t cost = traverse_primitive(pr);
        if(!std::isinf(cost))
          cost += pr.J(this->wi_) + this->w_ * this->dt_;

        succ_cost.push_back(cost);
        action_idx.push_back(i);
      }
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
