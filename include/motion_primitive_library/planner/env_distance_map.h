/**
 * @file env_distance_map.h
 * @biref environment for planning in voxel map
 */

#ifndef ENV_DISTANCE_MAP_H
#define ENV_DISTANCE_MAP_H
#include <motion_primitive_library/planner/env_map.h>

namespace MPL {
/**
 * @brief Voxel map environment
 */
template <int Dim> class env_distance_map : public env_map<Dim> {
public:
  /// Constructor with map util as input
  env_distance_map(std::shared_ptr<MapUtil<Dim>> map_util) : env_map<Dim>(map_util) {}

  void set_gradient_map(const vec_E<Vecf<Dim>>& map) {
    gradient_map_ = map;
  }

  void set_gradient_weight(decimal_t w) {
    gradient_weight_ = w;
  }

  void set_potential_map(const std::vector<int8_t>& map) {
    potential_map_ = map;
  }

  void set_potential_weight(decimal_t w) {
    potential_weight_ = w;
  }

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
      decimal_t v_value = gradient_map_[idx].dot(pt.vel);
      if(v_value > 0)
        v_value = 0;
      v_value = -v_value;
      const auto p_value = potential_map_[idx];
      if(p_value < 100)
        c += potential_weight_ * p_value + gradient_weight_ * v_value;
      else
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
   * @param action_idx The array stores corresponding idx of control for each
   * successor
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

    const Veci<Dim> pn = this->map_util_->floatToInt(curr.pos);
    if (this->map_util_->isOutside(pn))
      return;

    for (unsigned int i = 0; i < this->U_.size(); i++) {
      Primitive<Dim> pr(curr, this->U_[i], this->dt_);
      Waypoint<Dim> tn = pr.evaluate(this->dt_);
      if (tn == curr)
        continue;
      if (pr.valid_vel(this->v_max_) && pr.valid_acc(this->a_max_) &&
          pr.valid_jrk(this->j_max_)) {
        tn.use_pos = curr.use_pos;
        tn.use_vel = curr.use_vel;
        tn.use_acc = curr.use_acc;
        tn.use_jrk = curr.use_jrk;

        succ.push_back(tn);
        succ_idx.push_back(this->state_to_idx(tn));
        decimal_t cost = traverse_primitive(pr);
        if(!std::isinf(cost))
          cost += pr.J(this->wi_) + this->w_ * this->dt_;
        succ_cost.push_back(cost);
        action_idx.push_back(i);
      }
    }
  }

protected:
  std::vector<int8_t> potential_map_;
  vec_E<Vecf<Dim>> gradient_map_;
  decimal_t potential_weight_ = 0.1;
  decimal_t gradient_weight_ = 0.1;

};
}

#endif
