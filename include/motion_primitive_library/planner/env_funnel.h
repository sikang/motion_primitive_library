/**
 * @file env_funnel.h
 * @biref environment for planning using point cloud
 */

#ifndef ENV_FUNNEL_H
#define ENV_FUNNEL_H
#include <motion_primitive_library/collision_checking/funnel_util.h>
#include <motion_primitive_library/planner/env_base.h>
#include <motion_primitive_library/primitive/primitive.h>

namespace MPL {

/**
 * @brief Point cloud environment
 */
class env_funnel : public env_base<3> {
protected:
  std::unique_ptr<FunnelUtil> map_util_;

public:
  /// Simple constructor
  env_funnel() {}
  /// Simple constructor
  env_funnel(const vec_Vec3f &obs, const Vec3f &ori, const Vec3f &dim,
             decimal_t kp = 5, decimal_t kv = 3, decimal_t v = 3) {
    map_util_.reset(new FunnelUtil(kp, kv, v));
    map_util_->setObstacles(obs);
    map_util_->set_region(ori, dim);
  }

  ~env_funnel() {}

  /// Check if a point is in free space
  bool is_free(const Vec3f &pt) const { return true; }

  /**
   * @brief Get successor
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_idx The array stores successors' Key
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control for each
   * successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory
   * Here we use Heuristic function and multiply with 2
   */
  void get_succ(const Waypoint3D &curr, vec_E<Waypoint3D> &succ,
                std::vector<Key> &succ_idx, std::vector<decimal_t> &succ_cost,
                std::vector<int> &action_idx) const {
    succ.clear();
    succ_idx.clear();
    succ_cost.clear();
    action_idx.clear();

    expanded_nodes_.push_back(curr.pos);
    // ws_.push_back(curr);
    for (int i = 0; i < (int)U_.size(); i++) {
      Primitive3D pr(curr, U_[i], dt_);
      Waypoint3D tn = pr.evaluate(dt_);
      if (pr.valid_vel(v_max_) && pr.valid_acc(a_max_)) {
        Key tn_key = state_to_idx(tn);
        bool valid = map_util_->isFree(pr, curr.jrk, tn.jrk);
        if (valid) {
          tn.use_pos = curr.use_pos;
          tn.use_vel = curr.use_vel;
          tn.use_acc = curr.use_acc;
          tn.use_jrk = curr.use_jrk;

          succ.push_back(tn);
          succ_idx.push_back(tn_key);
          succ_cost.push_back(pr.J(wi_) + w_ * dt_);
          action_idx.push_back(i);
        }
      }
    }
  }
};
}

#endif
