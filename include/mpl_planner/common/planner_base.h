/**
 * @file planner_base.h
 * @brief base class for motion planning
 *
 * Base classes for planning
 */

#ifndef MPL_PLANNER_BASE_H
#define MPL_PLANNER_BASE_H

#include <mpl_planner/common/env_base.h>
#include <mpl_planner/common/graph_search.h>

/**
 * @brief Motion planning base util class
 */

namespace MPL {

template <int Dim, typename Coord> class PlannerBase {
public:
  /// Simple constructor
  PlannerBase(bool verbose = false) : planner_verbose_(verbose) {}
  /// Check if the planner has been initialized
  bool initialized() {
    return !(ss_ptr_ == nullptr);
  }

  /// Get optimal trajectory
  Trajectory<Dim> getTraj() const {
    return traj_;
  }
  /// Get expanded collision free primitives
  vec_E<Primitive<Dim>> getValidPrimitives() const {
    vec_E<Primitive<Dim>> prs;
    for (const auto &it : ss_ptr_->hm_) {
      if (it.second && !it.second->pred_coord.empty()) {
        for (unsigned int i = 0; i < it.second->pred_coord.size(); i++) {
          Coord key = it.second->pred_coord[i];
          // if(!ss_ptr_->hm_[key] || std::isinf(it.second->pred_action_cost[i]))
          if (std::isinf(it.second->pred_action_cost[i]))
            continue;
          Primitive<Dim> pr;
          ENV_->forward_action(ss_ptr_->hm_[key]->coord,
                               it.second->pred_action_id[i], pr);
          prs.push_back(pr);
        }
      }
    }

    if (planner_verbose_)
      printf("number of states in hm: %zu, number of valid prs: %zu\n",
             ss_ptr_->hm_.size(), prs.size());

    return prs;
  }
  /// Get expanded primitives
  vec_E<Primitive<Dim>> getAllPrimitives() const {
    vec_E<Primitive<Dim>> prs;
    for (const auto &it : ss_ptr_->hm_) {
      if (it.second && !it.second->pred_coord.empty()) {
        for (unsigned int i = 0; i < it.second->pred_coord.size(); i++) {
          Coord key = it.second->pred_coord[i];
          Primitive<Dim> pr;
          ENV_->forward_action(key, it.second->pred_action_id[i], pr);
          prs.push_back(pr);
        }
      }
    }

    if (planner_verbose_)
      printf("getAllPrimitives number of states in hm: %zu, number of prs: %zu\n",
             ss_ptr_->hm_.size(), prs.size());

    return prs;
  }
  /// Get points in open set
  vec_Vecf<Dim> getOpenSet() const {
    vec_Vecf<Dim> ps;
    for (const auto &it : ss_ptr_->pq_)
      ps.push_back(it.second->coord.pos);
    return ps;
  }
  /// Get points in close set
  vec_Vecf<Dim> getCloseSet() const {
    vec_Vecf<Dim> ps;
    for (const auto &it : ss_ptr_->hm_) {
      if (it.second && it.second->iterationclosed)
        ps.push_back(it.second->coord.pos);
    }
    return ps;
  }
  /// Get points neither in open nor close set
  vec_Vecf<Dim> getNullSet() const {
    vec_Vecf<Dim> ps;
    for (const auto &it : ss_ptr_->hm_) {
      if (it.second && !it.second->iterationopened)
        ps.push_back(it.second->coord.pos);
    }
    return ps;
  }
  /// Get expanded points, for A* it should be the same as the close set
  vec_Vecf<Dim> getExpandedNodes() const {
    return ENV_->expanded_nodes_;
  }
  /// Get number of expanded nodes
  int getExpandedNum() const {
    return ss_ptr_->expand_iteration_;
  }
  /**
   * @brief Prune state space
   * @param time_step set the root of state space to be the waypoint on the best
   * trajectory at best_child_[time_step]
   */
  void getSubStateSpace(int time_step) {
    ss_ptr_->getSubStateSpace(time_step);
  }
  /// Get trajectory total cost
  decimal_t getTrajCost() const {
    return traj_cost_;
  }
  /// Check tree validation
  void checkValidation() {
    ss_ptr_->checkValidation(ss_ptr_->hm_);
  }
  /// Reset state space
  void reset() {
    ss_ptr_ = nullptr;
    traj_ = Trajectory<Dim>();
  }
  /// Set max vel in each axis
  void setLPAstar(bool use_lpastar) {
    use_lpastar_ = use_lpastar;
    if (use_lpastar_)
      printf("[PlannerBase] use Lifelong Planning A*\n");
    else
      printf("[PlannerBase] use normal A*\n");
  }
  /// Set max vel in each axis
  void setVmax(decimal_t v) {
    ENV_->set_v_max(v);
    if (planner_verbose_)
      printf("[PlannerBase] set v_max: %f\n", v);
  }
  /// Set max acc in each axis
  void setAmax(decimal_t a) {
    ENV_->set_a_max(a);
    if (planner_verbose_)
      printf("[PlannerBase] set a_max: %f\n", a);
  }
  /// Set max jerk in each axis
  void setJmax(decimal_t j) {
    ENV_->set_j_max(j);
    if (planner_verbose_)
      printf("[PlannerBase] set j_max: %f\n", j);
  }
  /// Set max jerk in each axis
  void setYawmax(decimal_t yaw) {
    ENV_->set_yaw_max(yaw);
    if (planner_verbose_)
      printf("[PlannerBase] set yaw_max: %f\n", yaw);
  }
  /// Set max time step to explore
  void setTmax(decimal_t t) {
    ENV_->set_t_max(t);
    if (planner_verbose_)
      printf("[PlannerBase] set max time: %f\n", t);
  }
  /// Set dt for each primitive
  void setDt(decimal_t dt) {
    ENV_->set_dt(dt);
    if (planner_verbose_)
      printf("[PlannerBase] set dt: %f\n", dt);
  }
  /// Set weight for cost in time
  void setW(decimal_t w) {
    ENV_->set_w(w);
    if (planner_verbose_)
      printf("[PlannerBase] set w: %f\n", w);
  }
  /// Set greedy searching param
  void setEpsilon(decimal_t eps) {
    epsilon_ = eps;
    if (planner_verbose_)
      printf("[PlannerBase] set epsilon: %f\n", epsilon_);
  }
  /// Calculate heuristic using dynamics
  void setHeurIgnoreDynamics(bool ignore) {
    ENV_->set_heur_ignore_dynamics(ignore);
    if (planner_verbose_)
      printf("[PlannerBase] set heur_ignore_dynamics: %d\n", ignore);
  }

  /// Set max number of expansion
  void setMaxNum(int num) {
    max_num_ = num;
    if (planner_verbose_)
      printf("[PlannerBase] set max num: %d\n", max_num_);
  }
  /// Set U
  void setU(const vec_E<VecDf> &U) {
    ENV_->set_u(U);
  }
  /// Set prior trajectory
  void setPriorTrajectory(const Trajectory<Dim> &traj) {
    ENV_->set_prior_trajectory(traj);
    if (planner_verbose_)
      printf("[PlannerBase] set prior trajectory\n");
  }
  /// Set tolerance in geometric and dynamic spaces
  void setTol(decimal_t tol_dis, decimal_t tol_vel = 0, decimal_t tol_acc = 0) {
    ENV_->set_tol_dis(tol_dis);
    ENV_->set_tol_vel(tol_vel);
    ENV_->set_tol_acc(tol_acc);
    if (planner_verbose_) {
      printf("[PlannerBase] set tol_dis: %f\n", tol_dis);
      printf("[PlannerBase] set tol_vel: %f\n", tol_vel);
      printf("[PlannerBase] set tol_acc: %f\n", tol_acc);
    }
  }
  /**
   * @brief Planning thread
   * @param start start waypoint
   * @param goal goal waypoint
   *
   * The goal waypoint is the center of the goal region, the planner cannot find
   * the trajectory hits the exact goal state due to discretization
   */
  bool plan(const Coord &start, const Coord &goal) {
    if (planner_verbose_) {
      start.print("Start:");
      goal.print("Goal:");

      ENV_->info();
    }


    if (!ENV_->is_free(start.pos)) {
      printf(ANSI_COLOR_RED "[PlannerBase] start is not free!"
             ANSI_COLOR_RESET "\n");
      return false;
    }


    std::unique_ptr<MPL::GraphSearch<Dim, Coord>> planner_ptr(
      new MPL::GraphSearch<Dim, Coord>(planner_verbose_));

    // If use A*, reset the state space
    if (!use_lpastar_)
      ss_ptr_.reset(new MPL::StateSpace<Dim, Coord>(epsilon_));
    else {
      // If use LPA*, reset the state space only at the initial planning
      if (!initialized()) {
        if (planner_verbose_)
          printf(ANSI_COLOR_CYAN
                 "[MPPlanner] reset planner state space!" ANSI_COLOR_RESET "\n");
        ss_ptr_.reset(new MPL::StateSpace<Dim, Coord>(epsilon_));
      }
    }

    ENV_->set_goal(goal);

    ENV_->expanded_nodes_.clear();

    ss_ptr_->dt_ = ENV_->get_dt();
    if (use_lpastar_)
      traj_cost_ = planner_ptr->LPAstar(start, ENV_, ss_ptr_, traj_, max_num_);
    else
      traj_cost_ = planner_ptr->Astar(start, ENV_, ss_ptr_, traj_, max_num_);

    if (std::isinf(traj_cost_)) {
      if (planner_verbose_)
        printf(ANSI_COLOR_RED "[MPPlanner] Cannot find a traj!" ANSI_COLOR_RESET
               "\n");
      return false;
    }

    return true;
  }
protected:
  /// Env class
  std::shared_ptr<MPL::env_base<Dim>> ENV_;
  /// Planner workspace
  std::shared_ptr<MPL::StateSpace<Dim, Coord>> ss_ptr_;
  /// Optimal trajectory
  Trajectory<Dim> traj_;
  /// Total cost of final trajectory
  decimal_t traj_cost_;
  /// Greedy searching parameter
  decimal_t epsilon_ = 1.0;
  /// Maxmum number of expansion, -1 means no limitation
  int max_num_ = -1;
  /// Enable LPAstar for planning
  bool use_lpastar_ = false;
  /// Enabled to display debug message
  bool planner_verbose_;
};
}

#endif
