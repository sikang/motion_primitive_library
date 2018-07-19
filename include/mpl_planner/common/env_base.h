/**
 * @file env_base.h
 * @brief environment base class
 */

#ifndef MPL_ENV_BASE_H
#define MPL_ENV_BASE_H

#include <mpl_basis/trajectory.h>

namespace MPL {

/**
 * @brief Key for node
 *
 * We use string as the Key for indexing, by default the Key refers to 'pos-vel-acc-...'
 */
typedef std::string Key;

/**
 * @brief Base environment class
 */
template <int Dim>
class env_base {
  public:
    ///Simple constructor
    env_base() {}

    ///Check if state hit the goal region, use L-1 norm
    virtual bool is_goal(const Waypoint<Dim>& state) const {
      if(state.t >= t_max_)
        return true;
      bool goaled = (state.pos - goal_node_.pos).template lpNorm<Eigen::Infinity>() <= tol_dis_;
      if(goaled && goal_node_.use_vel && tol_vel_ > 0)
        goaled = (state.vel - goal_node_.vel).template lpNorm<Eigen::Infinity>() <= tol_vel_;
      if(goaled && goal_node_.use_acc && tol_acc_ > 0)
        goaled = (state.acc - goal_node_.acc).template lpNorm<Eigen::Infinity>() <= tol_acc_;
      if(goaled && goal_node_.use_yaw && tol_yaw_ > 0)
        goaled = std::abs(state.yaw - goal_node_.yaw) <= tol_yaw_;
      return goaled;
    }

    /**
     * @brief Heuristic function
     * @param Waypoint current state coord
     * @param t current state time
     */
    virtual decimal_t get_heur(const Waypoint<Dim> &state) const {
      if (goal_node_ == state)
        return 0;
      size_t id = state.t / dt_;
      if(!prior_traj_.empty() && id < prior_traj_.size())
        return cal_heur(state, prior_traj_[id].first) + prior_traj_[id].second;
      else
        return cal_heur(state, goal_node_);
    }

    /// calculate the cost from state to goal
    virtual decimal_t cal_heur(const Waypoint<Dim>& state,
                               const Waypoint<Dim>& goal) const {
      if(heur_ignore_dynamics_) {
        if(v_max_ > 0) {
          return w_*(state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        }
        else
          return w_*(state.pos - goal.pos).template lpNorm<Eigen::Infinity>();
      }
      //return 0;
      //return w_*(state.pos - goal.pos).norm();
      if(state.control == Control::JRK && goal.control == Control::JRK) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;
        const Vecf<Dim> a0 = state.acc;
        const Vecf<Dim> a1 = goal.acc;
        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -9*a0.dot(a0)+6*a0.dot(a1)-9*a1.dot(a1);
        decimal_t d = -144*a0.dot(v0)-96*a0.dot(v1)+96*a1.dot(v0)+144*a1.dot(v1);
        decimal_t e = 360*(a0-a1).dot(dp)-576*v0.dot(v0)-1008*v0.dot(v1)-576*v1.dot(v1);
        decimal_t f = 2880*dp.dot(v0+v1);
        decimal_t g = -3600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
           continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost)
            min_cost = cost;
        }
        return min_cost;
      }

      else if(state.control == Control::JRK && goal.control == Control::ACC) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;
        const Vecf<Dim> a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -8*a0.dot(a0);
        decimal_t d = -112*a0.dot(v0)-48*a0.dot(v1);
        decimal_t e = 240*a0.dot(dp)-384*v0.dot(v0)-432*v0.dot(v1)-144*v1.dot(v1);
        decimal_t f = dp.dot(1600*v0+960*v1);
        decimal_t g = -1600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost)
            min_cost = cost;
          //printf("t: %f, cost: %f\n",t, cost);
        }
        return min_cost;
      }

      else if(state.control == Control::JRK && goal.control == Control::VEL) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -5*a0.dot(a0);
        decimal_t d = -40*a0.dot(v0);
        decimal_t e = 60*a0.dot(dp)-60*v0.dot(v0);
        decimal_t f = 160*dp.dot(v0);
        decimal_t g = -100*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost)
            min_cost = cost;
        }
        return min_cost;
      }

      else if(state.control == Control::ACC && goal.control == Control::ACC) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;

        decimal_t c1 = -36*dp.dot(dp);
        decimal_t c2 = 24*(v0+v1).dot(dp);
        decimal_t c3 = -4*(v0.dot(v0)+v0.dot(v1)+v1.dot(v1));
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
        }

        return cost;
      }

      else if(state.control == Control::ACC && goal.control == Control::VEL) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;

        decimal_t c1 = -9*dp.dot(dp);
        decimal_t c2 = 12*v0.dot(dp);
        decimal_t c3 = -3*v0.dot(v0);
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
        }

        return cost;
      }
      else if(state.control == Control::VEL && goal.control == Control::VEL)
        return (w_ + 1) * (state.pos - goal.pos).norm();
      else
        return w_*(state.pos - goal.pos).norm() / v_max_;
    }

    ///Replace the original cast function
    inline Veci<Dim> round(const Vecf<Dim>& vec, decimal_t res) const {
      Veci<Dim> vecI;
      for(int i = 0; i < Dim; i++)
        vecI(i) = std::round(vec(i) / res);
      return vecI;
    }

    ///Convert a vec to a string
    std::string to_string(const Veci<Dim>& vec) const {
      std::string str;
      for(int i = 0; i < Dim; i++)
        str += std::to_string(vec(i)) + "-";
      return str;
    }

    ///Genegrate Key from state
    virtual Key state_to_idx(const Waypoint<Dim>& state) const {
      const Veci<Dim> pi = round(state.pos, ds_);
      if(state.control == Control::VEL)
        return to_string(pi);
      else if(state.control == Control::ACC) {
        const Veci<Dim> vi = round(state.vel, dv_);
        return to_string(pi) + to_string(vi);
      }
      else if(state.control == Control::JRK) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        return to_string(pi) + to_string(vi) + to_string(ai);
      }
      else if(state.control == Control::SNP) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        const Veci<Dim> ji = round(state.jrk, dj_);
        return to_string(pi) + to_string(vi) + to_string(ai) + to_string(ji);
      }
      else if(state.control == Control::VELxYAW) {
        int yawi = std::round(state.yaw/dyaw_);
        return to_string(pi) +
        std::to_string(yawi);
      }
       else if(state.control == Control::ACCxYAW) {
        const Veci<Dim> vi = round(state.vel, dv_);
        int yawi = std::round(state.yaw/dyaw_);
        return to_string(pi) + to_string(vi) +
          std::to_string(yawi);
      }
       else if(state.control == Control::JRKxYAW) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        int yawi = std::round(state.yaw/dyaw_);
        return to_string(pi) + to_string(vi) + to_string(ai) +
          std::to_string(yawi);
      }
       else if(state.control == Control::SNPxYAW) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        const Veci<Dim> ji = round(state.jrk, dj_);
        int yawi = std::round(state.yaw/dyaw_);
        return to_string(pi) + to_string(vi) + to_string(ai) + to_string(ji) +
          std::to_string(yawi);
      }
      else
        return "";
    }

    ///Recover trajectory
    void forward_action( const Waypoint<Dim>& curr,
                        int action_id, Primitive<Dim>& pr) const {
      pr = Primitive<Dim>(curr, U_[action_id], dt_);
    }

    ///Set control input
    void set_u(const vec_E<VecDf>& U) {
      U_ = U;
    }

    ///Set max vel in each axis
    void set_v_max(decimal_t v) {
      v_max_ = v;
    }

    ///Set max acc in each axis
    void set_a_max(decimal_t a) {
      a_max_ = a;
    }

    ///Set max acc in each axis
    void set_j_max(decimal_t j) {
      j_max_ = j;
    }

    ///Set max acc in each axis
    void set_yaw_max(decimal_t yaw) {
      yaw_max_ = yaw;
    }

    ///Set prior trajectory
    virtual void set_prior_trajectory(const Trajectory<Dim>& traj) {
      prior_traj_.clear();
      decimal_t total_time = traj.getTotalTime();
      for(decimal_t t = 0; t < total_time; t += dt_) {
        prior_traj_.push_back(std::make_pair(traj.evaluate(t),
                                             w_*(total_time - t)));
      }
    }

    ///Set dt for primitive
    void set_dt(decimal_t dt) {
      dt_ = dt;
    }

    ///Set ds
    void set_ds(decimal_t ds) {
      ds_ = ds;
    }

    ///Set dv
    void set_dv(decimal_t dv) {
      dv_ = dv;
    }

    ///Set da
    void set_da(decimal_t da) {
      da_ = da;
    }

    ///Set dj
    void set_dj(decimal_t dj) {
      dj_ = dj;
    }

    ///Set dyaw
    void set_dyaw(decimal_t dyaw) {
      dyaw_ = dyaw;
    }

    ///Set distance tolerance for goal region
    void set_tol_dis(decimal_t dis) {
      tol_dis_ = dis;
    }

    ///Set velocity tolerance for goal region
    void set_tol_vel(decimal_t vel) {
      tol_vel_ = vel;
    }

    ///Set acceleration tolerance for goal region
    void set_tol_acc(decimal_t acc) {
      tol_acc_ = acc;
    }

    ///Set acceleration tolerance for goal region
    void set_tol_yaw(decimal_t yaw) {
      tol_yaw_ = yaw;
    }

    ///set weight for cost in time, usually no need to change
    void set_w(decimal_t w) {
      w_ = w;
    }

    ///set weight for cost in yaw, usually no need to change
    void set_wyaw(decimal_t wyaw) {
      wyaw_ = wyaw;
    }

    ///set weight for cost in time, usually no need to change
    virtual void set_potential_weight(decimal_t w) {
    }

    ///set weight for cost in time, usually no need to change
    virtual void set_gradient_weight(decimal_t w) {
    }

    ///set weight for cost in time, usually no need to change
    virtual void set_potential_map(const std::vector<int8_t>& map) {
    }

    ///set weight for cost in time, usually no need to change
    virtual void set_gradient_map(const vec_E<Vecf<Dim>>& map) {
    }

    ///Set max time
    void set_t_max(int t) {
      t_max_ = t;
    }

    ///Set goal state
    bool set_goal(const Waypoint<Dim>& state) {
      if(prior_traj_.empty())
        goal_node_ = state;
      return prior_traj_.empty();
    }

    ///Set valid region
    void set_valid_region(const std::vector<bool>& valid_region) {
      valid_region_ = valid_region;
    }

    ///Set heur_ignore_dynamics
    void set_heur_ignore_dynamics(bool ignore) {
      heur_ignore_dynamics_ = ignore;
    }

    ///Print out params
    virtual void info() {
      printf(ANSI_COLOR_YELLOW "\n");
      printf("++++++++++++++++++++ env_base ++++++++++++++++++\n");
      printf("+                  w: %.2f               +\n", w_);
      printf("+               wyaw: %.2f               +\n", wyaw_);
      printf("+                 dt: %.2f               +\n", dt_);
      printf("+                 ds: %.2f               +\n", ds_);
      printf("+                 dv: %.2f               +\n", dv_);
      printf("+                 da: %.2f               +\n", da_);
      printf("+                 dj: %.2f               +\n", dj_);
      printf("+              t_max: %.2f               +\n", t_max_);
      printf("+              v_max: %.2f               +\n", v_max_);
      printf("+              a_max: %.2f               +\n", a_max_);
      printf("+              j_max: %.2f               +\n", j_max_);
      printf("+            yaw_max: %.2f               +\n", yaw_max_);
      printf("+              U num: %zu                +\n", U_.size());
      printf("+            tol_dis: %.2f               +\n", tol_dis_);
      printf("+            tol_vel: %.2f               +\n", tol_vel_);
      printf("+            tol_acc: %.2f               +\n", tol_acc_);
      printf("+            tol_yaw: %.2f               +\n", tol_yaw_);
      printf("+heur_ignore_dynamics: %d                 +\n", heur_ignore_dynamics_);
      printf("++++++++++++++++++++ env_base ++++++++++++++++++\n");
      printf(ANSI_COLOR_RESET "\n");
    }

    ///Check if a point is in free space
    virtual bool is_free(const Vecf<Dim>& pt) const {
      printf("Used Null is_free() for pt\n");
      return true;
    }

    ///Check if a primitive is in free space
    virtual bool is_free(const Primitive<Dim>& pr) const {
      printf("Used Null is_free() for pr\n");
      return true;
    }

    ///Retrieve dt
    decimal_t get_dt() const {
      return dt_;
    }

    /**
     * @brief Get successor
     * @param curr The node to expand
     * @param succ The array stores valid successors
     * @param succ_idx The array stores successors' Key
     * @param succ_cost The array stores cost along valid edges
     * @param action_idx The array stores corresponding idx of control for each
     * successor
     */
    virtual void get_succ(const Waypoint<Dim> &curr, vec_E<Waypoint<Dim>> &succ,
                          std::vector<Key> &succ_idx,
                          std::vector<decimal_t> &succ_cost,
                          std::vector<int> &action_idx) const {
      printf("Used Null get_succ()\n");
      succ.push_back(curr);
      succ_idx.push_back(state_to_idx(curr));
      succ_cost.push_back(0);
      action_idx.push_back(0);
    }

    /// Get the valid region
    std::vector<bool> get_valid_region() const { return valid_region_; }

    /// if enabled, ignore dynamics when calculate heuristic
    bool heur_ignore_dynamics_{true};
    /// weight of time cost
    decimal_t w_{10};
    /// weight of yaw
    decimal_t wyaw_{1};
    ///tolerance of position for goal region, 0.5 is the default
    decimal_t tol_dis_{0.5};
    ///tolerance of velocity for goal region, 0 means no tolerance
    decimal_t tol_vel_{0.0};
    ///tolerance of acceleration for goal region, 0 means no tolerance
    decimal_t tol_acc_{0.0};
    ///tolerance of yaw for goal region, 0 means no tolerance
    decimal_t tol_yaw_{0.0};
    ///max velocity
    decimal_t v_max_{-1};
    ///max acceleration
    decimal_t a_max_{-1};
    ///max jerk
    decimal_t j_max_{-1};
    ///max yaw
    decimal_t yaw_max_{-1};
    ///max time
    decimal_t t_max_{std::numeric_limits<decimal_t>::infinity()};
    ///duration of primitive
    decimal_t dt_{1.0};
    ///grid size in position
    decimal_t ds_{0.01};
    ///grid size in velocity
    decimal_t dv_{0.1};
    ///grid size in acceleration
    decimal_t da_{0.1};
    ///grid size in jerk
    decimal_t dj_{0.1};
    ///grid size in yaw
    decimal_t dyaw_{0.1};
    ///expanded nodes
    mutable vec_Vecf<Dim> expanded_nodes_;
    ///Array of constant control input
    vec_E<VecDf> U_;
    ///Goal node
    Waypoint<Dim> goal_node_;
    ///Prior trajectory
    vec_E<std::pair<Waypoint<Dim>, decimal_t>> prior_traj_;
    ///Valid region
    std::vector<bool> valid_region_;
};
}


#endif
