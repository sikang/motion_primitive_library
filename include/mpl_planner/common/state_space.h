/**
 * @file state_space.h
 * @brief state space class for graph search
 */
#ifndef MPL_STATE_SPACE_H
#define MPL_STATE_SPACE_H

#include <boost/heap/d_ary_heap.hpp> // boost::heap::d_ary_heap
#include <mpl_planner/common/env_base.h>
#include <unordered_map> // std::unordered_map

namespace MPL {
/// Heap element comparison
template <typename state> struct compare_pair {
  bool
  operator()(const std::pair<decimal_t, std::shared_ptr<state>> &p1,
             const std::pair<decimal_t, std::shared_ptr<state>> &p2) const {
    if (p1.first == p2.first) {
      // if equal compare gvals
      return std::min(p1.second->g, p1.second->rhs) >
             std::min(p2.second->g, p2.second->rhs);
    }
    return p1.first > p2.first;
  }
};

/// Define priority queue
template <typename state>
using priorityQueue =
    boost::heap::d_ary_heap<std::pair<decimal_t, std::shared_ptr<state>>,
                            boost::heap::mutable_<true>, boost::heap::arity<2>,
                            boost::heap::compare<compare_pair<state>>>;

/// Lattice of the graph in graph search
template <typename Coord> struct State {
  /// hash key in the hashmap
  Key hashkey; // discrete coordinates of this node
  /// state
  Coord coord;
  /// coordinates of successors
  vec_E<Coord> succ_coord;
  /// hashkey of successors
  std::vector<Key> succ_hashkey;
  /// action id of successors
  std::vector<int> succ_action_id;
  /// action cost of successors
  std::vector<decimal_t> succ_action_cost;
  /// hashkey of predecessors
  std::vector<Key> pred_hashkey;
  /// action id of predecessors
  std::vector<int> pred_action_id;
  /// action cost of predecessors
  std::vector<decimal_t> pred_action_cost;

  /// pointer to heap location
  typename priorityQueue<State<Coord>>::handle_type heapkey;

  // plan data
  /// start-to-state g value
  decimal_t g = std::numeric_limits<decimal_t>::infinity();
  /// rhs value based on g value
  decimal_t rhs = std::numeric_limits<decimal_t>::infinity();
  /// heuristic cost
  decimal_t h = std::numeric_limits<decimal_t>::infinity();
  /// label check if the state has been in the open set
  bool iterationopened = false;
  /// label check if the state has been closed
  bool iterationclosed = false;

  /// Simple constructor
  State(Key hashkey, const Coord& coord)
      : hashkey(hashkey), coord(coord) {}
};

/// Declare StatePtr
template <typename Coord> using StatePtr = std::shared_ptr<State<Coord>>;

/// Define hashmap type
template <typename Coord> using hashMap = std::unordered_map<Key, StatePtr<Coord>>;

/// State space
template <int Dim, typename Coord> struct StateSpace {
  /// Priority queue, open set
  priorityQueue<State<Coord>> pq_;
  /// Hashmap, stores all the nodes
  hashMap<Coord> hm_;
  /// Heuristic weight, default as 1
  decimal_t eps_;
  /// Execution time for each primitive
  decimal_t dt_;
  /// The best trajectory from previous plan
  vec_E<StatePtr<Coord>> best_child_;
  /// Maximum time of the valid trajectories
  decimal_t max_t_ = std::numeric_limits<decimal_t>::infinity();
  /// Number of expansion iteration
  int expand_iteration_ = 0;

  /// Simple constructor
  StateSpace(decimal_t eps = 1) : eps_(eps) {}

  /**
   * @brief Get the subtree
   * @param time_step indicates the root of the subtree (best_child_[time_step])
  */
  void getSubStateSpace(int time_step) {
    if (best_child_.empty())
      return;

    StatePtr<Coord> currNode_ptr = best_child_[time_step];
    currNode_ptr->pred_action_cost.clear();
    currNode_ptr->pred_action_id.clear();
    currNode_ptr->pred_hashkey.clear();
    currNode_ptr->coord.t = 0;

    for (auto &it : hm_) {
      it.second->g = std::numeric_limits<decimal_t>::infinity();
      it.second->rhs = std::numeric_limits<decimal_t>::infinity();
      it.second->pred_action_cost.clear();
      it.second->pred_action_id.clear();
      it.second->pred_hashkey.clear();
      it.second->coord.t = 0;
    }

    currNode_ptr->g = 0;
    currNode_ptr->rhs = 0;

    hashMap<Coord> new_hm;
    priorityQueue<State<Coord>> epq;
    currNode_ptr->heapkey =
      epq.push(std::make_pair(currNode_ptr->rhs, currNode_ptr));
    new_hm[currNode_ptr->hashkey] = currNode_ptr;

    while (!epq.empty()) {
      currNode_ptr = epq.top().second;
      epq.pop();

      if (currNode_ptr->coord.t == max_t_) {
        currNode_ptr->iterationclosed = false;
        currNode_ptr->g = std::numeric_limits<decimal_t>::infinity();
        currNode_ptr->succ_coord.clear();
        currNode_ptr->succ_hashkey.clear();
        currNode_ptr->succ_action_cost.clear();
        currNode_ptr->succ_action_id.clear();
      }

      for (unsigned int i = 0; i < currNode_ptr->succ_hashkey.size(); i++) {
        Key succ_key = currNode_ptr->succ_hashkey[i];

        StatePtr<Coord> &succNode_ptr = new_hm[succ_key];
        if (!succNode_ptr)
          succNode_ptr = hm_[succ_key];

        int id = -1;
        for (unsigned int i = 0; i < succNode_ptr->pred_hashkey.size(); i++) {
          if (succNode_ptr->pred_hashkey[i] == currNode_ptr->hashkey) {
            id = i;
            break;
          }
        }
        if (id == -1) {
          succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
          succNode_ptr->pred_action_cost.push_back(
            currNode_ptr->succ_action_cost[i]);
          succNode_ptr->pred_action_id.push_back(currNode_ptr->succ_action_id[i]);
        }

        decimal_t tentative_rhs =
          currNode_ptr->rhs + currNode_ptr->succ_action_cost[i];

        if (tentative_rhs < succNode_ptr->rhs) {
          succNode_ptr->coord.t = currNode_ptr->coord.t + dt_;
          succNode_ptr->rhs = tentative_rhs;
          if (succNode_ptr->iterationclosed) {
            succNode_ptr->g = succNode_ptr->rhs; // set g == rhs
            succNode_ptr->heapkey =
              epq.push(std::make_pair(succNode_ptr->rhs, succNode_ptr));
          }
        }
      }
    }

    hm_ = new_hm;
    pq_.clear();
    for (auto &it : hm_) {
      if (it.second->iterationopened && !it.second->iterationclosed)
        it.second->heapkey =
          pq_.push(std::make_pair(calculateKey(it.second), it.second));
    }
  }

  /// Increase the cost of actions
  vec_E<Primitive<Dim>> increaseCost(std::vector<std::pair<Key, int>> states,
                                     const std::shared_ptr<env_base<Dim>> &ENV) {
    vec_E<Primitive<Dim>> prs;
    for (const auto &affected_node : states) {
      // update edge
      StatePtr<Coord> &succNode_ptr = hm_[affected_node.first];
      const int i = affected_node.second; // i-th pred
      if (!std::isinf(succNode_ptr->pred_action_cost[i])) {
        succNode_ptr->pred_action_cost[i] =
          std::numeric_limits<decimal_t>::infinity();
        updateNode(succNode_ptr);

        Key parent_key = succNode_ptr->pred_hashkey[i];
        Primitive<Dim> pr;
        ENV->forward_action(hm_[parent_key]->coord,
                            succNode_ptr->pred_action_id[i], pr);
        prs.push_back(pr);

        int succ_act_id = hm_[affected_node.first]->pred_action_id[i];
        for (unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size();
             j++) {
          if (succ_act_id == hm_[parent_key]->succ_action_id[j]) {
            hm_[parent_key]->succ_action_cost[j] =
              std::numeric_limits<decimal_t>::infinity();
            break;
          }
        }
      }
    }

    return prs;
  }
  /// Decrease the cost of actions
  vec_E<Primitive<Dim>> decreaseCost(std::vector<std::pair<Key, int>> states,
                                     const std::shared_ptr<env_base<Dim>> &ENV) {
    vec_E<Primitive<Dim>> prs;
    for (const auto &affected_node : states) {
      StatePtr<Coord> &succNode_ptr = hm_[affected_node.first];
      const int i = affected_node.second;
      if (std::isinf(succNode_ptr->pred_action_cost[i])) {
        Key parent_key = succNode_ptr->pred_hashkey[i];
        Primitive<Dim> pr;
        ENV->forward_action(hm_[parent_key]->coord,
                            succNode_ptr->pred_action_id[i], pr);
        if (ENV->is_free(pr)) {
          prs.push_back(pr);
          succNode_ptr->pred_action_cost[i] = pr.J(pr.control()) + ENV->w_ * dt_;
          updateNode(succNode_ptr);
          int succ_act_id = succNode_ptr->pred_action_id[i];
          for (unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size();
               j++) {
            if (succ_act_id == hm_[parent_key]->succ_action_id[j]) {
              hm_[parent_key]->succ_action_cost[j] =
                succNode_ptr->pred_action_cost[i];
              break;
            }
          }
        }
      }
    }

    return prs;
  }
  /// Update the node in the graph
  void updateNode(StatePtr<Coord> &currNode_ptr) {
    // if currNode is not start, update its rhs
    // start rhs is assumed to be 0
    if (currNode_ptr->rhs != 0) {
      currNode_ptr->rhs = std::numeric_limits<decimal_t>::infinity();
      for (unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
        Key pred_key = currNode_ptr->pred_hashkey[i];
        if (currNode_ptr->rhs >
            hm_[pred_key]->g + currNode_ptr->pred_action_cost[i]) {
          currNode_ptr->rhs =
            hm_[pred_key]->g + currNode_ptr->pred_action_cost[i];
          currNode_ptr->coord.t = hm_[pred_key]->coord.t + dt_;
        }
      }
    }

    // if currNode is in openset, remove it
    if (currNode_ptr->iterationopened && !currNode_ptr->iterationclosed) {
      pq_.erase(currNode_ptr->heapkey);
      currNode_ptr->iterationclosed = true;
    }

    // if currNode's g value is not equal to its rhs, put it into openset
    // if(currNode_ptr->g != currNode_ptr->rhs || !currNode_ptr->iterationopened)
    // {
    if (currNode_ptr->g != currNode_ptr->rhs) {
      decimal_t fval = calculateKey(currNode_ptr);
      currNode_ptr->heapkey = pq_.push(std::make_pair(fval, currNode_ptr));
      currNode_ptr->iterationopened = true;
      currNode_ptr->iterationclosed = false;
    }
  }


  /// Calculate the fval as min(rhs, g) + h
  decimal_t calculateKey(const StatePtr<Coord> &node) {
    return std::min(node->g, node->rhs) + eps_ * node->h;
  }

  /// Check if the trajectory is blocked by new obstacle
  bool isBlocked() {
    for (const auto &ptr : best_child_) {
      if (ptr->g != ptr->rhs)
        return true;
    }
    return false;
  }
  /// Internal function to check if the graph is valid
  void checkValidation(const hashMap<Coord> &hm) {
    //****** Check if there is null element in succ graph
    for (const auto &it : hm) {
      if (!it.second)
        std::cout << "error!!! null element at key: " << it.first << std::endl;
    }

    /*
       for(const auto& it: pq_) {
       if(it.second->t >= 9)
       printf(ANSI_COLOR_RED "error!!!!!!!! t: %f, g: %f, rhs: %f, h: %f\n"
       ANSI_COLOR_RESET,
       it.second->t, it.second->g, it.second->rhs, it.second->h);
       }
       */

    //****** Check rhs and g value of close set
    printf("Check rhs and g value of closeset\n");
    int close_cnt = 0;
    for (const auto &it : hm) {
      if (it.second->iterationopened && it.second->iterationclosed) {
        printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
        close_cnt++;
      }
    }

    // Check rhs and g value of open set
    printf("Check rhs and g value of openset\n");
    int open_cnt = 0;
    for (const auto &it : hm) {
      if (it.second->iterationopened && !it.second->iterationclosed) {
        printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
        open_cnt++;
      }
    }

    // Check rhs and g value of null set
    printf("Check rhs and g value of nullset\n");
    int null_cnt = 0;
    for (const auto &it : hm) {
      if (!it.second->iterationopened) {
        printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
        null_cnt++;
      }
    }

    printf("hm: [%zu], open: [%d], closed: [%d], null: [%d]\n", hm.size(),
           open_cnt, close_cnt, null_cnt);

  }

  /**
   * @brief Update goal
   * @param ENV pointer of `env_base' class
   * @param goal if changed, use new goal to calculate heuristic
   */
  void updateGoal(std::shared_ptr<env_base<Dim>> &ENV,
                  const Coord &goal); // TODO: leave as null for now

  };
}

#endif
