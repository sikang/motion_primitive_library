/**
 * @file state_space.h
 * @brief state space class for graph search
 */
#ifndef MPL_STATE_SPACE_H
#define MPL_STATE_SPACE_H

#include <boost/heap/d_ary_heap.hpp> // boost::heap::d_ary_heap
#include <mpl_planner/common/env_base.h>
#include <boost/unordered_map.hpp> // std::unordered_map

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
  /// state
  Coord coord;
  /// coordinates of successors
  vec_E<Coord> succ_coord;
  /// action id of successors
  std::vector<int> succ_action_id;
  /// action cost of successors
  std::vector<decimal_t> succ_action_cost;
  /// coordinates of predecessors
  vec_E<Coord> pred_coord;
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
  State(const Coord& coord) : coord(coord) {}
};

/// Declare StatePtr
template <typename Coord> using StatePtr = std::shared_ptr<State<Coord>>;

/// Define hashmap type
template <typename Coord> using hashMap = boost::unordered_map<Coord, StatePtr<Coord>, boost::hash<Coord>>;

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
    const auto init_t = currNode_ptr->coord.t;

    currNode_ptr->pred_action_cost.clear();
    currNode_ptr->pred_action_id.clear();
    currNode_ptr->pred_coord.clear();

    for (auto &it : hm_) {
      it.second->g = std::numeric_limits<decimal_t>::infinity();
      it.second->rhs = std::numeric_limits<decimal_t>::infinity();
      it.second->pred_action_cost.clear();
      it.second->pred_action_id.clear();
      it.second->pred_coord.clear();
    }

    currNode_ptr->g = 0;
    currNode_ptr->rhs = 0;

    hashMap<Coord> new_hm;
    priorityQueue<State<Coord>> epq;
    currNode_ptr->heapkey =
      epq.push(std::make_pair(currNode_ptr->rhs, currNode_ptr));
    new_hm[currNode_ptr->coord] = currNode_ptr;

    while (!epq.empty()) {
      currNode_ptr = epq.top().second;
      epq.pop();

      for (unsigned int i = 0; i < currNode_ptr->succ_coord.size(); i++) {
        Coord succ_coord = currNode_ptr->succ_coord[i];

        StatePtr<Coord> &succNode_ptr = new_hm[succ_coord];
        if (!succNode_ptr)
          succNode_ptr = hm_[succ_coord];

        int id = -1;
        for (size_t i = 0; i < succNode_ptr->pred_coord.size(); i++) {
          if (succNode_ptr->pred_coord[i] == currNode_ptr->coord) {
            id = i;
            break;
          }
        }
        if (id == -1) {
          succNode_ptr->pred_coord.push_back(currNode_ptr->coord);
          succNode_ptr->pred_action_cost.push_back(
            currNode_ptr->succ_action_cost[i]);
          succNode_ptr->pred_action_id.push_back(currNode_ptr->succ_action_id[i]);
        }

        decimal_t tentative_rhs =
          currNode_ptr->rhs + currNode_ptr->succ_action_cost[i];

        if (tentative_rhs < succNode_ptr->rhs) {
          succNode_ptr->rhs = tentative_rhs;
          if (succNode_ptr->iterationclosed) {
            succNode_ptr->g = succNode_ptr->rhs; // set g == rhs
            succNode_ptr->heapkey =
              epq.push(std::make_pair(succNode_ptr->rhs, succNode_ptr));
          }
        }
      }
    }

    hm_.clear();
    pq_.clear();
    for (auto &it : new_hm) {
      it.second->coord.t -= init_t;
      for(auto& itt: it.second->pred_coord)
        itt.t -= init_t;
      for(auto& itt: it.second->succ_coord)
        itt.t -= init_t;
      hm_[it.second->coord] = it.second;
      if (it.second->iterationopened && !it.second->iterationclosed)
        it.second->heapkey =
          pq_.push(std::make_pair(calculateKey(it.second), it.second));
    }
    //printf("new_hm: %zu, hm_: %zu\n", new_hm.size(), hm_.size());

  }

  /// Increase the cost of actions
  void increaseCost(std::vector<std::pair<Coord, int>> states) {
    for (const auto &affected_node : states) {
      // update edge
      StatePtr<Coord> &succNode_ptr = hm_[affected_node.first];
      const int i = affected_node.second; // i-th pred
      if (!std::isinf(succNode_ptr->pred_action_cost[i])) {
        succNode_ptr->pred_action_cost[i] =
          std::numeric_limits<decimal_t>::infinity();
        updateNode(succNode_ptr);

        Coord parent_key = succNode_ptr->pred_coord[i];

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
  }
  /// Decrease the cost of actions
  void decreaseCost(std::vector<std::pair<Coord, int>> states,
                    const std::shared_ptr<env_base<Dim>> &ENV) {
    for (const auto &affected_node : states) {
      StatePtr<Coord> &succNode_ptr = hm_[affected_node.first];
      const int i = affected_node.second;
      if (std::isinf(succNode_ptr->pred_action_cost[i])) {
        Coord parent_key = succNode_ptr->pred_coord[i];
        Primitive<Dim> pr;
        ENV->forward_action(parent_key, succNode_ptr->pred_action_id[i], pr);
        if (ENV->is_free(pr)) {
          succNode_ptr->pred_action_cost[i] = ENV->calculate_intrinsic_cost(pr);
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
  }
  /// Update the node in the graph
  void updateNode(StatePtr<Coord> &currNode_ptr) {
    // if currNode is not start, update its rhs
    // start rhs is assumed to be 0
    if (currNode_ptr->rhs != 0) {
      currNode_ptr->rhs = std::numeric_limits<decimal_t>::infinity();
      for (unsigned int i = 0; i < currNode_ptr->pred_coord.size(); i++) {
        Coord pred_key = currNode_ptr->pred_coord[i];
        if (currNode_ptr->rhs > hm_[pred_key]->g + currNode_ptr->pred_action_cost[i]) {
          currNode_ptr->rhs =
            hm_[pred_key]->g + currNode_ptr->pred_action_cost[i];
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

  /// Internal function to check if the graph is valid
  void checkValidation(const hashMap<Coord> &hm) {
    //****** Check if there is null element in succ graph
    for (const auto &it : hm) {
      if (!it.second)
        std::cout << "error!!! null element at key: " << it.first << std::endl;
    }

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
