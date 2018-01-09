/**
 * @file graph_search.h
 * @brief backend of graph search, implemetation of A* and Lifelong Planning A*
 */

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <array>                          // std::array
#include <primitive/trajectory.h>

namespace MPL
{
  ///Declare `env' class
  class env_base;

  ///Key for hashmap
  typedef std::string Key;
  
  ///Heap element comparison
  template <class state>
  struct compare_pair
  {
    bool operator()(const std::pair<double,std::shared_ptr<state>>& p1, 
                    const std::pair<double,std::shared_ptr<state>>& p2) const
    {
      if( (p1.first >= p2.first - 0.000001) && (p1.first <= p2.first + 0.000001) )
      {
        // if equal compare gvals
        return std::min(p1.second->g, p1.second->rhs) < std::min(p2.second->g, p2.second->rhs);
      }
      return p1.first > p2.first;
    }
  };  

  ///Define priority queue
  template <class state>
  using priorityQueue = boost::heap::d_ary_heap<std::pair<double,std::shared_ptr<state>>, boost::heap::mutable_<true>, boost::heap::arity<2>, boost::heap::compare< compare_pair<state> >>;
  
  ///Lattice of the graph in graph search
  struct State
  {
    /// hash key in the hashmap
    Key hashkey; // discrete coordinates of this node
    /// state
    Waypoint coord; 
    /// minimum arrival time
    double t;
    /// hashkey of successors
    std::vector<Key> succ_hashkey;
    /// action id of successors
    std::vector<int> succ_action_id;
    /// action cost of successors
    std::vector<double> succ_action_cost;
    /// hashkey of predecessors
    std::vector<Key> pred_hashkey;
    /// action id of predecessors
    std::vector<int> pred_action_id;
    /// action cost of predecessors
    std::vector<double> pred_action_cost;

    /// pointer to heap location
    typename priorityQueue<State>::handle_type heapkey;

    // plan data
    double g = std::numeric_limits<double>::infinity();
    double rhs = std::numeric_limits<double>::infinity();
    double h;
    bool iterationopened = false;
    bool iterationclosed = false;
    bool epq_opened = false;

    /// Simple constructor
    State( Key hashkey, const Waypoint& coord )
      : hashkey(hashkey), coord(coord)
    {}

  };

  ///Declare StatePtr
  using StatePtr = std::shared_ptr<State>;
  
  ///Define hashmap type
  using hashMap = std::unordered_map<Key, StatePtr>;

  ///State space
  struct StateSpace
  {
    ///Priority queue, open set
    priorityQueue<State> pq_;
    ///Hashmap, stores all the nodes
    hashMap hm_;
    ///Heuristic weight, default as 1
    double eps_;
    ///Execution time for each primitive
    double dt_;
    ///The best trajectory from previous plan
    std::vector<StatePtr> best_child_;
    ///Goal node, initialized as null by default
    StatePtr goalNode_ptr_;
    ///Internal flag to trigger goal reset 
    bool need_to_reset_goal_ = false;
    ///Maximum time of the valid trajectories
    double max_t_ = std::numeric_limits<double>::infinity();

    ///Simple constructor
    StateSpace(double eps = 1): eps_(eps){}

    /**
     * @brief Get the subtree
     * @param time_step indicates the root of the subtree (best_child_[time_step-1])
     * @param ENV pointer of `env_base' class
     * @param goal if changed, use new goal to calculate heuristic
     */
    void getSubStateSpace(int time_step, std::shared_ptr<env_base>& ENV, const Waypoint& goal);
    ///Increase the cost of actions 
    std::vector<Primitive> increaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV);
    ///Decrease the cost of actions
    std::vector<Primitive> decreaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV);
    ///Update the node in the graph
    void updateNode(StatePtr& currNode_ptr);

    ///Calculate the fval as min(rhs, g) + h
    double calculateKey(const StatePtr& node);

    ///Internal function to check if the graph is valid
    void checkValidation(const hashMap& hm);
  };

  
  /**
   * @brief GraphSearch class
   *
   * Implement A* and Lifelong Planning A*
   */
  class GraphSearch
  {
    public:
      /**
       * @brief Simple empty constructor
       *
       * @param verbose enable print out debug infos, default is set to False
       */
      GraphSearch(bool verbose = false) : verbose_(verbose) {};

      /**
       * @brief Astar graph search
       *
       * @param start_coord start state
       * @param start_key key of the start state 
       * @param ENV object of `env_base' class
       * @param ss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      double Astar(const Waypoint& start_coord, Key start_key,
          const std::shared_ptr<env_base>& ENV, std::shared_ptr<StateSpace> ss_ptr, 
          Trajectory& traj, int max_expand = -1, double max_t = 0);
      /**
       * @brief Lifelong Planning Astar graph search
       *
       * @param start_coord start state
       * @param start_key key of the start state 
       * @param ENV object of `env_base' class
       * @param ss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      double LPAstar(const Waypoint& start_coord, Key start_key, 
          const std::shared_ptr<env_base>& ENV, std::shared_ptr<StateSpace> ss_ptr, 
          Trajectory& traj, int max_expand = -1, double max_t = 0);
    private:
      ///Recover trajectory 
      Trajectory recoverTraj(StatePtr ptr, std::shared_ptr<StateSpace> ss_ptr, 
          const std::shared_ptr<env_base>& ENV, const Key& start_idx);
      ///Verbose flag
      bool verbose_ = false;
 
 };
}
#endif
