/**
 * @file state_space.h
 * @brief state space class for graph search
 */

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map> // std::unordered_map
#include <motion_primitive_library/primitive/primitive.h> 
#include <motion_primitive_library/planner/env_base.h> 

namespace MPL
{
  ///Key for hashmap
  typedef std::string Key;

  ///Heap element comparison
  template <class state>
  struct compare_pair
  {
    bool operator()(const std::pair<double,std::shared_ptr<state>>& p1, 
                    const std::pair<double,std::shared_ptr<state>>& p2) const
    {
      if( p1.first == p2.first )
      {
        // if equal compare gvals
        return std::min(p1.second->g, p1.second->rhs) > std::min(p2.second->g, p2.second->rhs);
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
    /// coordinates of successors
    std::vector<Waypoint> succ_coord;
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
    /// start-to-state g value
    double g = std::numeric_limits<double>::infinity();
    /// rhs value based on g value
    double rhs = std::numeric_limits<double>::infinity();
    /// heuristic cost
    double h = std::numeric_limits<double>::infinity();
    /// label check if the state has been in the open set
    bool iterationopened = false;
    /// label check if the state has been closed
    bool iterationclosed = false;

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
    ///Maximum time of the valid trajectories
    double max_t_ = std::numeric_limits<double>::infinity();
    ///Number of expansion iteration
    int expand_iteration_ = 0;

    ///Simple constructor
    StateSpace(double eps = 1): eps_(eps){}

    /**
     * @brief Get the subtree
     * @param time_step indicates the root of the subtree (best_child_[time_step])
    */
    void getSubStateSpace(int time_step);
    
    /**
     * @brief Update goal
     * @param ENV pointer of `env_base' class
     * @param goal if changed, use new goal to calculate heuristic
     */
    void updateGoal(std::shared_ptr<env_base>& ENV, const Waypoint& goal);
    ///Increase the cost of actions 
    std::vector<Primitive> increaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV);
    ///Decrease the cost of actions
    std::vector<Primitive> decreaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV);
    ///Update the node in the graph
    void updateNode(StatePtr& currNode_ptr);

    ///Calculate the fval as min(rhs, g) + h
    double calculateKey(const StatePtr& node);

    ///Check if the trajectory is blocked by new obstacle
    bool isBlocked();
    ///Internal function to check if the graph is valid
    void checkValidation(const hashMap& hm);
  };



}
