#include <planner/graph_search.h>
#include <planner/env_base.h>
#include <primitive/primitive_util.h>

using namespace MPL;

/**************************** Recover Trajectory ***************************/

Trajectory GraphSearch::recoverTraj(StatePtr currNode_ptr, std::shared_ptr<StateSpace> ss_ptr, const std::shared_ptr<env_base>& ENV, const Key& start_key) {
  // Recover trajectory
  ss_ptr->best_child_.clear();
  std::vector<Primitive> prs;
  while( !currNode_ptr->pred_hashkey.empty())
  {
    if(verbose_)
      std::cout << "t: " << currNode_ptr->t << " --> " << currNode_ptr->t - ss_ptr->dt_ << std::endl;
    ss_ptr->best_child_.push_back(currNode_ptr);
    int min_id = -1;
    double min_rhs = std::numeric_limits<double>::infinity();
    double min_g = std::numeric_limits<double>::infinity();
    for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
      Key key = currNode_ptr->pred_hashkey[i];
      //std::cout << "action id: " << currNode_ptr->pred_action_id[i] << " parent g: " << ss_ptr->hm_[key]->g << " action cost: " << currNode_ptr->pred_action_cost[i] << " parent key: " <<key << std::endl;
      if(min_rhs > ss_ptr->hm_[key]->g + currNode_ptr->pred_action_cost[i]) {
        min_rhs = ss_ptr->hm_[key]->g + currNode_ptr->pred_action_cost[i];
        min_g = ss_ptr->hm_[key]->g;
        min_id = i;
      }
      else if(!std::isinf(currNode_ptr->pred_action_cost[i]) &&
          min_rhs == ss_ptr->hm_[key]->g + currNode_ptr->pred_action_cost[i]) {
        if(min_g < ss_ptr->hm_[key]->g) {
          min_g = ss_ptr->hm_[key]->g;
          min_id = i;
        }
      }
    }

    if(min_id >= 0) {
      Key key = currNode_ptr->pred_hashkey[min_id];
      int action_idx = currNode_ptr->pred_action_id[min_id];
      currNode_ptr = ss_ptr->hm_[key];
      Primitive pr;
      ENV->forward_action( currNode_ptr->coord, action_idx, pr );
      prs.push_back(pr);
      if(verbose_) {
        //std::cout << "parent t: " << currNode_ptr->t << " key: " << key << std::endl;
        printf("Take action id: %d,  action cost: J: [%f, %f, %f]\n", action_idx, pr.J(0), pr.J(1), pr.J(2));
        //print_coeffs(pr);
      }
    }
    else {
      if(verbose_) {
        printf(ANSI_COLOR_RED "Trace back failure, the number of predecessors is %zu: \n", currNode_ptr->pred_hashkey.size());
        for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
          Key key = currNode_ptr->pred_hashkey[i];
          printf("i: %d, gvalue: %f, cost: %f\n" ANSI_COLOR_RESET, i, ss_ptr->hm_[key]->g, currNode_ptr->pred_action_cost[i]);
        }
      }

      break;
    }

    if(currNode_ptr->hashkey == start_key)
      break;
  }

  std::reverse(prs.begin(), prs.end());
  std::reverse(ss_ptr->best_child_.begin(), ss_ptr->best_child_.end());
  return Trajectory(prs);
}



/********************************* Astar  **************************************/

double GraphSearch::Astar(const Waypoint& start_coord, Key start_key,
    const std::shared_ptr<env_base>& ENV, std::shared_ptr<StateSpace> ss_ptr, 
    Trajectory& traj, int max_expand, double max_t)
{
  // Check if done
  if( ENV->is_goal(start_coord) )
    return 0;
  
  // Initialize start node
  StatePtr currNode_ptr = ss_ptr->hm_[start_key];
  if(ss_ptr->pq_.empty()) {
    if(verbose_) 
      printf(ANSI_COLOR_GREEN "Start from new node!\n" ANSI_COLOR_RESET);
    currNode_ptr = std::make_shared<State>(State(start_key, start_coord));
    currNode_ptr->t = 0;
    currNode_ptr->g = 0;
    currNode_ptr->h = ENV->get_heur(start_coord, currNode_ptr->t);
    double fval = currNode_ptr->g + ss_ptr->eps_ * currNode_ptr->h;
    currNode_ptr->heapkey = ss_ptr->pq_.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    ss_ptr->hm_[start_key] = currNode_ptr;
  }

  int expand_iteration = 0;
  while(true)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = ss_ptr->pq_.top().second;     
    ss_ptr->pq_.pop(); 
    currNode_ptr->iterationclosed = true; // Add to closed list

    // Get successors
    std::vector<Waypoint> succ_coord;
    std::vector<MPL::Key> succ_key;
    std::vector<double> succ_cost;
    std::vector<int> succ_act_id;

    ENV->get_succ( currNode_ptr->coord, succ_coord, succ_key, succ_cost, succ_act_id);

    // Process successors (satisfy dynamic constraints but might hit obstacles)
    for( unsigned s = 0; s < succ_coord.size(); ++s )
    {
      // If the primitive is occupied, skip
      if(std::isinf(succ_cost[s]))
        continue;
 
      // Get child
      StatePtr& succNode_ptr = ss_ptr->hm_[ succ_key[s] ];
      if( !succNode_ptr )
      {
        succNode_ptr = std::make_shared<State>(State(succ_key[s], succ_coord[s]) );
        succNode_ptr->t = currNode_ptr->t + ENV->dt_;
        succNode_ptr->h = ENV->get_heur( succNode_ptr->coord, succNode_ptr->t); 
        /*
         * Comment this block if build multiple connected graph
        succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        succNode_ptr->pred_action_id.push_back(succ_act_id[s]);
        succNode_ptr->pred_action_cost.push_back(succ_cost[s]);
        */
      }

      /**
       * Comment following if build single connected graph
       */
      int id = -1;
      for(unsigned int i = 0; i < succNode_ptr->pred_hashkey.size(); i++) {
        if(succNode_ptr->pred_hashkey[i] == currNode_ptr->hashkey) {
          id = i;
          break;
        }
      }
      if(id == -1) {
        succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        succNode_ptr->pred_action_cost.push_back(succ_cost[s]);
        succNode_ptr->pred_action_id.push_back(succ_act_id[s]);
      }
      else {
        succNode_ptr->pred_action_cost[id] = succ_cost[s];
        succNode_ptr->pred_action_id[id] = succ_act_id[s];
      }
      /**
       * 
       */


      // see if we can improve the value of successor
      // taking into account the cost of action
      double tentative_gval = currNode_ptr->g + succ_cost[s];

      if( tentative_gval < succNode_ptr->g )
      {
        /**
         * Comment this block if build multiple connected graph
	succNode_ptr->pred_hashkey.front() = currNode_ptr->hashkey;  // Assign new parent
	succNode_ptr->pred_action_id.front() = succ_act_id[s];
	succNode_ptr->pred_action_cost.front() = succ_cost[s];
        */
	succNode_ptr->t = currNode_ptr->t + ENV->dt_;
	succNode_ptr->g = tentative_gval;    // Update gval

	double fval = succNode_ptr->g + (ss_ptr->eps_) * succNode_ptr->h;

	// if currently in OPEN, update
	if( succNode_ptr->iterationopened && !succNode_ptr->iterationclosed)
	{
          if(verbose_) {
            if((*succNode_ptr->heapkey).first < fval) {
              std::cout << "UPDATE fval(old) = " << (*succNode_ptr->heapkey).first << std::endl;
              std::cout << "UPDATE fval = " << fval << std::endl;
            }
          }

	  (*succNode_ptr->heapkey).first = fval;     // update heap element
	  //ss_ptr->pq.update(succNode_ptr->heapkey);
	  ss_ptr->pq_.increase( succNode_ptr->heapkey );       // update heap
	}
	// if currently in CLOSED
	else if( succNode_ptr->iterationopened && succNode_ptr->iterationclosed)
	{
	  printf(ANSI_COLOR_RED "ASTAR ERROR!\n" ANSI_COLOR_RESET);
	  // succNode_ptr->heapkey = ss_ptr->pq_.push( std::make_pair(fval,succNode_ptr) );
	  // succNode_ptr->iterationopened = ss_ptr->searchiteration;
	  // succNode_ptr->iterationclosed = 0;
	}
	else // new node, add to heap
	{
	  //std::cout << "ADD fval = " << fval << std::endl;
	  succNode_ptr->heapkey = ss_ptr->pq_.push( std::make_pair(fval, succNode_ptr));
	  succNode_ptr->iterationopened = true;
	}
      }
    } 

    // If goal reached, abort!
    if(ENV->is_goal(currNode_ptr->coord)) 
      break;

    // If maximum time reached, abort!
    if(max_t > 0 && currNode_ptr->t >= max_t && !std::isinf(currNode_ptr->g)) {
      if(verbose_) 
        printf(ANSI_COLOR_GREEN "MaxExpandTime [%f] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_t);
      break;
    }


    // If maximum expansion reached, abort!
    if(max_expand > 0 && expand_iteration >= max_expand) {
      printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
      return std::numeric_limits<double>::infinity();
    }

    // If pq is empty, abort!
    if( ss_ptr->pq_.empty()) {
      printf(ANSI_COLOR_RED "Priority queue is empty!!!!!!\n\n" ANSI_COLOR_RESET);
      return std::numeric_limits<double>::infinity();
    }
  }

  if(verbose_) {
    double fval = ss_ptr->calculateKey(currNode_ptr);
    printf(ANSI_COLOR_GREEN "goalNode fval: %f, g: %f!\n" ANSI_COLOR_RESET, fval, currNode_ptr->g);
    printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expand_iteration);
  }

  if(ENV->is_goal(currNode_ptr->coord)) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Reached Goal !!!!!!\n\n" ANSI_COLOR_RESET);
  }
 

  traj = recoverTraj(currNode_ptr, ss_ptr, ENV, start_key);
  return currNode_ptr->g;
}

/********************************* LPAstar  **************************************/

double GraphSearch::LPAstar(const Waypoint& start_coord, Key start_key, 
    const std::shared_ptr<env_base>& ENV, std::shared_ptr<StateSpace> ss_ptr, 
    Trajectory& traj, int max_expand, double max_t)
{
  // Check if done
  if( ENV->is_goal(start_coord) ) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Start is inside goal region!\n" ANSI_COLOR_RESET);
    return 0;
  }

  ss_ptr->max_t_ = max_t > 0 ? max_t : std::numeric_limits<double>::infinity();
  // Initialize start node
  StatePtr currNode_ptr = ss_ptr->hm_[start_key];
  if(!currNode_ptr) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Start from new node!\n" ANSI_COLOR_RESET);
    currNode_ptr = std::make_shared<State>(State(start_key, start_coord));
    currNode_ptr->t = 0;
    currNode_ptr->g = std::numeric_limits<double>::infinity();
    currNode_ptr->rhs = 0;
    currNode_ptr->h = ENV->get_heur(start_coord, currNode_ptr->t);
    currNode_ptr->heapkey = ss_ptr->pq_.push( std::make_pair(ss_ptr->calculateKey(currNode_ptr), currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
    ss_ptr->hm_[start_key] = currNode_ptr;
  }
  // Initialize null goal node
  if(ss_ptr->need_to_reset_goal_) {
    if(verbose_)
      printf(ANSI_COLOR_GREEN "Reset goal!\n" ANSI_COLOR_RESET);
    ss_ptr->goalNode_ptr_ = nullptr;
  }

  StatePtr& goalNode_ptr = ss_ptr->goalNode_ptr_;
  if(!goalNode_ptr) {
    goalNode_ptr = std::make_shared<State>(State(Key(), Waypoint()));
    ss_ptr->need_to_reset_goal_ = false;
  }


  int expand_iteration = 0;
  while(ss_ptr->pq_.top().first < std::min(goalNode_ptr->g, goalNode_ptr->rhs) || goalNode_ptr->rhs != goalNode_ptr->g)
  {
    expand_iteration++;
    // Get element with smallest cost
    currNode_ptr = ss_ptr->pq_.top().second;     
    ss_ptr->pq_.pop(); 
    currNode_ptr->iterationclosed = true; // Add to closed list

    if(currNode_ptr->g > currNode_ptr->rhs) 
      currNode_ptr->g = currNode_ptr->rhs;
    else {
      currNode_ptr->g = std::numeric_limits<double>::infinity();
      ss_ptr->updateNode(currNode_ptr);
    }


    // Get successors
    std::vector<Waypoint> succ_coord;
    std::vector<MPL::Key> succ_key;
    std::vector<double> succ_cost;
    std::vector<int> succ_act_id;

    ENV->get_succ( currNode_ptr->coord, succ_coord, succ_key, succ_cost, succ_act_id);
    currNode_ptr->succ_hashkey.resize(succ_coord.size());
    currNode_ptr->succ_action_id.resize(succ_coord.size());
    currNode_ptr->succ_action_cost.resize(succ_coord.size());

    // Process successors
    for( unsigned s = 0; s < succ_coord.size(); ++s )
    {
      // Get child
      StatePtr& succNode_ptr = ss_ptr->hm_[ succ_key[s] ];
      if( !(succNode_ptr) ) {
        succNode_ptr = std::make_shared<State>(State(succ_key[s], succ_coord[s]) );
        succNode_ptr->h = ENV->get_heur( succNode_ptr->coord, currNode_ptr->t + ENV->dt_);   // compute heuristic        
      }

      // store the hashkey
      currNode_ptr->succ_hashkey[s] = succ_key[s];
      currNode_ptr->succ_action_id[s] = succ_act_id[s];
      currNode_ptr->succ_action_cost[s] = succ_cost[s];

      int id = -1;
      for(unsigned int i = 0; i < succNode_ptr->pred_hashkey.size(); i++) {
        if(succNode_ptr->pred_hashkey[i] == currNode_ptr->hashkey) {
          id = i;
          break;
        }
      }
      if(id == -1) {
        succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        succNode_ptr->pred_action_cost.push_back(succ_cost[s]);
        succNode_ptr->pred_action_id.push_back(succ_act_id[s]);
      }
      else {
        succNode_ptr->pred_action_cost[id] = succ_cost[s];
        succNode_ptr->pred_action_id[id] = succ_act_id[s];
      }

      ss_ptr->updateNode(succNode_ptr);
    }


    // If goal reached, terminate!
    if(ENV->is_goal(currNode_ptr->coord)) 
      goalNode_ptr = currNode_ptr;
    // If maximum time reached, terminate!
    else if(max_t > 0 && currNode_ptr->t >= max_t) {
      if(verbose_)
        printf(ANSI_COLOR_GREEN "MaxExpandTime [%f] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_t);
      goalNode_ptr = currNode_ptr;
    }

    // If maximum expansion reached, abort!
    if(max_expand > 0 && expand_iteration >= max_expand) {
      if(verbose_)
        printf(ANSI_COLOR_RED "MaxExpandStep [%d] Reached!!!!!!\n\n" ANSI_COLOR_RESET, max_expand);
      return std::numeric_limits<double>::infinity();
    }

    // If pq is empty, abort!
    if( ss_ptr->pq_.empty()) {
      if(verbose_)
        printf(ANSI_COLOR_RED "Priority queue is empty!!!!!!\n\n" ANSI_COLOR_RESET);
      return std::numeric_limits<double>::infinity();
    }
  }

 
  // Report value of goal
  if(verbose_) {
    printf(ANSI_COLOR_GREEN "goalNode fval: %f, g: %f, rhs: %f!\n" ANSI_COLOR_RESET, 
        ss_ptr->calculateKey(goalNode_ptr), goalNode_ptr->g, goalNode_ptr->rhs);
   // printf(ANSI_COLOR_GREEN "currNode fval: %f, g: %f, rhs: %f!\n" ANSI_COLOR_RESET, 
   //     ss_ptr->calculateKey(currNode_ptr), currNode_ptr->g, currNode_ptr->rhs);
    printf(ANSI_COLOR_GREEN "Expand [%d] nodes!\n" ANSI_COLOR_RESET, expand_iteration);
  }

  // If no expansion, recover from the goal directly
  if(expand_iteration == 0) 
    currNode_ptr = goalNode_ptr;

  // Check if the goal is reached, if reached, set the flag to be True
  if(ENV->is_goal(currNode_ptr->coord)) {
    if(verbose_) {
      //currNode_ptr->coord.print();
      //ENV->goal_node_.print();
      printf(ANSI_COLOR_GREEN "Reached Goal !!!!!!\n\n" ANSI_COLOR_RESET);
    }
  }
  // Recover trajectory
  traj = recoverTraj(currNode_ptr, ss_ptr, ENV, start_key);

  return currNode_ptr->g;
}

void StateSpace::checkValidation(const hashMap& hm) {
  // Check if there is null element in succ graph
  for(const auto& it: hm) {
    if(!it.second) {
      std::cout << "error!!! null element at key: " << it.first << std::endl;
    }
  }

  /*
  for(const auto& it: pq_) {
    if(it.second->t >= 9)
      printf(ANSI_COLOR_RED "error!!!!!!!! t: %f, g: %f, rhs: %f, h: %f\n" ANSI_COLOR_RESET,
          it.second->t, it.second->g, it.second->rhs, it.second->h);
  }
  */

  // Check rhs and g value of close set
  printf("Check rhs and g value of closeset\n");
  int close_cnt = 0;
  for(const auto& it: hm) {
    if(it.second->iterationopened && it.second->iterationclosed) {
      //printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      close_cnt ++;
    }
  }
 
  // Check rhs and g value of open set
  printf("Check rhs and g value of openset\n");
  int open_cnt = 0;
  for(const auto& it: hm) {
    if(it.second->iterationopened && !it.second->iterationclosed) {
      //printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      open_cnt ++;
    }
  }

  // Check rhs and g value of null set
  printf("Check rhs and g value of nullset\n");
  int null_cnt = 0;
  for(const auto& it: hm) {
    if(!it.second->iterationopened) {
      //printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      null_cnt ++;
    }
  }

  printf("hm: [%zu], open: [%d], closed: [%d], null: [%d]\n", 
      hm.size(), open_cnt, close_cnt, null_cnt);
} 

void StateSpace::getSubStateSpace(int time_step, std::shared_ptr<env_base>& ENV, const Waypoint& new_goal) {
  ENV->set_goal(new_goal);

  if(best_child_.empty())
    return;
  
  if(!ENV->is_goal(best_child_.back()->coord)) 
    need_to_reset_goal_ = true;

  StatePtr currNode_ptr = best_child_[time_step-1];
  currNode_ptr->pred_action_cost.clear();
  currNode_ptr->pred_action_id.clear();
  currNode_ptr->pred_hashkey.clear();
  currNode_ptr->t = 0;

  for(auto& it: hm_) {
    it.second->g = std::numeric_limits<double>::infinity();
    it.second->rhs = std::numeric_limits<double>::infinity();
    it.second->pred_action_cost.clear();
    it.second->pred_action_id.clear();
    it.second->pred_hashkey.clear();
    it.second->t = 0;
  }

  currNode_ptr->g = 0;
  currNode_ptr->rhs = 0;

  hashMap new_hm;
  priorityQueue<State> epq;
  currNode_ptr->heapkey = epq.push(std::make_pair(currNode_ptr->rhs, currNode_ptr));
  new_hm[currNode_ptr->hashkey] = currNode_ptr;

  while(!epq.empty()) {
    currNode_ptr = epq.top().second; epq.pop();
    // If reached maximum time, reopen it and clear successors array
    if(currNode_ptr->t >= max_t_) {
      currNode_ptr->iterationclosed = false;
      currNode_ptr->g = std::numeric_limits<double>::infinity();
      currNode_ptr->succ_hashkey.clear();
      currNode_ptr->succ_action_cost.clear();
      currNode_ptr->succ_action_id.clear();
    }

    for(unsigned int i = 0; i < currNode_ptr->succ_hashkey.size(); i++) {
      Key succ_key = currNode_ptr->succ_hashkey[i];

      StatePtr& succNode_ptr = new_hm[succ_key];
      if(!succNode_ptr) {
        succNode_ptr = hm_[succ_key];
        succNode_ptr->t = currNode_ptr->t + dt_;
        succNode_ptr->epq_opened = false;
      }

      int id = -1;
      for(unsigned int i = 0; i < succNode_ptr->pred_hashkey.size(); i++) {
        if(succNode_ptr->pred_hashkey[i] == currNode_ptr->hashkey) {
          id = i;
          break;
        }
      }
      if(id == -1) {
        succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        succNode_ptr->pred_action_cost.push_back(currNode_ptr->succ_action_cost[i]);
        succNode_ptr->pred_action_id.push_back(currNode_ptr->succ_action_id[i]);
      }

      double tentative_rhs = currNode_ptr->rhs + currNode_ptr->succ_action_cost[i];

      if(tentative_rhs < succNode_ptr->rhs) {
        succNode_ptr->t = currNode_ptr->t + dt_;
        succNode_ptr->rhs = tentative_rhs;
        if(succNode_ptr->epq_opened) // if succ exists in epq, update epq
        {
          (*succNode_ptr->heapkey).first = succNode_ptr->rhs;     // update heap element
          epq.increase( succNode_ptr->heapkey );       // update heap
        }
        else {
          // if the node is in original closeset, add to epq
          if(succNode_ptr->iterationclosed) {
            succNode_ptr->g = succNode_ptr->rhs; // set g == rhs
            succNode_ptr->heapkey = epq.push( std::make_pair(succNode_ptr->rhs, succNode_ptr) );
            succNode_ptr->epq_opened = true;
          }
        }
      }
    }

  }

  hm_ = new_hm;

  bool goal_changed = ENV->goal_node_ != new_goal;
  pq_.clear();
  for(auto& it: hm_) {
    if(it.second->iterationopened && !it.second->iterationclosed) {
      // If goal changed, recalculate the heuristic
      if(goal_changed)
        it.second->h = ENV->get_heur(it.second->coord, it.second->t);
      it.second->heapkey = pq_.push( std::make_pair(calculateKey(it.second), it.second) );
    }
  }
}


std::vector<Primitive> StateSpace::increaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV) {
  std::vector<Primitive> prs;
  for(const auto& affected_node: states) {
    //update edge
    StatePtr& succNode_ptr = hm_[affected_node.first];
    const int i = affected_node.second; // i-th pred
    if(!std::isinf(succNode_ptr->pred_action_cost[i])) {
      succNode_ptr->pred_action_cost[i] = std::numeric_limits<double>::infinity();
      updateNode(succNode_ptr);

      Key parent_key = succNode_ptr->pred_hashkey[i];
      Primitive pr;
      ENV->forward_action( hm_[parent_key]->coord, succNode_ptr->pred_action_id[i], pr );
      prs.push_back(pr);

      int succ_act_id = hm_[affected_node.first]->pred_action_id[i];
      for(unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size(); j++) {
        if(succ_act_id == hm_[parent_key]->succ_action_id[j]) {
          hm_[parent_key]->succ_action_cost[j] = std::numeric_limits<double>::infinity();
          break;
        }
      }
    }
  }

  if(!prs.empty())
    need_to_reset_goal_ = true;

  return prs;
}

std::vector<Primitive> StateSpace::decreaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base>& ENV) {
  std::vector<Primitive> prs;
  for(const auto& affected_node: states) {
    StatePtr& succNode_ptr = hm_[affected_node.first];
    const int i = affected_node.second;
    if(std::isinf(succNode_ptr->pred_action_cost[i])) {
      Key parent_key = succNode_ptr->pred_hashkey[i];
      Primitive pr;
      ENV->forward_action( hm_[parent_key]->coord, succNode_ptr->pred_action_id[i], pr );
      if(ENV->is_free(pr)) {
        prs.push_back(pr);
        succNode_ptr->pred_action_cost[i] = pr.J(ENV->wi_) + ENV->w_*dt_;
        updateNode(succNode_ptr);
        int succ_act_id = succNode_ptr->pred_action_id[i];
        for(unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size(); j++) {
          if(succ_act_id == hm_[parent_key]->succ_action_id[j]) {
            hm_[parent_key]->succ_action_cost[j] = succNode_ptr->pred_action_cost[i];
            break;
          }
        }

      }
    }
  }

  if(!prs.empty())
    need_to_reset_goal_ = true;
 
  return prs;
}

inline void StateSpace::updateNode(StatePtr& currNode_ptr) {
  //printf("update node at t: %f, rhs: %f\n", currNode_ptr->t, currNode_ptr->rhs);
  // if currNode is not start, update its rhs
  // start rhs is assumed to be 0
  if(currNode_ptr->rhs != 0) {
    currNode_ptr->rhs = std::numeric_limits<double>::infinity();
    for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
      Key pred_key = currNode_ptr->pred_hashkey[i];
      if(currNode_ptr->rhs > hm_[pred_key]->g + currNode_ptr->pred_action_cost[i]) {
        currNode_ptr->rhs = hm_[pred_key]->g + currNode_ptr->pred_action_cost[i];
        currNode_ptr->t = hm_[pred_key]->t + dt_;
      }
    }
  }

  // if currNode is in openset, remove it
  if(currNode_ptr->iterationopened && !currNode_ptr->iterationclosed ) {
    pq_.erase(currNode_ptr->heapkey);
    currNode_ptr->iterationclosed = true;
  }

  // if currNode's g value is not equal to its rhs, put it into openset
  // if(currNode_ptr->g != currNode_ptr->rhs || !currNode_ptr->iterationopened) {
  if(currNode_ptr->g != currNode_ptr->rhs) {
    double fval = calculateKey(currNode_ptr);
    currNode_ptr->heapkey = pq_.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
  }
}

inline double StateSpace::calculateKey(const StatePtr& node) {
  return std::min(node->g, node->rhs) + eps_ * node->h;
}

