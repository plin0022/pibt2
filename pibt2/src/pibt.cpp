#include "../include/pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBT::SOLVER_NAME;
}

void PIBT::run()
{
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
//    // use initial distance
//    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    // use current distance
    if (a->curr_d != b->curr_d) return a->curr_d < b->curr_d;
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{
        i,                          // id
        s,                          // current location
        nullptr,                    // next location
        g,                          // goal
        0,                          // elapsed
        d,                          // dist from s -> g
        0,                         // curr_dist from s-> g
        getRandomFloat(0, 1, MT),// tie-breaker
        0,
        0
    };
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
    }



    // observing current_comp
    volatile int curr_sum_of_comp = 0;
    for (auto a : A)
    {
      curr_sum_of_comp = curr_sum_of_comp + a->current_comp;
    }

    // modified acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);

    if (curr_sum_of_comp == 0)
    {
      for (auto a : A) {
        // clear
        if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
        occupied_next[a->v_next->id] = nullptr;
        // set next location
        config[a->id] = a->v_next;
        occupied_now[a->v_next->id] = a;
        // check goal condition
        check_goal_cond &= (a->v_next == a->g);
        // update priority
        a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
        // reset params
        a->v_now = a->v_next;
        a->v_next = nullptr;
        a->current_comp = 0;
      }
    }
    else
    {
      volatile int initial_sum_comp = 0;
      for (auto a : A)
      {
        initial_sum_comp = initial_sum_comp + a->current_comp;
        config[a->id] = a->v_next;
      }

      // simulation
      for (int idx = 0; idx < 100; ++idx)
      {
        for (auto a : A) {
          // clear
          if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
          occupied_next[a->v_next->id] = nullptr;
          // backtrace
          occupied_now[a->v_now->id] = a;
          // update priority
          if (a->current_comp != 0)
          {
            a->tie_breaker = getRandomFloat(0, 1, MT);
          }
          a->curr_d = 0;

          // reset params
          a->v_next = nullptr;
          a->sum_of_comp = a->sum_of_comp - a->current_comp;
          a->current_comp = 0;
        }

        updateCURRENTDIS(A);

        // replan
        std::sort(A.begin(), A.end(), compare);
        for (auto a : A) {
          // if the agent has next location, then skip
          if (a->v_next == nullptr) {
            // determine its next location
            funcPIBT(a);
          }
        }

        volatile int temp_sum_comp = 0;
        for (auto a : A)
        {
          temp_sum_comp = temp_sum_comp + a->current_comp;
        }

        if (temp_sum_comp < initial_sum_comp)
        {
          initial_sum_comp = temp_sum_comp;
          for (auto a : A)
          {
            config[a->id] = a->v_next;
          }
        }

      }

      for (auto a : A) {
        // clear
        if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
        occupied_next[a->v_next->id] = nullptr;
        // set next location
        occupied_now[config[a->id]->id] = a;
        // check goal condition
        check_goal_cond &= (config[a->id] == a->g);
        // update priority
        a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
//        a->curr_d = 0;
        // reset params
        a->v_now = config[a->id];
        a->v_next = nullptr;
        a->current_comp = 0;
      }
    }



    // acting
//    bool check_goal_cond = true;
//    Config config(P->getNum(), nullptr);
//    for (auto a : A) {
//      // clear
//      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
//      occupied_next[a->v_next->id] = nullptr;
//      // set next location
//      config[a->id] = a->v_next;
//      occupied_now[a->v_next->id] = a;
//      // check goal condition
//      check_goal_cond &= (a->v_next == a->g);
//      // update priority
//      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
//      // reset params
//      a->v_now = a->v_next;
//      a->v_next = nullptr;
//    }
    

    // update plan
    solution.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
//    if (timestep >= max_timestep || overCompTime()) {
//      break;
//    }

    // failed
    if (timestep >= max_timestep) {
      break;
    }
  }

  // sum of compromises of all agents
  volatile int all_sum_of_comp = 0;
  for (auto a : A)
  {
    all_sum_of_comp = all_sum_of_comp + a->sum_of_comp;
  }


  // memory clear
  for (auto a : A) delete a;
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  // find out the shortest distance
  volatile int shortest_dist = pathDist(ai->id, C[0]);
  volatile int comp = 0;

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;
    if (aj != nullptr && u == aj->v_now) continue;

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)) continue;  // replanning
    }


    // define compromise
    if (ai->v_now == ai->g)
    {
      comp = 2 * (pathDist(ai->id, ai->v_next) - shortest_dist);
    }
    else
    {
      comp = pathDist(ai->id, ai->v_next) - shortest_dist;
    }

    // success to plan next one step
    ai->current_comp = comp;
    ai->sum_of_comp = ai->sum_of_comp + comp;
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;

  // define compromise
  comp = pathDist(ai->id, ai->v_next) - shortest_dist;
  ai->current_comp = comp;
  ai->sum_of_comp = ai->sum_of_comp + comp;
  return false;
}

void PIBT::updateCURRENTDIS(const Agents& A)
{
  // update current distance
  for (auto a : A)
  {
    // get candidates
    Nodes C = a->v_now->neighbor;
    C.push_back(a->v_now);
    volatile int current_value = pathDist(a->id, a->v_now);

    // get dis_vector
//    std::vector<int> dis_vector;
//    for (auto c_node : C)
//    {
//      dis_vector.push_back(pathDist(a->id, c_node));
//    }

    // evaluate the current position potential for deciding priority of the agent
    volatile int final_value = 5;
    for (auto c_node : C)
    {
      final_value = final_value + pathDist(a->id, c_node) - current_value;
    }



//    // Find the iterator to the minimum element
//    auto min_it = std::min_element(dis_vector.begin(), dis_vector.end());
//
//    // Dereference the iterator to get the minimum value
//    volatile int min_value = *min_it;

    // find out the shortest distance
//    volatile int counter = 0;
//    for (auto dis_num : dis_vector)
//    {
//      if (dis_num == min_value) counter = counter + 1;
//    }

    a->curr_d = a->tie_breaker + (float) final_value;
  }
}


void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
