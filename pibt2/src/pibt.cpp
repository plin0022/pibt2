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
  // find out boss
  auto compare_boss = [](Agent* a, const Agent* b) {
    // top layer
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;

    if (a->curr_d != b->curr_d) return a->curr_d < b->curr_d;
    // use boss tie-breaker to find boss
    return a->boss_tie_breaker > b->boss_tie_breaker;

  };

  // compare with boss value
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    if (a->boss != b->boss) return a->boss > b->boss;

    // use flexibility
//    if (a->flex != b->flex) return a->flex < b->flex;

//    if (a->curr_d != b->curr_d) return a->curr_d < b->curr_d;


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
        0,
        getRandomFloat(0, 1, MT),  // boss tie-breaker
        0,                         // curr_dist from current-> g
        0,                           // flexibility
        getRandomFloat(0, 1, MT),// tie-breaker
        0,
        0
    };
    a->curr_d = pathDist(a->id, a->v_now);
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());


  // main loop
  int boss_id = 0;
  int timestep = 0;
  float random_number = 0;




  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

//    random_number = getRandomFloat(0, 1, MT);
//
//    if (random_number < 0.3) updateCURRENTDIS(A);
//    updateCURRENTDIS(A);

//
//     update boss
//    if (timestep == 0)
//    {
//      std::sort(A.begin(), A.end(), compare_boss);
//      A[0]->boss = 1;
//      boss_id = A[0]->id;
//    }
//    else
//    {
//      volatile bool flag = false;
//      for (auto a : A)
//      {
//        if ((a->id == boss_id) && (a->v_now == a->g))
//        {
//          a->elapsed = 0;
//          a->boss = 0;
//          flag = true;
//          break;
//        }
//      }
//      if (flag)
//      {
//        std::sort(A.begin(), A.end(), compare_boss);
//        A[0]->boss = 1;
//        boss_id = A[0]->id;
//      }
//    }




    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
    }



    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
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

      // update boss tie-breaker, so any agent has a chance to be boss
      a->boss_tie_breaker = getRandomFloat(0, 1, MT);

      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;

      // update current distance
      a->curr_d = pathDist(a->id, a->v_now);
    }


    // update plan
    solution.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }

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
    int flex_v = flex_table[ai->id][v->id];
    int flex_u = flex_table[ai->id][u->id];

    if (d_v != d_u) return d_v < d_u;

    if (flex_v != flex_u) return flex_v > flex_u;

    // tie breaker
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

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;   // avoid vertex conflict
    if (aj != nullptr && u == aj->v_now) continue;   // avoid swap conflict

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)) continue;  // replanning
    }

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;

  return false;
}

// evaluate flexibility for a node to an agent
int PIBT::evalFlex(Node* a_node, Agent* a)
{
  auto compare_node = [&](Node* const v, Node* const u) {
    int d_v = pathDist(a->id, v);
    int d_u = pathDist(a->id, u);

    if (d_v != d_u) return d_v < d_u;

    return false;
  };

  Nodes C = a_node->neighbor;
  C.push_back(a_node);


  std::sort(C.begin(), C.end(), compare_node);

  volatile int min_dis = pathDist(a->id, C[0]);
  volatile int final_value = 0;
  int current_dis = 0;


  for (auto v : C)
  {
    // give zero mark if occupied by another agent in next step
    if (occupied_next[v->id] != nullptr) continue;

    current_dis = pathDist(a->id, v);

    if ((current_dis - min_dis) == 0)
    {
      final_value = final_value + 2;
    }
    else if ((current_dis - min_dis) == 1)
    {
      final_value = final_value + 1;
    }
    else if ((current_dis - min_dis) == 2)
    {
      final_value = final_value + 0;
    }
  }

  return final_value;

}

void PIBT::updateCURRENTDIS(const Agents& A)
{
  // update current distance
  for (auto a : A)
  {
    // get candidates
    Nodes C = a->v_now->neighbor;
    C.push_back(a->v_now);


    // get dis_vector
    std::vector<int> dis_vector;
    for (auto c_node : C)
    {
      dis_vector.push_back(pathDist(a->id, c_node));
    }

    // Find the iterator to the minimum element
    auto min_it = std::min_element(dis_vector.begin(), dis_vector.end());

    // Dereference the iterator to get the minimum value
    volatile int min_value = *min_it;

    // evaluate the current position potential for deciding priority of the agent
    volatile int final_value = 0;
    for (auto c_node : C)
    {
      if (occupied_next[c_node->id] != nullptr) continue;

      if ((pathDist(a->id, c_node) - min_value) == 0)
      {
        final_value = final_value + 2;
      }
      else if ((pathDist(a->id, c_node) - min_value) == 1)
      {
        final_value = final_value + 1;
      }
      else if ((pathDist(a->id, c_node) - min_value) == 2)
      {
        final_value = final_value + 0;
      }
    }

    a->flex = final_value;
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
