/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    int boss;           // boss_value
    float boss_tie_breaker;   // randomly chosen boss
    float curr_d;         // curr_d
    float tie_breaker;  // epsilon, tie-breaker
    int sum_of_comp;    // compromises
    int current_comp;   // current compromises used for comparing plans
  };
  using Agents = std::vector<Agent*>;

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  Agents temp_occupied_now;
  Agents temp_occupied_next;

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);


  // main
  void run();

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void updateCURRENTDIS(const Agents& A);
  void setParams(int argc, char* argv[]);
  static void printHelp();

};
