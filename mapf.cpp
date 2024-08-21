#include <getopt.h>

#include <default_params.hpp>
#include <fstream>
#include <hca.hpp>
#include <iomanip>
#include <iostream>
#include <pibt.hpp>
#include <pibt_plus.hpp>
#include <problem.hpp>
#include <push_and_swap.hpp>
#include <random>
#include <variant>
#include <vector>

void printHelp();
std::unique_ptr<MAPF_Solver> getSolver(const std::string solver_name,
                                       MAPF_Instance* P, bool verbose, int argc,
                                       char* argv[]);

int main(int argc, char* argv[])
{
  std::string instance_file = "";
  std::string scen_file = "";
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char* argv_copy[argc + 1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"scenario", required_argument, 0, 'm'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"help", no_argument, 0, 'h'},
      {"time-limit", required_argument, 0, 'T'},
      {"log-short", no_argument, 0, 'L'},
      {"make-scen", no_argument, 0, 'P'},
      {0, 0, 0, 0},
  };
  bool make_scen = false;
  bool log_short = false;
  int max_comp_time = -1;

  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:m:o:s:vhPT:L", longopts,
                            &longindex)) != -1) {
    switch (opt) {
      case 'i':
        instance_file = std::string(optarg);
        break;
      case 'm':
        scen_file = std::string(optarg);
        break;
      case 'o':
        output_file = std::string(optarg);
        break;
      case 's':
        solver_name = std::string(optarg);
        break;
      case 'v':
        verbose = true;
        break;
      case 'h':
        printHelp();
        return 0;
      case 'P':
        make_scen = true;
        break;
      case 'L':
        log_short = true;
        break;
      case 'T':
        max_comp_time = std::atoi(optarg);
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapf -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  if (scen_file.length() == 0) {
    std::cout << "specify instance file using -m [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapf -i ../instance/sample.txt" << std::endl;
    return 0;
  }



  // set up parameters for generating charts
  std::string base_path = "../instances/mapf/scen-random/random-32-32-20-random-";
//  std::string base_path = "../instances/mapf/scen-random_1/ost003d-random-";
//  std::string base_path = "../instances/mapf/scen-random_2/random-64-64-20-random-";
//  std::string base_path = "../instances/mapf/scen-random_warehouse-10-20-10-2-1/warehouse-10-20-10-2-1-random-";
//  std::string base_path = "../instances/mapf/scen-random_warehouse-10-20-10-2-2/warehouse-10-20-10-2-2-random-";
//  std::string base_path = "../instances/mapf/scen-random_warehouse-20-40-10-2-1/warehouse-20-40-10-2-1-random-";
//  std::string base_path = "../instances/mapf/scen-random_warehouse-20-40-10-2-2/warehouse-20-40-10-2-2-random-";

  using VarType = std::variant<int, float>;
  std::vector<std::vector<VarType>> obj_table;


  // objectives
  volatile float temp_soc = 0;
  volatile float temp_soc_lb = 0;
  volatile float temp_makespan = 0;
  volatile float temp_makespan_lb = 0;
  volatile float temp_time = 0;


  volatile float sum_soc_over_lb = 0;
  volatile float sum_makespan_over_lb = 0;
  volatile float sum_time = 0;
  volatile float sum_success = 0;



  int max_num_agents = 200;
  for (int num_of_agents = 10; num_of_agents <= max_num_agents; num_of_agents = num_of_agents + 10)
  {
    // initialize parameters
    sum_soc_over_lb = 0;
    sum_makespan_over_lb = 0;
    sum_time = 0;
    sum_success = 0;


    for (int scen_num = 1; scen_num <= 25; ++scen_num)
    {
      scen_file = base_path + std::to_string(scen_num) + ".scen";
      // set problem
      auto P = MAPF_Instance(instance_file, scen_file, num_of_agents);

      // set max computation time (otherwise, use param in instance_file)
      if (max_comp_time != -1) P.setMaxCompTime(max_comp_time);

      // create scenario
      if (make_scen) {
        P.makeScenFile(output_file);
        return 0;
      }

      // solve
      auto solver = getSolver(solver_name, &P, verbose, argc, argv_copy);
      solver->setLogShort(log_short);
      solver->solve();
      if (solver->succeed() && !solver->getSolution().validate(&P)) {
        std::cout << "error@mapf: invalid results" << std::endl;
        return 0;
      }
      solver->printResult();

      temp_soc = (float)solver->getSOC();
      temp_soc_lb = (float)solver->getLowerBoundSOC();
      temp_makespan = (float)solver->getMakespan();
      temp_makespan_lb = (float)solver->getLowerBoundMakespan();
      temp_time = solver->getCompTime();

      if (solver->succeed())
      {
        sum_success = sum_success + 1;

        sum_soc_over_lb = sum_soc_over_lb + temp_soc / temp_soc_lb;
        sum_makespan_over_lb = sum_makespan_over_lb + temp_makespan / temp_makespan_lb;
        sum_time = sum_time + temp_time;
      }

    }

    std::vector<VarType> temp_list;
    temp_list.push_back(num_of_agents);
    temp_list.push_back(sum_success / 25);
    temp_list.push_back(sum_time / sum_success);
    temp_list.push_back(sum_soc_over_lb / sum_success);
    temp_list.push_back(sum_makespan_over_lb / sum_success);



    obj_table.push_back(temp_list);


  }


  // print out the test results

  // Create an output file stream for the CSV file
  std::ofstream outfile("output.csv");

  // Check if the file is opened successfully
  if (!outfile.is_open()) {
    std::cerr << "Failed to open the file." << std::endl;
    return 1;
  }

  // Set precision for floating point numbers
  outfile << std::fixed << std::setprecision(5);

  // Iterate over the rows of the table
  for (const auto& row : obj_table) {
    // Iterate over the elements in each row
    for (size_t i = 0; i < row.size(); ++i) {
      // Use std::visit to handle the different types in VarType
      std::visit([&outfile](auto&& arg) {
        outfile << arg;
      }, row[i]);
      // Add a comma if it's not the last element in the row
      if (i < row.size() - 1) {
        outfile << ",";
      }
    }
    // Print a new line after each row
    outfile << std::endl;
  }

  // Close the file
  outfile.close();



//  // output result
//  solver->makeLog(output_file);
//  if (verbose) {
//    std::cout << "save result as " << output_file << std::endl;
//  }

  return 0;
}

std::unique_ptr<MAPF_Solver> getSolver(const std::string solver_name,
                                       MAPF_Instance* P, bool verbose, int argc,
                                       char* argv[])
{
  std::unique_ptr<MAPF_Solver> solver;
  if (solver_name == "PIBT") {
    solver = std::make_unique<PIBT>(P);
  } else if (solver_name == "HCA") {
    solver = std::make_unique<HCA>(P);
  } else if (solver_name == "PIBT_PLUS") {
    solver = std::make_unique<PIBT_PLUS>(P);
  } else if (solver_name == "PushAndSwap") {
    solver = std::make_unique<PushAndSwap>(P);
  } else {
    std::cout << "warn@mapf: "
              << "unknown solver name, " + solver_name + ", continue by PIBT"
              << std::endl;
    solver = std::make_unique<PIBT>(P);
  }
  solver->setParams(argc, argv);
  solver->setVerbose(verbose);
  return solver;
}

void printHelp()
{
  std::cout << "\nUsage: ./mapf [OPTIONS] [SOLVER-OPTIONS]\n"
            << "\n**instance file is necessary to run MAPF simulator**\n\n"
            << "  -i --instance [FILE_PATH]     instance file path\n"
            << "  -o --output [FILE_PATH]       ouptut file path\n"
            << "  -v --verbose                  print additional info\n"
            << "  -h --help                     help\n"
            << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
            << "  -T --time-limit [INT]         max computation time (ms)\n"
            << "  -L --log-short                use short log"
            << "  -P --make-scen                make scenario file using "
               "random starts/goals"
            << "\n\nSolver Options:" << std::endl;
  // each solver
  PIBT::printHelp();
  HCA::printHelp();
  PIBT_PLUS::printHelp();
  PushAndSwap::printHelp();
}
