///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////

/*
Change log:

Created first code draft. Commented as many lines as possible. -- Santi

*/

#include <iostream>

// Our random tree planner
#include "RTP.h"

// Including the other planners of interest from the OMPL library
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>

// Including the Benchmark program
#include <ompl/tools/benchmark/Benchmark.h>

// Per the project file suggestion, we also include the SE3 Rigid Body Planning program.
#include <omplapp/apps/SE3RigidBodyPlanning.h>

void benchmarkApartment()
{
    // Initialize the SE3RigidBodyPlanning setup
    ompl::app::SE3RigidBodyPlanning setup;

    // Initialize run count variable for counting.
    int run_count = 50;

    // Initialize runtime and memory limit variables. 
    // Without these, the benchmark risks running forever.
    double runtime_lim = 60.0;
    double memory_lim = 10000.0;

    /*
    Set up the mesh and environments. These require calling the .dae files for 
    the environment. Note to Eli and/or instructors: not really sure if we have to
    make these calls general so that it can run on any maching or if we're operating 
    under the assumption that we're using Docker all the time.

    */
    std::string robot = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string world = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";

    // Initialize the start state (values taken from the robot.dae file)
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81);
    start->setY(106.15);
    start->setZ(36.46);
    start->rotation().setIdentity();

    // Initialize the end state.
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(-31.19);
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->rotation().setIdentity();

    // Finalizing setup
    setup.setStartAndGoalStates(start, goal);
    
    // Set collision checking (taken from SE3RigidBodyPlanning.cpp)
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01); 
    setup.setup();

    // Print the setup (optional)
    setup.print();

    // Request the benchmark
    ompl::tools::Benchmark::Request request(runtime_lim, memory_lim, run_count);
    ompl::tools::Benchmark bench(setup, "Apartment");


    // Add the planners to the benchmark as needed
    bench.addPlanner(std::make_shared<ompl::geometric::RTP>(setup.getSpaceInformation())); // RTP
    bench.addPlanner(std::make_shared<ompl::geometric::EST>(setup.getSpaceInformation())); // EST
    bench.addPlanner(std::make_shared<ompl::geometric::RRT>(setup.getSpaceInformation())); // RRT
    bench.addPlanner(std::make_shared<ompl::geometric::PRM>(setup.getSpaceInformation())); // PRM

    // Run the benchmark
    bench.benchmark(request);
    
    // Save benchmark results to file for use with Planner-Arena:
    bench.saveResultsToFile();

}

// Note: this code is the exact same as that for the Apartment environment, just tailored to Home.
void benchmarkHome()
{
    // Initialize the SE3RigidBodyPlanning setup
    ompl::app::SE3RigidBodyPlanning setup;

    // Initialize run count variable for counting.
    int run_count = 50;

    // Initialize runtime and memory limit variables. 
    // Without these, the benchmark risks running forever.
    double runtime_lim = 60.0;
    double memory_lim = 10000.0;

    /*
    Set up the mesh and environments. These require calling the .dae files for 
    the environment. Note to Eli and/or instructors: not really sure if we have to
    make these calls general so that it can run on any maching or if we're operating 
    under the assumption that we're using Docker all the time.

    */
    std::string robot = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string world = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";

    // Initialize the start state (values taken from the robot.dae file)
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setIdentity();

    // Initialize the end state.
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(262.95);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setIdentity();

    // Finalizing setup
    setup.setStartAndGoalStates(start, goal);
    
    // Set collision checking (taken from SE3RigidBodyPlanning.cpp)
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01); 
    setup.setup();

    // Print the setup (optional)
    setup.print();

    // Request the benchmark
    ompl::tools::Benchmark::Request request(runtime_lim, memory_lim, run_count);
    ompl::tools::Benchmark bench(setup, "Home");

    // Add the planners to the benchmark as needed
    bench.addPlanner(std::make_shared<ompl::geometric::RTP>(setup.getSpaceInformation())); // RTP
    bench.addPlanner(std::make_shared<ompl::geometric::EST>(setup.getSpaceInformation())); // EST
    bench.addPlanner(std::make_shared<ompl::geometric::RRT>(setup.getSpaceInformation())); // RRT
    bench.addPlanner(std::make_shared<ompl::geometric::PRM>(setup.getSpaceInformation())); // PRM

    // Run the benchmark
    bench.benchmark(request);
    
    // Save benchmark results to file for use with Planner-Arena:
    bench.saveResultsToFile();

}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkApartment();
            break;
        case 2:
            benchmarkHome();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
