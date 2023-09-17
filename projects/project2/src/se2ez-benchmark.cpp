/* Author: Zachary Kingston */

#include <iostream>

#include <boost/program_options.hpp>

#include <se2ez/core.h>
#include <se2ez/plan.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>

using namespace ompl;
using namespace se2ez;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    std::string filename;
    double time;
    std::size_t runs;

    po::options_description desc("Program Usage");
    desc.add_options()                                                                               //
        ("help", "produce help message")                                                             //
        ("filename,f", po::value<std::string>(&filename)->required(), "YAML file to use")            //
        ("time,t", po::value<double>(&time)->default_value(60.), "time per run for benchmark")       //
        ("runs,r", po::value<std::size_t>(&runs)->default_value(50), "number of runs to benchmark")  //
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }

    auto robot = io::loadRobot(filename);
    if (!robot)
        return -1;

    plan::EZPlans planner(robot);
    planner.initialize();
    planner.setStartGoal("start", "goal");

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(time, 10000, runs);
    tools::Benchmark b(*planner.setup, filename);

    auto rrtc1 = std::make_shared<geometric::RRTConnect>(planner.info);
    rrtc1->setName("RRTConnect Range 5.");
    rrtc1->setRange(5.);

    auto rrtc2 = std::make_shared<geometric::RRTConnect>(planner.info);
    rrtc2->setName("RRTConnect Range 1.");
    rrtc2->setRange(1.);

    auto rrtc3 = std::make_shared<geometric::RRTConnect>(planner.info);
    rrtc3->setName("RRTConnect Range 0.1.");
    rrtc3->setRange(0.1);

    auto rrt1 = std::make_shared<geometric::RRT>(planner.info);
    rrt1->setName("RRT Range 5.");
    rrt1->setRange(5.);

    auto rrt2 = std::make_shared<geometric::RRTConnect>(planner.info);
    rrt2->setName("RRT Range 1.");
    rrt2->setRange(1.);

    auto rrt3 = std::make_shared<geometric::RRT>(planner.info);
    rrt3->setName("RRT Range 0.1.");
    rrt3->setRange(0.1);

    b.addPlanner(rrtc1);
    b.addPlanner(rrtc2);
    b.addPlanner(rrtc3);
    b.addPlanner(rrt1);
    b.addPlanner(rrt2);
    b.addPlanner(rrt3);
    b.addPlanner(std::make_shared<geometric::KPIECE1>(planner.info));
    b.addPlanner(std::make_shared<geometric::PRM>(planner.info));

    b.benchmark(request);
    b.saveResultsToFile();
}
