/* Author: Zachary Kingston */

#include <iostream>

#include <se2ez/core.h>
#include <se2ez/plan.h>

using namespace se2ez;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Please give YAML filename to load!" << std::endl;
        return -1;
    }

    auto robot = io::loadRobot(std::string(argv[1]));
    if (!robot)
        return -1;

    plan::EZPlans planner(robot);
    planner.initialize();
    planner.setPlanner("rrt:RRTConnect");
    planner.setStartGoal("start", "goal");

    ompl::base::PlannerStatus solved = planner.setup->solve(1.0);

    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        planner.setup->getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}
