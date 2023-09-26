///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

/* Change log*/

#include <iostream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a point robot.
}

void planBox(const std::vector<Rectangle> &obstacles)
{
{
    // Define the state space, space information, and problem definition

    // Create a 2D real vector state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    // Set bounds for the state space as needed
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -1.0);  // Lower bound for x
    bounds.setHigh(0, 1.0);  // Upper bound for x
    bounds.setLow(1, -1.0);  // Lower bound for y
    bounds.setHigh(1, 1.0);  // Upper bound for y
    space->setBounds(bounds);

    // Create a space information using the state space
    auto si(std::make_shared<ompl::control::SpaceInformation>(space));
    ompl::control::ProblemDefinitionPtr pdef(new ompl::control::ProblemDefinition(si));

    // Set the state validity checker for point robot
    si->setStateValidityChecker(std::bind(&isValidStatePoint, std::placeholders::_1, obstacles));

    // Create the RTP planner and set problem definition
    ompl::control::RTPPtr planner(new ompl::control::RTP(si));
    planner->setProblemDefinition(pdef);

    // Attempt to solve the problem
    ompl::base::PlannerStatus status = planner->solve(ompl::base::PlannerTerminationCondition(10.0));

    if (status == ompl::base::PlannerStatus::SOLVED)
    {
        std::cout << "Box robot planning succeeded!" << std::endl;
        // Extract and print the solution path, if needed
        ompl::base::PathPtr solutionPath = pdef->getSolutionPath();
        // Print or process the solution path as needed
    }
    else
    {
        std::cout << "Box robot planning failed." << std::endl;
    }
}
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
    
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.

    // Define the obstacles for environment 2
    Rectangle obstacle1 = {1.0, 1.0, 0.2, 0.4};
    Rectangle obstacle2 = {2.0, 2.5, 0.3, 0.2};
    Rectangle obstacle3 = {4.0, 3.5, 0.4, 0.2};
    
    obstacles.clear();
    obstacles.push_back(obstacle1);
    obstacles.push_back(obstacle2);
    
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
