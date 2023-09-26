///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////

/* Change log*/

#include <iostream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    // Create a 2D real vector state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    // Set the bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);

    // Create an instance of space information for the state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // Set the state validity checker
    si->setStateValidityChecker( [] (const State* state) -> bool
        return isStateValidPoint(state, obstacles); 
    ); 
    
    // Create start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -0.7;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -0.8;
    
    // Create goal state
    ompl::base::ScopedState<> goal(space); 
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.8;
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.8;

    // Create problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the RTP planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RTP>(si));

    // Give the problem definition to the RTP planner
    planner->setProblemDefinition(pdef);

    // Perform the setup steps for the planner
    planner->setup();

    // Print the setting used for the space information
    si->printSettings(std::cout);

    // Print the problem settings
    pdef->print(std::cout);

    // Request to solve the problem within thirty seconds of planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(30.0);

    if (solved)
    {
        // get the goal information from pdef and inquire about the solution path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Solution path was found:" << std::endl;

        // placeholder print path to screen (later, can put an output stream to a textfile
        // for visualization, once we know the RTP implementation is correct)
        path->print(std::cout);
    }
    else
    {
        std::cout << "No solution was found." << std::endl;
    }
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // Define the state space, space information, and problem definition

    // Create a 2D real vector state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
    // auto space(std::make_shared<ompl::base::SE2StateSpace>()); maybe use this to include rotation?

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

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // Initialize obstacles vector as empty
    obstacles.clear();
    
    // Define the obstacles
    Rectange obstacle1 = {-0.5, -1.0, 0.2, 1.4};
    Rectange obstacle2 = {-0.4, 0.6, 0.2, 0.4};
    Rectange obstacle3 = {0.0, -1.0, 0.2, 0.7};
    Rectange obstacle4 = {0.0, -0.1, 0.2, 1.1};
    Rectange obstacle5 = {0.4, 0.3, 0.6, 0.2};
    Rectange obstacle6 = {0.5, 0.6, 0.2, 0.4};

    // Add the new obstacles to the obstacle vector 
    obstacles.push_back(obstacle1); 
    obstacles.push_back(obstacle2); 
    obstacles.push_back(obstacle3); 
    obstacles.push_back(obstacle4); 
    obstacles.push_back(obstacle5); 
    obstacles.push_back(obstacle6); 
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
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
