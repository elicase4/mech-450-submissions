///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////

/* Change log

Implemented finalized planBox and MakeEnv2 functions. 
Also implemented isValid functions but they're not needed.
Edited the general code a bit for syntax, but some errors persist. -- Santi, 9/26/23 7:40pm


*/

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
    si->setStateValidityChecker([obstacles] (const ompl::base::State* state){
        return isValidStatePoint(state, obstacles);
    }); 
    
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
        std::cout << "Point Robot solution path was found." << std::endl;

        // placeholder print path to screen (later, can put an output stream to a textfile
        // for visualization, once we know the RTP implementation is correct)
        path->print(std::cout);
    }
    else
    {
        std::cout << "No point robot solution was found." << std::endl;
    }
}

/* This function is to plan for a box (square) robot. Written by Santi.*/
void planBox(const std::vector<Rectangle> &obstacles)
{
   
    // Create a 2D real vector state space (taken from Clayton's Piazza comments)
    auto space(std::make_shared<ompl::base::SE2StateSpace>());

    // Define the bounds for the space. To start, we'll use a 2x1 box.
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(2);

    // Set the bounds
    space->setBounds(bounds);

    // Create a space information using the state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    

    // Set the state validity checker for point robot
    // si->setStateValidityChecker(std::bind(&isValidStatePoint, std::placeholders::_1, obstacles));
    si->setStateValidityChecker([obstacles] (const ompl::base::State* state){
        // using sideLen = 0.05
        return isValidStateSquare(state, 0.05, obstacles);
    }); 

    // Create start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::SE2StateSpace::StateType>()->setX(0.1);
    start->as<ompl::base::SE2StateSpace::StateType>()->setY(0.1);
    
    // Create goal state
    ompl::base::ScopedState<> goal(space); 
    goal->as<ompl::base::SE2StateSpace::StateType>()->setX(0.8);
    goal->as<ompl::base::SE2StateSpace::StateType>()->setY(0.8);

    // Problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si)); 

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);


    // Create the RTP planner for the space and set the problem definition
    auto planner(std::make_shared<ompl::geometric::RTP>(si));
    planner->setProblemDefinition(pdef);

    // Setup the planner.
    planner->setup();

    // Print problem settings
    pdef->print(std::cout);


    // Attempt to solve the problem
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(30.0);

    // If the planner found a solution, print it to screen
    if (solved)
    {
        std::cout << "Box robot planning succeeded!" << std::endl;
        // Extract and print the solution path, if needed
        ompl::base::PathPtr solutionPath = pdef->getSolutionPath();
        // Print or process the solution path as needed
        solutionPath->print(std::cout);
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
    
    // Initialize setter for obstacles
    std::vector<double> obstacleAdd;
    
    // Rectangle setter function
    auto addRectangle = [&] (const std::vector<double>& settings) {
        Rectangle rectangleTmp;
        rectangleTmp.x = settings[0];
        rectangleTmp.y = settings[1];
        rectangleTmp.width = settings[2];
        rectangleTmp.height = settings[3];
        obstacles.push_back(rectangleTmp);
    };

    // Define the obstacles
    obstacleAdd = {-0.5, -1.0, 0.2, 1.4};
    addRectangle(obstacleAdd);
    obstacleAdd = {-0.4, 0.6, 0.2, 0.4};
    addRectangle(obstacleAdd);
    obstacleAdd = {0.0, -1.0, 0.2, 0.7};
    addRectangle(obstacleAdd);
    obstacleAdd = {0.0, -0.1, 0.2, 1.1};
    addRectangle(obstacleAdd);
    obstacleAdd = {0.4, 0.3, 0.6, 0.2};
    addRectangle(obstacleAdd);
    obstacleAdd = {0.5, 0.6, 0.2, 0.4};
    addRectangle(obstacleAdd);

    std::cout << "Environment 1 created using "<< obstacles.size() << " total obstacles."<< std::endl;
}

// Make the second environment to test our planner. Written by Santi.
void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // Define the obstacles as desired. For simplicity, any given obstacle is defined as follows
    // Rectangle obstacle{num};
    // obstacle{num}.x = {num};
    // obstacle{num}.y = {num};
    // obstacle{num}.width = {num};
    // obstacle{num}.height = {num};
    // This, of course, implies that only rectangles are used. Repeat as many times as desired

    // Note to instructors: this can probably (definitely) be simplified with a for loop. 
    // I'm not built for that just yet. Sorry.

    Rectangle obstacle1;                        // define first obstacle
    obstacle1.x = 1.0;                          // xcoordinate of origin
    obstacle1.y = 1.0;                          // ycoordinate of origin
    obstacle1.width = 0.2;                      // obstacle width
    obstacle1.height = 0.4;                     // obstacle height

    Rectangle obstacle2;                        
    obstacle2.x = 2.0;                          
    obstacle2.y = 2.5;                          
    obstacle2.width = 0.3;                      
    obstacle2.height = 0.2; 
    
    Rectangle obstacle3;                        
    obstacle3.x = 4.0;                          
    obstacle3.y = 3.5;                          
    obstacle3.width = 0.4;                      
    obstacle3.height = 0.2; 
    
    obstacles.clear();
    obstacles.push_back(obstacle1);
    obstacles.push_back(obstacle2);
    obstacles.push_back(obstacle3);

    std::cout << "Environment 2 created using "<< obstacles.size() << " total obstacles."<< std::endl;
}

// I think we can implement with lambda funcitons that use the provided implementations in collisionchecking.h
/*
// We also need functions to find state validity in order to continue planning.
bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a real vector state space
    auto rvlstate = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Get the coordinates of the state as x and y
    double x = rvlstate->values[0];
    double y = rvlstate->values[1];

    // Check if the state is valid and  return the result
    return isValidStatePoint(x, y, obstacles);
}

// This is the same boolean function as the previous one, except for the square robot.
bool isValidStateSquare(const ompl::base::State *state, double sidelength, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a real vector state space
    auto rvlstate = state->as<ompl::base::SE2StateSpace::StateType>(0);

    // Get the coordinates of the state as x and y
    double x = rvlstate->values[0];
    double y = rvlstate->values[1];

    // Get the orientation of the square as theta by using the SE2 space
    auto rotstate = state->as<ompl::base::SE2StateSpace::StateType>(1)

    double theta = rotstate->value;

    // Check if the state is valid and return the result
    return isValidStateSquare(x, y, theta, sidelength, obstacles);
}
*/

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
        std::cout << " (1) Eli's Environment" << std::endl;
        std::cout << " (2) Santi's Environment" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            std::cout<<"Environment 1"<<std::endl;
            makeEnvironment1(obstacles);
            break;
        case 2:
            std::cout<<"Environment 2"<<std::endl;
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            std::cout<<"Point Robot"<<std::endl;
            planPoint(obstacles);
            break;
        case 2:
            std::cout<<"Box Robot"<<std::endl;
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
