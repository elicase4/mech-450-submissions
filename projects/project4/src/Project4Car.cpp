///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Eli Case, Santi Parra-Vargas, Jason Ye
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Headers for state space and control space representations
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Headers for RRT and KPIECE planners and projection evaluator
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Header for path and env output
#include <fstream>

// Header to assist file naming convention
#include <string>

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace* space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ompl::base::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        auto se2State = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
        auto r2State = se2State->as<ompl::base::RealVectorStateSpace::StateType>(0);
        
        // Project the robot's state into a 2D space
        projection(0) = r2State->values[0];  
        projection(1) = r2State->values[1];  
    }
};

void carODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control,
            ompl::control::ODESolver::StateType& qdot)
{
        // Extract control inputs: angular velocity and forward acceleration.
        const double* velocity = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        // Extract the car's current state variables.
        const double carOrientation = q[2];
        const double carVelocity = q[3];

        // Initialize the state derivative vector 
        qdot.resize(4);

        // Calculate the time derivatives of the state variables
        qdot[0] = carVelocity * cos(carOrientation); 
        qdot[1] = carVelocity * sin(carOrientation); 
        qdot[2] = velocity[0]; 
        qdot[3] = velocity[1]; 
}

void makeStreet(std::vector<Rectangle>& obstacles)
{
    Rectangle rec1;
    Rectangle rec2;
    Rectangle rec3;

    rec1.x = 0;
    rec1.y = 0;
    rec1.width = 0.02;
    rec1.height = 0.05;

    rec2.x = 4;
    rec2.y = 3;
    rec2.width = 2;
    rec2.height = 2;

    rec3.x = 8;
    rec3.y = 4;
    rec3.width = 1;
    rec3.height = 4;

    obstacles.push_back(rec1);
    //obstacles.push_back(rec2);
    //obstacles.push_back(rec3);
}

void carPostIntegration(const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State* result)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0));
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle>& obstacles)
{
    // Initialize the state spaces
    auto se2Space(std::make_shared<ompl::base::SE2StateSpace>()); 
    auto r1Space(std::make_shared<ompl::base::RealVectorStateSpace>(1)); 
    
    // Set the bounds on the state space
    ompl::base::RealVectorBounds se2Bounds(2);
    se2Bounds.setLow(0.0);
    se2Bounds.setHigh(10.0);
    se2Space->setBounds(se2Bounds);

    ompl::base::RealVectorBounds r1Bounds(1);
    r1Bounds.setLow(-2.0);
    r1Bounds.setHigh(2.0);
    r1Space->as<ompl::base::RealVectorStateSpace>()->setBounds(r1Bounds);

    // Create compound state space
    ompl::base::StateSpacePtr space = se2Space + r1Space;

    // Create a control space
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2)); 

    // Set the bounds on the control space
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(-1.0);
    cbounds.setHigh(1.0);
    cspace->setBounds(cbounds);

    // Assign the simple setup information to the simple setup pointer
    auto ss(std::make_shared<ompl::control::SimpleSetup>(cspace));
    
    // Set state validity checker to include collision checking and bounds checking
    ompl::control::SpaceInformation* si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker(
            [obstacles, si] (const ompl::base::State* state) { 
                return si->satisfiesBounds(state) && isValidStateSquare(state, 0.25, obstacles); }
    ); 

    // Initialze the ODE solver function to use as the state propagator
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));

    // Set the state propagator
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
   
    // Set the propgation step size
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    
    // Add the pendulum projection for KPIECE1
    space->registerDefaultProjection(
         ompl::base::ProjectionEvaluatorPtr(new CarProjection(space.get()))
    );

    // Create the start state
    ompl::base::ScopedState<> start(space);
    start[0] = 1.0; 
    start[1] = 9.0; 
    start[2] = 0.1; 
    start[3] = 0.0; 
    
    // Create the goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 9.0; 
    goal[1] = 1.0;
    goal[2] = 0.0; 
    goal[3] = 0.0;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr& ss, int choice)
{
    // Initialize string variables
    std::string outputMessageSuccess;
    std::string outputMessageFailure;
    std::string filePath;
    
    // choice is what planner to use.
    if (choice == 1)
    {
        //RRT
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));

        // Set custom output messages and file names
        outputMessageSuccess = "Solution Path was found for the car using the RRT planner.";
        outputMessageFailure = "No solution path was found for the car using the RRT planner.";
        filePath = "txt_output/car/RRT.txt"; 

    }
    else if (choice == 2)
    {
        //KPIECE1
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        
        // Set custom output messages and file names
        outputMessageSuccess = "Solution Path was found for the car using the KPIECE1 planner.";
        outputMessageFailure = "No solution path was found for the car using the KPIECE1 planner.";
        filePath = "txt_output/car/KPIECE1.txt"; 
    }
    else if (choice == 3)
    {
        //RG-RRT
        // ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
        
        // Set custom output messages and file names
        outputMessageSuccess = "Solution Path was found for the car using the RG-RRT planner.";
        outputMessageFailure = "No solution path was found for the car using the RG-RRT planner.";
        filePath = "txt_output/car/RGRRT.txt"; 
    }
    
    ss->setup();

    ompl::base::PlannerStatus solved = ss->solve(60.0);
    
    if (solved)
    {
        // Get the geoemtric solution path
        ompl::geometric::PathGeometric pathGeometric = ss->getSolutionPath().asGeometric();
        std::cout << outputMessageSuccess << std::endl;

        // Output geometric solution path to file
        std::ofstream file(filePath);
        pathGeometric.print(file);
    }
    else
    {
        std::cout << outputMessageFailure << std::endl;
    }

}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
