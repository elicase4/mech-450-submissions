///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Eli Case, Santi Parra-Vargas, Jason Ye
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

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

// Global constant for gravitational acceleration
#define G 9.81

// Global constant for pi
#define PI 3.14159

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace* space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Set the dimension of the projection space to 2 for R^2.
        return 2;
    }

    void project(const ompl::base::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Extract state values from state space object
        const double* state_values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        // Set the projection components equal to the state variable compoenents.
        projection(0) = state_values[0];
        projection(1) = state_values[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot)
{
    // Extract control values from control space object
    const double* u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;  
   
    // Write the ODE into qdot
    qdot.resize(2);  
    qdot[0] = q[1];
    qdot[1] = -G*cos(q[1]) + u[0];   
}

// State validity checker
bool isStateValid(const ompl::control::SpaceInformation* si, const ompl::base::State* state)
{
    // Extract state values from State pointer
    const double* state_values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   
    // Enforce that omega and torque be within limits 
    return si->satisfiesBounds(state) && abs(state_values[1]) <= 10.0;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Create a state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    // Set the bounds on the state space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-15.0);
    bounds.setHigh(15.0);
    space->setBounds(bounds);

    // Create a control space
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 1)); 

    // Set the bounds on the control space
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    // Initialize simple setup information
    ompl::control::SimpleSetup ss(cspace);
    
    // Set state validity checker the omega bounds of [-10, 10] since there are no environment obstacles
    ompl::control::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
            [si](const ompl::base::State* state) {return isStateValid(si, state);}
    ); 

    // Initialze the ODE solver function to use as the state propagator
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE);

    // Set the state propagator
    ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // Set the propgation step size
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    
    // Add the pendulum projection for KPIECE1
    ss.getStateSpace()->registerDefaultProjection(
         ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(ss.getStateSpace().get()))
    );
    
    // Create the start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -0.5*PI; 
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.0; 
    
    // Create the goal state
    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.5*PI; 
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    // Assign the simple setup information to the simple setup pointer
    auto ssPtr = std::make_shared<ompl::control::SimpleSetup>(ss);

    return ssPtr;
}

void planPendulum(ompl::control::SimpleSetupPtr& ss, int choice)
{
    // Print the problem settings
    ss->print(std::cout);

    // Initialize planner variable
    ompl::base::PlannerPtr planner;

    // Initialize string variables
    std::string outputMessageSuccess;
    std::string outputMessageFailure;
    std::string filePath;

    // Switch over planner choices
    switch (choice)
    {
        // Use RRT planner 
        case 1:
            {
                // Instantiate the RRT Planner
                planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
                                
                // Set custom output messages and file names
                outputMessageSuccess = "Solution Path was found for the pendulum using the RRT planner.";
                outputMessageFailure = "No solution path was found for the pendulum using the RRT planner.";
                filePath = "txt_output/pendulumRRT.txt";

                break;
            } 

        // Use KPIECE1 planner
        case 2:
            {
                // Instantiate the KPIECE1 Planner
                planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());

                // Set custom output messages and file names
                outputMessageSuccess = "Solution Path was found for the pendulum using the KPIECE1 planner.";
                outputMessageFailure = "No solution path was found for the pendulum using the KPIECE1 planner.";
                filePath = "txt_output/pendulumKPIECE1.txt";
                
                break;
            }

        // Use RG-RRT planner
        case 3:
            {
                // Note to implement RG-RRT later
                std::cout << "RG-RRT Planner to be implemented after checkpoint 1." << std::endl;

                break;
            }
       
        // Input the planner information into simple setup
        ss->setPlanner(planner);

        // Setup the problem
        ss->setup();

        // Request to solve the planning problem within 240s of planning time
        ompl::base::PlannerStatus solved = ss->solve(240.0);

        // Output solution path if solved
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
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
