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
#include <ompl/base/spaces/SO2StateSpace.h>

// Headers for RRT and KPIECE planners and projection evaluator
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Header for path and env output
#include <fstream>

// Header to assist file naming convention
#include <string>

// Include math library
#include <cmath>

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
        // Set the dimension of the projection space to 1.
        return 2;
    }

    void project(const ompl::base::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Extract state values from state space object
        auto compoundState = state->as<ompl::base::CompoundState>();
        const double theta = compoundState->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        const double omega = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];

        // Set the projection components equal to the state variable compoenents.
        projection(0) = theta;
        projection(1) = omega;
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot)
{
    // Extract control values from control space object
    const double* u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values; 

    // Write the ODE into qdot
    qdot.resize(2);  
    qdot[0] = q[1];
    qdot[1] = -G*cos(q[0]) + u[0];   
}

void pendulumPostIntegration(const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State* result)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

ompl::control::SimpleSetupPtr createPendulum(double torque, std::string boundsFilePath, std::string startgoalFilePath)
{
    // Create the state spaces
    auto thetaSpace(std::make_shared<ompl::base::SO2StateSpace>());
    auto omegaSpace(std::make_shared<ompl::base::RealVectorStateSpace>(1));

    // Set the bounds on the state space
    ompl::base::RealVectorBounds omegaBounds(1);
    omegaBounds.setLow(-10.0);
    omegaBounds.setHigh(10.0);
    omegaSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(omegaBounds);

    // Create compound state space
    ompl::base::StateSpacePtr space = thetaSpace + omegaSpace;

    // Record the environment bounds to an output file
    std::ofstream boundsFile(boundsFilePath);
    boundsFile << omegaBounds.low[0] << "," << omegaBounds.high[0] << "," << 0.5*omegaBounds.low[0] << "," << 0.5*omegaBounds.high[0] << std::endl;

    // Output the amunt of environment obstacles
    std::cout << "Pendulum environment created using " << 0 << " total obstacles."<< std::endl;

    // Create a control space
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 1)); 

    // Set the bounds on the control space
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    // Assign the simple setup information to the simple setup pointer
    auto ss(std::make_shared<ompl::control::SimpleSetup>(cspace));
    
    // Set state validity checker the omega bounds of [-10, 10] since there are no environment obstacles
    ompl::control::SpaceInformation* si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker(
            [si](const ompl::base::State* state) { return si->satisfiesBounds(state); }
    ); 

    // Initialze the ODE solver function to use as the state propagator
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));

    // Set the state propagator
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &pendulumPostIntegration));
   
    // Set the propgation step size
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    
    // Add the pendulum projection for KPIECE1
    space->registerDefaultProjection(
         ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space.get()))
    );
    
    // Create the start state
    ompl::base::ScopedState<> start(space);
    start[0] = -0.5*PI; 
    start[1] = 0.0; 
    
    // Create the goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 0.5*PI; 
    goal[1] = 0.0;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);
    
    // Record the start and goal information to an output file
    std::ofstream startgoalFile(startgoalFilePath);
    startgoalFile << start[1] << "," << start[0] << "," << goal[1] << "," << goal[0] << "," << 0.05 << std::endl;

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr& ss, int choice, std::string geopathFilePath)
{
    // Print the problem settings
    ss->print(std::cout);

    // Initialize string variables
    std::string outputMessageSuccess;
    std::string outputMessageFailure;

    // Switch over planner choices
    switch (choice)
    {
        // Use RRT planner 
        case 1:
        {
            // Instantiate the RRT Planner
            ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
                               
            // Set custom output messages and file names
            outputMessageSuccess = "Solution Path was found for the pendulum using the RRT planner.";
            outputMessageFailure = "No solution path was found for the pendulum using the RRT planner.";

            break;
        } 

        // Use KPIECE1 planner
        case 2:
        {
            // Instantiate the KPIECE1 Planner
            ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));

            // Set custom output messages and file names
            outputMessageSuccess = "Solution Path was found for the pendulum using the KPIECE1 planner.";
            outputMessageFailure = "No solution path was found for the pendulum using the KPIECE1 planner.";
                
            break;
        }

        // Use RG-RRT planner
        case 3:
        {
            // Instantiate the RG-RRT Planner
            // ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));

            // Set custom output messages and file names
            outputMessageSuccess = "Solution Path was found for the pendulum using the RG-RRT planner.";
            outputMessageFailure = "No solution path was found for the pendulum using the RG-RRT planner.";

            break;
        }
    }
       
    // Setup any additional information for the problem
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
        std::ofstream pathFile(geopathFilePath);
        pathGeometric.print(pathFile);
    }
    else
    {
        std::cout << outputMessageFailure << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int argc, char** argv)
{
    // Terminate if the correct number of input arguments are not provided
    if (argc != 4)
    {
        std::cout << "Please provide the file names for the planning path and planning bounds in the format shown below:" << "\n" << "PROGRAM_NAME [file path to geometric path output] [file path to bounds output] [file path to start goal output]" << std::endl;
        exit(1);
    }

    // Initialze input arguments for output text file names
    std::string geopathFilePath(argv[1]);
    std::string boundsFilePath(argv[2]);
    std::string startgoalFilePath(argv[3]);
    
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

    ompl::control::SimpleSetupPtr ss = createPendulum(torque, boundsFilePath, startgoalFilePath);

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

        planPendulum(ss, planner, geopathFilePath);
    }

    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
