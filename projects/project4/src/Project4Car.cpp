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
        auto r2 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        // Project the robot's state into a 2D space
        projection[0] = r2->values[0];  
        projection[1] = r2->values[1];  

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
        qdot.resize(q.size(), 0);

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
    rec1.width = 2;
    rec1.height = 5;

    rec2.x = 3;
    rec2.y = 3;
    rec2.width = 2;
    rec2.height = 2;

    rec3.x = 7;
    rec3.y = 4;
    rec3.width = 1;
    rec3.height = 4;

    obstacles.push_back(rec1);
    obstacles.push_back(rec2);
    obstacles.push_back(rec3);


}

bool isValidStateCar(ompl::control::SpaceInformation* si, const ompl::base::State* state, const std::vector<Rectangle>& obstacles)
{
    // check for collisions
    auto r2 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double x = r2->values[0];
    double y = r2->values[1];
    double orientation = r2->values[2];

    return si->satisfiesBounds(state) && isValidStateSquare(state, 0.25, obstacles);
}

void carPostIntegration(const ompl::base::State* state)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0));
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    ss->getSpaceInformation()->setPropagationStepSize(0.05);

    return nullptr;
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
        filePath = "txt_output/carRRT.txt"; 
    }
    else if (choice == 2)
    {
        //KPIECE1
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        
        // Set custom output messages and file names
        outputMessageSuccess = "Solution Path was found for the car using the KPIECE1 planner.";
        outputMessageFailure = "No solution path was found for the car using the KPIECE1 planner.";
        filePath = "txt_output/carKPIECE1.txt"; 
    }
    else if (choice == 3)
    {
        //RG-RRT
        // ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
        
        // Set custom output messages and file names
        outputMessageSuccess = "Solution Path was found for the car using the RG-RRT planner.";
        outputMessageFailure = "No solution path was found for the car using the RG-RRT planner.";
        filePath = "txt_output/carRGRRT.txt"; 
    }
    
    
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(300.0);
    
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
