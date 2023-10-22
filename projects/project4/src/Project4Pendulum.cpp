///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Eli Case, Santi Parra-Vargas, Jason Ye
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Headers for state space and control space representations
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Global constant for gravitational acceleration
#define G 9.81

// Global constant for pi
#define PI 3.14159

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
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

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control* control, ompl::control::ODESolver::StateType &qdot)
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
   
    // Enforce that omega be within limits 
    return si->satisfiesBounds(state) && abs(state_values[1]) <= 10.0;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for planning.
   
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
            [si](const ompl::base::State* state) {return isStateValid(si, state); }); 
    
    // Assign the simple setup information to the simple setup pointer
    ompl::control::SimpleSetupPtr ssPtr = std::make_shared<ompl::control::SimpleSetup>(ss);

    return ssPtr;
}

void planPendulum(ompl::control::SimpleSetupPtr& /*ss*/, int /*choice*/)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
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
