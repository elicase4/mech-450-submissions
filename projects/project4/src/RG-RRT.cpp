///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Santi Parra-Vargas
//////////////////////////////////////

// Our .h file
#include "RG-RRT.h"

// Additional files from OMPL or C++ that might be helpful?
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <limits>

/*
// Constructor
ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
 {
     specs_.approximateSolutions = true;
     siC_ = si.get();
  
     Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
     Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RRT::getIntermediateStates);
 }

 // Destructor 
 ompl::control::RGRRT::~RGRRT()
 {
     freeMemory();
 }
  
 void ompl::control::RGRRT::setup()
 {
     base::Planner::setup();
     if (!nn_)
         nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
     nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
 }
  
// Clear function
 void ompl::control::RGRRT::clear()
 {
     Planner::clear();
     sampler_.reset();
     controlSampler_.reset();
     freeMemory();
     if (nn_)
         nn_->clear();
     lastGoalMotion_ = nullptr;
 }

 
 // Free memory function 
 void ompl::control::RGRRT::freeMemory()
 {
     if (nn_)
     {
         std::vector<Motion *> motions;
         nn_->list(motions);
         for (auto &motion : motions)
         {
             if (motion->state)
                 si_->freeState(motion->state);
             if (motion->control)
                 siC_->freeControl(motion->control);
             delete motion;
         }
     }
 }


Generate Reachable Set Function

This function performs the generation of the reachable set for the planner. This is completely new and separate from 
the rest of the code that is primarily just taken from the RRT file in the OMPL library. This function is called in 
the solve function written below.

 void ompl::control::RGRRT::GRS(Motion* motion){

    // Create vector doubles (called LO and HI) that include the bounds, and find the range of the bounds
    const std::vector<double>& LO = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().low;
    const std::vector<double>& HI = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().high;

    double range = HI[0] - LO[0];

    // Propagate the input states for a range between -10 and 10 over a for loop. 
    for (double propagation = LO[0]; propagation <= HI[0]; propagation += range/10.0)
    {
        Motion *newmotion = new Motion(siC_);
        siC_->copyControl(newmotion->control, motion->control);

        // Set the desired control values and set the first value of that array to the propagation in the loop.
        double *options = motion->control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        options[0] = propagation;

        // Apply the control to the motion. This uses the propagateWhileValid function from the SpaceInformation file in OMPL.
        motion->steps = siC_->propagateWhileValid(motion->state, newmotion->control, 1, newmotion->state);

        // Add the propagation to the reachable motions
        motion->ReachS.push_back(newmotion);
    }
 }

// Main solve function
 ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
 {
     checkValidity();
     base::Goal *goal = pdef_->getGoal().get();
     auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
  
     while (const base::State *st = pis_.nextStart())
     {
         auto *motion = new Motion(siC_);
         si_->copyState(motion->state, st);
         siC_->nullControl(motion->control);

         // NEW: generate the reachable set using the above function
         GRS(motion);
         ///////////////////////////////////////////////////////

         nn_->add(motion);
     }
  
     if (nn_->size() == 0)
     {
         OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
         return base::PlannerStatus::INVALID_START;
     }
  
     if (!sampler_)
         sampler_ = si_->allocStateSampler();
     if (!controlSampler_)
         controlSampler_ = siC_->allocDirectedControlSampler();
  
     OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
  
     Motion *solution = nullptr;
     Motion *approxsol = nullptr;
     double approxdif = std::numeric_limits<double>::infinity();
  
     auto *rmotion = new Motion(siC_);
     base::State *rstate = rmotion->state;
     Control *rctrl = rmotion->control;
     base::State *xstate = si_->allocState();
  
     while (ptc == false)
     {
         // sample random state (with goal biasing)
         if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
             goal_s->sampleGoal(rstate);
         else
             sampler_->sampleUniform(rstate);
  
         // find closest state in the tree
         Motion *nmotion = nn_->nearest(rmotion);
  
         // sample a random control that attempts to go towards the random state, and also sample a control duration 
         unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
  
         if (addIntermediateStates_)
         {
             // this code is contributed by Jennifer Barry
             std::vector<base::State *> pstates;
             cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
  
             if (cd >= siC_->getMinControlDuration())
             {
                 Motion *lastmotion = nmotion;
                 bool solved = false;
                 size_t p = 0;
                 for (; p < pstates.size(); ++p)
                 {
                     // create a motion
                     auto *motion = new Motion();
                     motion->state = pstates[p];
                     // we need multiple copies of rctrl
                     motion->control = siC_->allocControl();
                     siC_->copyControl(motion->control, rctrl);
                     motion->steps = 1;
                     motion->parent = lastmotion;
                     lastmotion = motion;
                     // NEW: generate the reachable set using the above function
                    GRS(motion);
                    ///////////////////////////////////////////////////////
                     nn_->add(motion);
                     double dist = 0.0;
                     solved = goal->isSatisfied(motion->state, &dist);
                     if (solved)
                     {
                         approxdif = dist;
                         solution = motion;
                         break;
                     }
                     if (dist < approxdif)
                     {
                         approxdif = dist;
                         approxsol = motion;
                     }
                 }
  
                 // free any states after we hit the goal
                 while (++p < pstates.size())
                     si_->freeState(pstates[p]);
                 if (solved)
                     break;
             }
             else
                 for (auto &pstate : pstates)
                     si_->freeState(pstate);
         }
         else
         {
             if (cd >= siC_->getMinControlDuration())
             {
                 // create a motion 
                 auto *motion = new Motion(siC_);
                 si_->copyState(motion->state, rmotion->state);
                 siC_->copyControl(motion->control, rctrl);
                 motion->steps = cd;
                 motion->parent = nmotion;
                 // NEW: generate the reachable set using the above function
                GRS(motion);
                ///////////////////////////////////////////////////////
                 nn_->add(motion);
                 double dist = 0.0;
                 bool solv = goal->isSatisfied(motion->state, &dist);
                 if (solv)
                 {
                     approxdif = dist;
                     solution = motion;
                     break;
                 }
                 if (dist < approxdif)
                 {
                     approxdif = dist;
                     approxsol = motion;
                 }
             }
         }
     }
  
     bool solved = false;
     bool approximate = false;
     if (solution == nullptr)
     {
         solution = approxsol;
         approximate = true;
     }
  
     if (solution != nullptr)
     {
         lastGoalMotion_ = solution;
  
         // construct the solution path
         std::vector<Motion *> mpath;
         while (solution != nullptr)
         {
             mpath.push_back(solution);
             solution = solution->parent;
         }
  
         // set the solution path
         auto path(std::make_shared<PathControl>(si_));
         for (int i = mpath.size() - 1; i >= 0; --i)
             if (mpath[i]->parent)
                 path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
             else
                 path->append(mpath[i]->state);
         solved = true;
         pdef_->addSolutionPath(path, approximate, approxdif, getName());
     }
  
     if (rmotion->state)
         si_->freeState(rmotion->state);
     if (rmotion->control)
         siC_->freeControl(rmotion->control);
     delete rmotion;
     si_->freeState(xstate);
  
     OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
  
     return {solved, approximate};
 }
  
 void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
 {
     Planner::getPlannerData(data);
  
     std::vector<Motion *> motions;
     if (nn_)
         nn_->list(motions);
  
     double delta = siC_->getPropagationStepSize();
  
     if (lastGoalMotion_)
         data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
  
     for (auto m : motions)
     {
         if (m->parent)
         {
             if (data.hasControls())
                 data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                              control::PlannerDataEdgeControl(m->control, m->steps * delta));
             else
                 data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
         }
         else
             data.addStartVertex(base::PlannerDataVertex(m->state));
     }
 }

*/
