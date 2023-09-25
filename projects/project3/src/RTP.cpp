///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////

/* 
Goal of Project 3: Implement RTP as described

Most recent changes: 
Santi -- added the files to the repo, started looking at project details. 9/19/23 1:45pm
Eli -- reorganized file tree to match the makefile. 9/24/23 - 4:15 pm
Added information from RRT.cpp and edited down for our use. Further testing needed. -- Santi, 9/25/23, 5:21pm
All changes made (also applied to the .h file):
    - renamed all Motion variables to Nodes (for easier visualization per piazza suggestions)
    - renamed RRT --> RTP
    - removed all instances of intermediate states, since our planner doesn't use that.
*/

/* Instructions
Implement RTP for rigid body motion planning. 

At a minimum, your planner must derive from ompl::base::Planner and correctly implement the 
solve(), clear(), and getPlannerData() functions. Solve should emit an exact solution path 
when one is found. If time expires, solve should also emit an approximate path that ends at 
the closest state to the goal in the tree. Your planner does not need to know the geometry 
of the robot or the environment, or the exact C-space it is planning in. These concepts are 
abstracted away in OMPL so that planners can be implemented generically.*/

#include "RTP.h" // I assume the two files are in the src file.

// I assume these files remain the same...?
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>


ompl::control::RTP::RTP(const SpaceInformationPtr &si) : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
}

ompl::control::RTP::~RTP()
{
    freeMemory();
}

void ompl::control::RTP::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Node *>(this));
    nn_->setDistanceFunction([this](const Node *a, const Node *b) { return distanceFunction(a, b); });
}

void ompl::control::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalNode_ = nullptr;
}

void ompl::control::RTP::freeMemory()
{
    if (nn_)
    {
        std::vector<Node *> Nodes;
        nn_->list(Nodes);
        for (auto &Node : Nodes)
        {
            if (Node->state)
                si_->freeState(Node->state);
            if (Node->control)
                siC_->freeControl(Node->control);
            delete Node;
        }
    }
}

ompl::base::PlannerStatus ompl::control::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    // check if desired end position exists
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // initialize nodes
    while (const base::State *st = pis_.nextStart())
    {
        auto *node = new Node(siC_);
        si_->copyState(node->state, st);
        siC_->nullControl(node->control);
        nn_->add(node);
    }

    // if no valid initial state is found, return the error status.
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // no clue what this does lol
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // generate solution and approximate solution nodes
    Node *solution = nullptr;
    Node *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rnode = new Node(siC_);
    base::State *rstate = rnode->state;
    Control *rctrl = rnode->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        
        
        
        if (cd >= siC_->getMinControlDuration())
        {
            /* create a motion */
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;

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

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rnode->state)
        si_->freeState(rnode->state);
    if (rnode->control)
        siC_->freeControl(rnode->control);
    delete rnode;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}
