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
Rewrote portions of code to help it compile and reverted some changes. Added comments to each line for understanding. -- Santi, 9/26/23, 1:31pm


Notes to the instructors:
=========================
	- The bulk of this code was taken from the available OMPL documentation on Github, per the suggestion in the project file.
	  We made some changes to the code to make it applicable to our application, such as removing all instances of intermediate
	  state calculations, since RTP does not need this.
	- We don't quite understand absolutely everything that's going on, but we tried our best to comment each line with what we coudl understand.
	- We've made similar changes to the RRT.h file to get our RTP.h file.

All changes made (also applied to the .h file):
    - renamed RRT --> RTP
    - removed all instances of intermediate states, since our planner doesn't use that.
    - added new NodeVec vector variable for node storage
*/

/* Instructions
Implement RTP for rigid body motion planning. 

At a minimum, your planner must derive from ompl::base::Planner and correctly implement the 
solve(), clear(), and getPlannerData() functions. Solve should emit an exact solution path 
when one is found. If time expires, solve should also emit an approximate path that ends at 
the closest state to the goal in the tree. Your planner does not need to know the geometry 
of the robot or the environment, or the exact C-space it is planning in. These concepts are 
abstracted away in OMPL so that planners can be implemented generically.*/

#include "RTP.h"

/* These were taken straight from RRT.cpp, since they're required for certain functions. */
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

/* Define the planner. This was taken from the RRT.cpp file and edited to remove all intermediate states instances. */
ompl::geometric::RTP::RTP(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "RTP") {
	
	// allow approximate solutions
	specs_.approximateSolutions = true;

	// i assume this means allow direct solutions?
	specs_.directed = true;

	// declare the range and goal bias parameters based on pre-determined settings
	Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
	Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");

}

/* Deconstruction and memory freeing */
ompl::geometric::RTP::~RTP() {
	freeMemory();
}

/* Clear function to remove all nodes from memory after the planner is done */
void ompl::geometric::RTP::clear() {
	Planner::clear();
	sampler_.reset();
	freeMemory();
	NodeVec.clear(); 
	lastGoalMotion_ = nullptr;
}

/* Clear the vector that holds the motions of our tree */
void ompl::geometric::RTP::freeMemory()
{
	NodeVec.clear();
}

/* Main solve function*/
ompl::base::PlannerStatus ompl::geometric::RTP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
	// Check the validity of the initial state
	checkValidity();

	// Extract goal node from desired end position based on problem definition
	ompl::base::Goal *goal = pdef_->getGoal().get(); 

	// Cast the goal node to the GoalSampleableRegion
	auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal); 

	// Build initial form of the tree.
	while (const ompl::base::State *st = pis_.nextStart()) 
	{
		// create a node (Motion) pointer, initialize first node with start state
		auto *motion = new Motion(si_); 

		// Copy the state into the SpaceInformation variable
		si_->copyState(motion->state, st); 

		// Add node to tree. push_back is used over append for consistency with other functions.
		NodeVec.push_back(motion); 
	}

	// If there are no nodes in the NodeVec vector, then there are no valid start states. Throw error if so.
	if (NodeVec.size() == 0) 
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return ompl::base::PlannerStatus::INVALID_START;
	}

	// no clue what this does, but it looks like it has to do with allocating memory from the sampler to the SpaceInformation.
	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	// Print out the number of states in the tree when planning begins.
	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), NodeVec.size());

	// Initialize both solution pointers
	Motion *solution = nullptr; 
	Motion *approxsol = nullptr;

	// no clue what this does either. looks like it has to do with finding the difference between the exact solution and approximate solution?
	double approxdif = std::numeric_limits<double>::infinity();

	// create new node in the space
	auto *rmotion = new Motion(si_);
	ompl::base::State *rstate = rmotion->state; // rstate : holds our sampled random state

	while (!ptc)
	{
		/* sample random state (with goal biasing) */

		if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
			goal_s->sampleGoal(rstate); // sample the goal region where possible
		else
			sampler_->sampleUniform(rstate); // if not, then sample a state uniformly

		/* Choose a random state from the tree */
		int randI = std::rand() % NodeVec.size();
		Motion *nmotion = NodeVec[randI];


		/* check if path between two states is valid (takes pointers to State objects)*/
		if (si_->checkMotion(nmotion->state, rstate)) 
		{
				auto *motion = new Motion(si_);        // allocate memory for a state
				si_->copyState(motion->state, rstate); // copy the selected random state into the new node state
				motion->parent = nmotion;              // set the randomly sampled node's parent as the node of interest
				NodeVec.push_back(motion);             // append the new node to the tree

				nmotion = motion;

			/*Check if the goal is reached (ie distance from goal = 0)*/
			double dist = 0.0;
			bool sat = goal->isSatisfied(nmotion->state, &dist);

			/*if statement if exact solution is reached*/
			if (sat) 
			{
				approxdif = dist;
				solution = nmotion;
				break;
			}

			/*if exact solution not reached, check for approximate solution*/
			if (dist < approxdif) 
			{
				approxdif = dist;
				approxsol = nmotion;
			}
		}
	}

	/* set solution booleans to decide which solution is displayed later.*/
	bool solved = false;
	bool approximate = false;

	/* approximate solution */
	if (solution == nullptr)
	{
		solution = approxsol;
		approximate = true;
	}

	if (solution != nullptr) // exact solution
	{
		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<Motion *> mpath; // mpath stores our Motion pointers for the path
		while (solution != nullptr)
		{
			mpath.push_back(solution);
			solution = solution->parent; // Keep moving up the tree of motions until nullptr (root)
		}

		/* set the solution path */
		auto path(std::make_shared<PathGeometric>(si_));
		for (int i = mpath.size() - 1; i >= 0; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(path, approximate, approxdif, getName());
		solved = true;
	}

	/* not sure what this does. my guess is that it has to do with freeing memory if a new node was found after the goal was reached.*/
	if (rmotion->state != nullptr)
		si_->freeState(rmotion->state);
	delete rmotion;


	/* Display result information*/
	OMPL_INFORM("%s: Created %u states", getName().c_str(), NodeVec.size());

	/* return the determined solution. */
	return {solved, approximate};
}

/*Planner setup. I assume most of this happens under the hood from the other .h files and within OMPL?*/
void ompl::geometric::RTP::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);
}


void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	// if the last node, make a vertex to designate the goal node.
	if (lastGoalMotion_ != nullptr)
		data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

	for (auto &motion : NodeVec)
	{
		// create the start node vertex if none has been made already
		if (motion->parent == nullptr)
			data.addStartVertex(ompl::base::PlannerDataVertex(motion->state));
		else
		// create line segments to connect tree portions
			data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state), ompl::base::PlannerDataVertex(motion->state));
	}
}



