///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////

/*

Change log:

Added lines from RTT.h and edited down to use. Further testing needed -- Santi, 9/25/23 4:26pm
Reverted some changes so the file compiles properly. Added comments to each line for understanding. -- Santi, 9/26/23, 10:55am


Notes to the instructors:
=========================
	- The bulk of this code was taken from the available OMPL documentation on Github, per the suggestion in the project file.
	  We made some changes to the code to make it applicable to our application, such as removing all instances of intermediate
	  state calculations, since RTP does not need this.
	- We've made similar changes to the RRT.cpp file to get our RTP.cpp file.
*/
#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

/*This was taken from the RRT files, and is necessary for the base::Planner object*/
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
	namespace geometric
	{

		class RTP : public base::Planner
		{
		public:
			/* Basic constructor and deconstructor overrides*/
			RTP(const base::SpaceInformationPtr &si);

			~RTP() override;

            /*get existing planner data, if any*/
			void getPlannerData(base::PlannerData &data) const override;

            /*Define the main "solve" function, with the PlannerTerminationCondition being the time limit*/
			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /*Define the clear() function */
			void clear() override;

			/* Set the goal bias. Usually between 0.0 and 1.0 and
                ideally around 0.05, per the RRT files provided on Github.*/
			void setGoalBias(double goalBias)
			{
				goalBias_ = goalBias;
			}

			/* Get the goal bias from the planner */
			double getGoalBias() const
			{
				return goalBias_;
			}

			/* Robot range */
			void setRange(double distance)
			{
				maxDistance_ = distance;
			}

			double getRange() const
			{
				return maxDistance_;
			}
			
			void setup() override;

		protected:
			/** \brief Representation of a motion

				This only contains pointers to parent motions as we
				only need to go backwards in the tree. */
			class Motion // A motion contains a current state and its parent motion in the tree, basically an edge in the graph
			{
			public:
				Motion() = default;

				/** \brief Constructor that allocates memory for the state */
				Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
				{
				}

				~Motion() = default;

				/** \brief The state contained by the motion */
				base::State *state{nullptr};

				/** \brief The parent motion in the exploration tree */
				Motion *parent{nullptr};
			};

			/* Free memory */
			// void freeMemory();

			/* Function to find distance between nodes*/
			double distanceFunction(const Motion *a, const Motion *b) const
			{
				return si_->distance(a->state, b->state);
			}

			/* State sampler */
			base::StateSamplerPtr sampler_;

			/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
			 * available) */
			double goalBias_{.05};

			/* Maximum node length. We assume it is extremely small, hence we use 0..*/
			double maxDistance_{0.};

			/* RNG generator*/
			RNG rng_;

			/* Take the most recent goal node */
			Motion *lastGoalMotion_{nullptr};

			/* Create a vector called NodeVec that saves all the nodes in the tree. This was NOT in the original RRT.h file and we added this ourselves.*/
			std::vector<Motion *> NodeVec;
		};
	}  // namespace geometric
}  // namespace ompl

#endif
