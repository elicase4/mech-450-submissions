///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Santi Parra-Vargas
//////////////////////////////////////

/*
The following code builds the RG-RRT planner.

It follows the general structure of the RTP planner used in Project 3, but tailored for Project 4.

Change log:

    -- Built the basis using RRT and minor edits. - Santi, 10/22/23 6 pm

*/

#ifndef RGRRT_H
#define RGRRT_H

// Include the base planner class
#include <ompl/control/planners/PlannerIncludes.h>

// Include the nearest neighbors code 
#include <ompl/datastructures/NearestNeighbors.h>

namespace ompl
{
    namespace control
    {
        class RGRRT : public base::Planner
        {

        public:
            // Basic constructor and destructor overrides
            RGRRT(const SpaceInformationPtr &si);
            ~RGRRT() override;

            // Define the main "solve" function with the termination condition being the time spent.
            base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            // Define the clear function
            void clear() override;

            // Set the goal bias. Usually between 0.0 and 1.0, and ideally around 0.05 (per the Github RRT files).
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            // Get the goal bias from the planner
            double getGoalBias() const
            {
                return goalBias_;
            }

            /*
            Because we're using intermediate states and nearest neighbors for this project (vs project 3, where we 
            removed all instances of intermediate states), this is the first point that truly differs from the last project.
            The bulk of this code just comes straight from the OMPL RRT code on the website and Github.
            */
            
            // Get any added intermediate states that are to be added to the tree itself.
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            // Add any intermediate states to the tree if they are to be added.
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            // Grab planner data
            void getPlannerData(base::PlannerData &data) const override;

            // Set nearest neighbors datastructure
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                // Warn that existing set will be deleted. Clear any existing data.
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Nearest Neighbors will now clear all states to call the set.");
                clear();

                // Create new datastructure and set it up.
                nn_ = std::make_shared<NN<Motion*>>();
                setup();
            }

            // Call the setup
            void setup() override;


        // The public definitions end here.

        protected:

            // Create the motion representation
            class Motion
            {
                public:
                    Motion() = default;

                    // Constructor for the motion
                    Motion(const SpaceInformation *si)
                        : state(si->allocState()), control(si->allocControl())
                    {
                    }

                    ~Motion() = default;

                    // Create the state contained in the motion
                    base::State *state{nullptr};

                    // Create the control contained in the motion
                    Control* control{nullptr};

                    // Create the number of steps this control will be applied for
                    unsigned int steps{0};

                    // Create the parent motion in the exploration tree
                    Motion *parent{nullptr};

                    // Create the reachable set (this is what separates RGRRT from the "normal" RRT!!)
                    std::vector<Motion *> ReachS;
            };

            // Free the planner's memory after
            void freeMemory();

            // Compute distances between motions 

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state,b->state);
            }

            // State sampler
            base::StateSamplerPtr sampler_;

            // Control sampler
            DirectedControlSamplerPtr controlSampler_;

            const SpaceInformation* siC_;

            // A nearest-neighbors datastructure containing the tree of motions 
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            // set the goal bias
            double goalBias_{0.05};

            // Define whether we want to add any intermediate states to the built motion tree. Here, we set it to false.
            bool addIntermediateStates_{false};

            // Find the latest goal motion.
            Motion* lastGoalMotion_{nullptr};

            // Random number generator
            RNG rng_;

            // Generate the reachable set (NEW)
            void GRS(Motion* const motion);

            // Select a reachable motion (NEW)
            int selectReachableMotion(const Motion* nearmotion, const Motion* randmotion);
        };
         
    }  // namespace control 
}  // namespace ompl

#endif
