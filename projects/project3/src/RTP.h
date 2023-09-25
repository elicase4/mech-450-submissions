///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Santi Parra-Vargas, Eli Case
//////////////////////////////////////


/*

Change log:

Added lines from RTT.h and edited down to use. Further testing needed -- Santi, 9/25/23 4:26pm

*/

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        // the following is taken from RRT.h and edited down for our use.

          class RTP : public base::Planner
        {

            public:

                // constructor
                RTP(const SpaceInformationPtr &si);

                ~RTP() override;

                // solve for predetermined timeframe. return true if exact solution found.
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                void clear() override;

                void SetGoalBias(double goalBias)
                {
                    goalBias_ = goalBias;
                }

                double getGoalBias() const
                {
                    return goalBias_;
                }

                void getPlannerData(base::Plannerdata &data) const override;

                // nearest neighbors structure is skipped because rtp doesn't use that.

                void setup() override;

            projected:

                class Node
                {
                    public:
                        Node() = default;

                        Node(const SpaceInformation *si)
                            : state(si->allocState()), control(si->allocControl())
                            {}

                        ~Node() = default;

                        base::State *state{nullptr};

                        Control *control{nullptr};

                        unsigned int steps{0};

                        Node *parent{nullptr};

                }; // Node class

                void freeMemory();

                double distanceFunction(const Node *a, const Node *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                base::StateSamplerPtr sampler_;

                DirectedControlSamplerPtr controlSampler;

                const SpaceInformation *siC_;

                double goalBias_{0.05};

                RNG rng_;

                Node *lastGoalNode_{nullptr};
        }

    }  // namespace geometric
}  // namespace ompl

#endif

