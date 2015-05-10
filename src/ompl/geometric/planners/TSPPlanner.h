/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Lucas Giger */

#ifndef OMPL_GEOMETRIC_PLANNERS_TSPPLANNER_
#define OMPL_GEOMETRIC_PLANNERS_TSPPLANNER_

#include "ompl/base/Planner.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        
        class TSPPlanner : public base::Planner
        {
        public:
            /// \brief Constructor requires the space information to plan in
            TSPPlanner(const base::SpaceInformationPtr &si);

            /// \brief Destructor
            virtual ~TSPPlanner(void);

            /// \brief Adds the given planner to the set of planners used to
            /// compute candidate paths.
            void setLocalPlanner(base::PlannerPtr &planner);

            /// \brief Set the problem definition for the planners. The
            /// problem needs to be set before calling solve(). Note:
            /// If this problem definition replaces a previous one, it
            /// may also be necessary to call clear().
            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            /// \brief Method that solves the motion planning problem.  This method
            /// terminates under just two conditions, the given argument condition,
            /// or when the maximum path length in the optimization objective is met.
            /// \remarks This method spawns a separate thread for each planner
            /// employed, and applies a series of optimization methods to the set
            /// of paths generated by each planning thread.
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /// \brief Clear all internal planning datastructures. Planner
            /// settings are not affected. Subsequent calls to solve()
            /// will ignore all previous work.
            virtual void clear(void);

            /// \brief Get information about the most recent run of the motion planner.
            /// \remarks This call is ambiguous in this tool.  By default, the
            /// planner data for the first planner in the planner list is returned.
            /*virtual void getPlannerData(base::PlannerData &data) const;*/

            /// \brief Perform any necessary configuration steps.  This method
            /// also invokes ompl::base::SpaceInformation::setup() if needed. This
            /// must be called before solving
            virtual void setup(void);

            /// \brief Check to see if the planner is in a working
            /// state (setup has been called, a goal was set, the
            /// input states seem to be in order). In case of error,
            /// this function throws an exception.
            virtual void checkValidity(void);


            /// \brief Retrieve a pointer to the ith planner instance
            base::PlannerPtr getLocalPlanner() const;


            /** \brief Return best cost found so far by algorithm */
            /*std::string getBestCost() const;*/

            double getLocalPlanningTime (void) const;

            void setLocalPlanningTime (double time);

            void setStates(std::vector< base::State * > states);

            void setUsageOfMotionValidator(bool use){
                useMotionValidator = use;
            }


        protected:
            /// \brief The function that the planning threads execute when
            /// solving a motion planning problem.
            /*virtual void threadSolve(base::Planner *planner, const base::PlannerTerminationCondition &ptc);*/

            /// \brief The list of planners used for solving the problem.
            base::PlannerPtr localPlanner_;


            std::vector< base::State * > states_;

            double localPlanningTime_;

            bool statesSet;

            bool useMotionValidator;

        };
    }
}
#endif
