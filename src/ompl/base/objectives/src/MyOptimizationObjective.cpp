/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

#include "ompl/base/objectives/MyOptimizationObjective.h"
#include "ompl/base/spaces/SE3StateSpace.h"

ompl::base::MyOptimizationObjective::
MyOptimizationObjective(const SpaceInformationPtr &si) :
    ompl::base::OptimizationObjective(si)
{
    description_ = "Path Length for SE3 State";
}

ompl::base::Cost ompl::base::MyOptimizationObjective::stateCost(const State *s) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::MyOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
     // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *se3state1 = s1->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *pos1 = se3state1->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *se3state2 = s2->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *pos2 = se3state2->as<ompl::base::RealVectorStateSpace::StateType>(0);

    double distance = sqrt((pos1->values[0]-pos2->values[0])*(pos1->values[0]-pos2->values[0])+(pos1->values[1]-pos2->values[1])*(pos1->values[1]-pos2->values[1])+(pos1->values[2]-pos2->values[2])*(pos1->values[2]-pos2->values[2]));

    return ompl::base::Cost(distance);
}

ompl::base::Cost ompl::base::MyOptimizationObjective::motionCostHeuristic(const State *s1, const State *s2) const
{
    return motionCost(s1, s2);
}
