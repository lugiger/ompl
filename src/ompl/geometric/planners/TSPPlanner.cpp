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

#include "ompl/geometric/planners/TSPPlanner.h"

#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include <fstream>

#include "LKH.h"
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <boost/thread.hpp>
#include <ompl/geometric/PathSimplifier.h>

ompl::geometric::TSPPlanner::TSPPlanner (const ompl::base::SpaceInformationPtr &si) :
    ompl::base::Planner(si, "TSPPlanner")
    
{
    pstype_ = NOSIMPLIFIER;
    statesSet = false;
    useMotionValidator = true;
    specs_.approximateSolutions = true;
    specs_.multithreaded = true;
    specs_.optimizingPaths = true;
    localPlanner_ = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
    localPlanningTime_ = 1.0; 
    clearLocalPlannerForEachIteration = true;
    /*Planner::declareParam<bool>("skip_invalid_states", this, &TSPPlanner::setShortcut, &TSPPlanner::isShortcutting, "0,1");
    Planner::declareParam<bool>("hybridize", this, &TSPPlanner::setHybridize, &TSPPlanner::isHybridizing, "0,1");
    Planner::declareParam<unsigned int>("max_hybrid_paths", this, &TSPPlanner::setMaxHybridizationPath, &TSPPlanner::maxHybridizationPaths, "0:1:50");
    Planner::declareParam<unsigned int>("num_planners", this, &TSPPlanner::setDefaultNumPlanners, &TSPPlanner::getDefaultNumPlanners, "0:64");*/

    /*addPlannerProgressProperty("best cost REAL",
                               boost::bind(&TSPPlanner::getBestCost, this));*/
}

ompl::geometric::TSPPlanner::~TSPPlanner()
{
}

void ompl::geometric::TSPPlanner::setLocalPlanner(base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != si_.get())
    {
        OMPL_ERROR("NOT adding local planner: SpaceInformation instances are different", planner->getName().c_str());
        return;
    }


    localPlanner_= planner;
}

void ompl::geometric::TSPPlanner::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    ompl::base::Planner::setProblemDefinition(pdef);
}

ompl::base::PlannerStatus ompl::geometric::TSPPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    //base::Goal *goal = pdef_->getGoal().get();
    //std::vector<boost::thread*> threads(planners_.size());
	bool alwaysExact = true;
    ompl::geometric::PathSimplifierPtr psk_(new ompl::geometric::PathSimplifier(si_));
    base::ProblemDefinitionPtr localPdef(new base::ProblemDefinition(si_));
    base::MotionValidatorPtr mot = si_->getMotionValidator();
    base::StateValidityCheckerPtr sta = si_->getStateValidityChecker();
    base::OptimizationObjectivePtr opt = pdef_->getOptimizationObjective();
    localPdef->setOptimizationObjective(opt);
    Distance = 0;
    if (!opt)
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
        opt.reset(new base::PathLengthOptimizationObjective(si_));
        pdef_->setOptimizationObjective(opt);
    }
    else
    {
        if (!dynamic_cast<base::PathLengthOptimizationObjective*>(opt.get()))
            OMPL_WARN("The optimization objective is not set for path length.  The specified optimization criteria may not be optimized over.");
    }

    // Disable output from the motion planners, except for errors
    //msg::LogLevel currentLogLevel = msg::getLogLevel();
    // msg::setLogLevel(std::max(msg::LOG_ERROR, currentLogLevel));

    std::string cmatrix = "\n";
    std::stringstream cs;
    int maxID = states_.size();
    std::vector<ompl::geometric::PathGeometric> pathMatrix;
    
    for (int i = 0; i< maxID*maxID; i++)
    {
    
    pathMatrix.push_back(ompl::geometric::PathGeometric(si_));
	}	
    OMPL_INFORM("Starting to calculate Cost Matrix");
    for(int i = 0; i< maxID-1; i++)
        for(int j = i+1; j < maxID; j++)
        {
          	cs.str(std::string());
          	if(useMotionValidator){
          		if(mot->checkMotion(states_[i],states_[j])){
          		
            	
            		cs << ((int) 100*(opt->motionCost(states_[i],states_[j]).value()));
            		cmatrix += cs.str();
            		cmatrix += "\n";
            	
            		pathMatrix[i*maxID + j].append(ompl::geometric::PathGeometric(si_,states_[i],states_[j]));
            	
            		pathMatrix[j*maxID + i].append(ompl::geometric::PathGeometric(si_,states_[j],states_[i]));
            	}
                else{
                    localPdef->clearSolutionPaths();
                    localPdef->clearStartStates();
                    localPdef->clearGoal();
                    localPdef->setStartAndGoalStates(states_[i],states_[j]);
                    if(clearLocalPlannerForEachIteration)
                        localPlanner_->clear();
                    localPlanner_->setProblemDefinition(localPdef);
                
                    if(localPlanner_->solve(localPlanningTime_))
                    {   
                        switch(pstype_){

                            case NOSIMPLIFIER:
                                pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                                pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                                pathMatrix[j*maxID + i].reverse();
                                cs <<  ((int) 100*(localPdef->getSolutionPath()->cost(opt).value()));
                                alwaysExact = !localPdef->hasApproximateSolution();
                                std::cout<< "Cost local: " << cs.str() << " ";
                                cmatrix += cs.str();
                                cmatrix += "\n";
                            break;
                            case SHORTCUTTING:
                                pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                                psk_->shortcutPath(pathMatrix[i*maxID + j], 5, 5, 0.33, 0.005);
                                pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                                pathMatrix[j*maxID + i].reverse();
                                cs <<  ((int) 100*(pathMatrix[i*maxID + j].cost(opt).value()));
                                alwaysExact = !localPdef->hasApproximateSolution();
                                std::cout<< "Cost local: " << cs.str() << " ";
                                cmatrix += cs.str();
                                cmatrix += "\n";
                            break;
                            case SMOOTHING:
                                pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                                psk_->smoothBSpline(pathMatrix[i*maxID + j]);
                                pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                                pathMatrix[j*maxID + i].reverse();
                                cs <<  ((int) 100*(pathMatrix[i*maxID + j].cost(opt).value()));
                                alwaysExact = !localPdef->hasApproximateSolution();
                                std::cout<< "Cost local: " << cs.str() << " ";
                                cmatrix += cs.str();
                                cmatrix += "\n";
                            break;
                            default:
                                OMPL_INFORM("NOT A VALID SIMPLIFIER CHOSEN!");
                            };

                    }           


                    else{
                        cs << 10000000;
                        std::cout<< "Cost local: " << cs.str() << " ";
                        cmatrix += cs.str();
                        cmatrix += "\n";
                    }
                }
            }
          	else
          	{   localPdef->clearSolutionPaths();
          		localPdef->clearStartStates();
          		localPdef->clearGoal();
            	localPdef->setStartAndGoalStates(states_[i],states_[j]);
                if(clearLocalPlannerForEachIteration)
         		localPlanner_->clear();
            	localPlanner_->setProblemDefinition(localPdef);
            	
            	if(localPlanner_->solve(localPlanningTime_))
            	{   
                    switch(pstype_){

                        case NOSIMPLIFIER:
                            pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                            pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                            pathMatrix[j*maxID + i].reverse();
                            cs <<  ((int) 100*(localPdef->getSolutionPath()->cost(opt).value()));
                            alwaysExact = !localPdef->hasApproximateSolution();
                            std::cout<< "Cost local: " << cs.str() << " ";
                            cmatrix += cs.str();
                            cmatrix += "\n";
                        break;
                        case SHORTCUTTING:
                            pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                            psk_->shortcutPath(pathMatrix[i*maxID + j], 5, 5, 0.33, 0.005);
                            pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                            pathMatrix[j*maxID + i].reverse();
                            cs <<  ((int) 100*(pathMatrix[i*maxID + j].cost(opt).value()));
                            alwaysExact = !localPdef->hasApproximateSolution();
                            std::cout<< "Cost local: " << cs.str() << " ";
                            cmatrix += cs.str();
                            cmatrix += "\n";
                        break;
                        case SMOOTHING:
                            pathMatrix[i*maxID + j].append(*(boost::static_pointer_cast<ompl::geometric::PathGeometric>(localPdef->getSolutionPath())));
                            psk_->smoothBSpline(pathMatrix[i*maxID + j]);
                            pathMatrix[j*maxID + i].append(pathMatrix[i*maxID + j]);
                            pathMatrix[j*maxID + i].reverse();
                            cs <<  ((int) 100*(pathMatrix[i*maxID + j].cost(opt).value()));
                            alwaysExact = !localPdef->hasApproximateSolution();
                            std::cout<< "Cost local: " << cs.str() << " ";
                            cmatrix += cs.str();
                            cmatrix += "\n";
                        break;
                        default:
                            OMPL_INFORM("NOT A VALID SIMPLIFIER CHOSEN!");
                        };

                }           


                else{
                    cs << 10000000;
                    std::cout<< "Cost local: " << cs.str() << " ";
                    cmatrix += cs.str();
                    cmatrix += "\n";
                }
          	}

          	

        }
       
        
       	std::cout << cmatrix << std::endl;
       /* // Clear any previous planning data for the set of planners
        clear();

        // Spawn a thread for each planner.  This will shortcut the best path after solving.
        for (size_t i = 0; i < threads.size(); ++i)
            threads[i] = new boost::thread(boost::bind(&TSPPlanner::threadSolve, this, planners_[i].get(), ptc));
		
        // Join each thread, and then delete it
        for (std::size_t i = 0 ; i < threads.size() ; ++i)
        {
            threads[i]->join();
            delete threads[i];
        }*/


    /* ------------- TSP -------------- */

    double ** dummyvector = new double* [maxID];
    

    //use provided interface of the TSP solver 
    std::string params = "MOVE_TYPE=5\n";
    //params += "PRECISION=1\n";
    params += "PATCHING_C=3\n";
    params += "PATCHING_A=2\n";
    params += "RUNS=1\n";
    params += "TIME_LIMIT=5\n";
    params += "TRACE_LEVEL=1\n";
    params += "OUTPUT_TOUR_FILE=tempTour.txt\n";
    params += "EOF";

    std::string prob = "NAME:inspection\n";

    prob += "TYPE:TSP\n";
    prob += "EDGE_WEIGHT_TYPE:EXPLICIT\n";
    prob += "EDGE_WEIGHT_FORMAT:UPPER_ROW\n";

    std::stringstream ss; ss<<(maxID);
    prob += "DIMENSION:"+ss.str()+"\n";
    prob += "EDGE_WEIGHT_SECTION:"+cmatrix;
    /*if(endPoint)
      prob += "FIXED_EDGES_SECTION\n"+ss.str()+" 1\n-1\n";
    prob += "NODE_COORD_SECTION\n";*/
    prob += "EOF";


    size_t length = params.length();
    char * par;
    assert(par = (char*) malloc(length+10));
    strcpy(par, params.c_str());
    length = prob.length();
    char * pro;
    assert(pro = (char*) malloc(length+10));
    strcpy(pro, prob.c_str());
    /* call TSP solver */
    LKHmainFunction(maxID,dummyvector,par,pro);

    delete[] dummyvector;
    
   
  

  

    //GET BEST TOUR
    int i, j, Forwards;

    std::vector<int> Tour;

    for (i = 1; i < maxID && BestTour[i] != 1; i++);
        Forwards = BestTour[i < maxID ? i + 1 : 1] < BestTour[i > 1 ? i - 1 : maxID];
    for (j = 1; j <= maxID; j++) {
        std::cout << BestTour[i] << " ";
        Tour.push_back(BestTour[i]);
        if (Forwards) {
            if (++i > maxID)
                i = 1;
        } else if (--i < 1)
            i = maxID;
    }

    
    std::cout << "\nBest Cost " << BestCost << "\n";
    

   	std::cout <<"Generating Final Path" << std::endl;
  ompl::geometric::PathGeometric * finalPath = new ompl::geometric::PathGeometric(si_);
  for( int i = 0; i < maxID-1;i++){
  
    for (unsigned int j = 0; j<pathMatrix[(Tour[i]-1)*maxID+(Tour[i+1]-1)].getStateCount()-1;j++)
    finalPath->append(pathMatrix[(Tour[i]-1)*maxID+(Tour[i+1]-1)].getStates()[j+1]);
    
  }

  std::cout << "PathLength is :" << finalPath->length() << "\n";
  std::cout << "StateCount is :" << finalPath->getStateCount() << "\n";
  
        
    ptc.terminate();
  	ompl::base::Cost cost= ompl::base::Cost(BestCost);
  	
    ompl::base::PathPtr path(finalPath);
        // Add the solution path.
    ompl::base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    psol.setOptimized(opt, cost, true);
    pdef_->addSolutionPath(psol);
    

       		  if(alwaysExact){
  		  	
            return ompl::base::PlannerStatus::EXACT_SOLUTION;}
        return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;




    
}

/*void ompl::geometric::TSPPlanner::threadSolve(base::Planner* planner, const base::PlannerTerminationCondition &ptc)
{
    // compute a motion plan
    base::PlannerStatus status = planner->solve(ptc);

    // Shortcut the best solution found so far
    if (shortcut_ && status == base::PlannerStatus::EXACT_SOLUTION)
    {
        geometric::PathGeometric* sln = static_cast<geometric::PathGeometric*>(pdef_->getSolutionPath().get());
        geometric::PathGeometric* pathCopy = new geometric::PathGeometric(*sln);
        geometric::PathSimplifier ps(pdef_->getSpaceInformation());
        if (ps.shortcutPath(*pathCopy))
        {
            double difference = 0.0;
            bool approximate = !pdef_->getGoal()->isSatisfied(pathCopy->getStates().back(), &difference);
            pdef_->addSolutionPath(base::PathPtr(pathCopy), approximate, difference);
        }
        else delete pathCopy;
    }
}*/

void ompl::geometric::TSPPlanner::clear(void)
{
    Planner::clear();
    //for (size_t i = 0; i < planners_.size(); ++i)
        localPlanner_->clear();
}



void ompl::geometric::TSPPlanner::setup(void)
{
    Planner::setup();

    /*if (planners_.size() == 0)
    {
        planners_.reserve(defaultNumPlanners_);
        for (unsigned int i = 0; i < defaultNumPlanners_; ++i)
        {
            planners_.push_back(tools::SelfConfig::getDefaultPlanner(pdef_->getGoal()));
            planners_.back()->setProblemDefinition(pdef_);
        }
        OMPL_INFORM("%s: No planners specified; using %u instances of %s",
            getName().c_str(), planners_.size(), planners_[0]->getName().c_str());
    }

    for (size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->setup();*/
}

void ompl::geometric::TSPPlanner::checkValidity(void)
{
    //for (size_t i = 0; i < planners_.size(); ++i)
        localPlanner_->checkValidity();
}



ompl::base::PlannerPtr ompl::geometric::TSPPlanner::getLocalPlanner(void) const
{
    
    return localPlanner_;
}


/*std::string ompl::geometric::TSPPlanner::getBestCost() const
{
    base::Cost bestCost(std::numeric_limits<double>::quiet_NaN());
    if (pdef_ && pdef_->getSolutionCount() > 0)
        bestCost = base::Cost(pdef_->getSolutionPath()->length());
    return boost::lexical_cast<std::string>(bestCost);
}*/
double ompl::geometric::TSPPlanner::getLocalPlanningTime(void) const
{
 return localPlanningTime_;
}

void  ompl::geometric::TSPPlanner::setLocalPlanningTime(double time)
{
  localPlanningTime_ = time;
}

void  ompl::geometric::TSPPlanner::setStates(std::vector< base::State * > states)
{
  states_ = states;
  statesSet = true;
}