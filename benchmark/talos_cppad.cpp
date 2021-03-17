///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////


#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/utils/file-io.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <example-robot-data/path.hpp>

#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/utils/file-io.hpp"

#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/core/codegen/action-base.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/core/costs/control.hpp"

template <typename Scalar>
void build_bipedal_action_models(boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& runningActionModel,
                            boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& terminalActionModel) {
	typedef typename crocoddyl::MathBaseTpl<Scalar>::VectorXs VectorXs;
	typedef typename crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<Scalar>
	  DifferentialActionModelFreeFwdDynamics;
	typedef typename crocoddyl::IntegratedActionModelEulerTpl<Scalar> IntegratedActionModelEuler;
	typedef typename crocoddyl::ActuationModelFullTpl<Scalar> ActuationModelFull;
	typedef typename crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
	typedef typename crocoddyl::CostModelAbstractTpl<Scalar> CostModelAbstract;
	typedef typename crocoddyl::CostModelStateTpl<Scalar> CostModelState;
	typedef typename crocoddyl::CostModelControlTpl<Scalar> CostModelControl;
	
	
	const std::string RF = "right_sole_link";
    const std::string LF = "left_sole_link";
	pinocchio::ModelTpl<double> modeld;
	pinocchio::urdf::buildModel("/opt/openrobots/share/example-robot-data/robots/talos_data/robots/talos_reduced.urdf", modeld);
    std::vector<pinocchio::JointIndex> locked_joints_id;
	pinocchio::ModelTpl<Scalar> model_full(modeld.cast<Scalar>()), model;
	
	// Compute locked joints id
	std::vector<std::string> controlled_joints_names{"arm_left_1_joint",
		                                             "arm_left_2_joint",
		                                             "arm_left_3_joint",
		                                             "arm_left_4_joint"};
    for(std::vector<std::string>::const_iterator it = model_full.names.begin() + 1;it != model_full.names.end(); ++it) {
        const std::string & joint_name = *it; 
        if(std::find(controlled_joints_names.begin(), controlled_joints_names.end(), joint_name) == controlled_joints_names.end()) {
            locked_joints_id.push_back(model_full.getJointId(joint_name));
        }
    }

	pinocchio::buildReducedModel(model_full, locked_joints_id, VectorXs::Zero(model_full.nq), model);
    pinocchio::srdf::loadReferenceConfigurations(model, "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/talos.srdf",false);
    pinocchio::DataTpl<Scalar> data(model);
    // Define default state
    VectorXs default_state(model.nq + model.nv);
    default_state << model.referenceConfigurations["half_sitting"],VectorXs::Zero(model.nv);
	boost::shared_ptr<crocoddyl::StateMultibodyTpl<Scalar> > state =
	  boost::make_shared<crocoddyl::StateMultibodyTpl<Scalar> >(boost::make_shared<pinocchio::ModelTpl<Scalar> >(model));	
	
	boost::shared_ptr<ActuationModelFull> actuation = boost::make_shared<ActuationModelFull>(state);									
	
	// Define regularization costs
	boost::shared_ptr<CostModelAbstract> xRegCost = boost::make_shared<CostModelState>(state,default_state,actuation->get_nu()); 
    boost::shared_ptr<CostModelAbstract> uRegCost = boost::make_shared<CostModelControl>(state,actuation->get_nu());

	// Create a cost model for the action model.
	boost::shared_ptr<CostModelSum> runningCostModel = boost::make_shared<CostModelSum>(state,actuation->get_nu());
	boost::shared_ptr<CostModelSum> terminalCostModel = boost::make_shared<CostModelSum>(state,actuation->get_nu());

	// Then let's added the cost functions
	runningCostModel->addCost("xReg", xRegCost, Scalar(2e-2));
	runningCostModel->addCost("uReg", uRegCost, Scalar(4e-4));
	
	terminalCostModel->addCost("xReg", xRegCost, Scalar(2e-2));

	// Next, we need to create an action model for running and terminal knots. The
	// forward dynamics (computed using ABA) are implemented
	// inside DifferentialActionModelFullyActuated.
	boost::shared_ptr<DifferentialActionModelFreeFwdDynamics> runningDAM =
	   boost::make_shared<DifferentialActionModelFreeFwdDynamics>(state, actuation,runningCostModel);

    boost::shared_ptr<DifferentialActionModelFreeFwdDynamics> terminalDAM =
	   boost::make_shared<DifferentialActionModelFreeFwdDynamics>(state, actuation,terminalCostModel);

	runningActionModel =
	   boost::make_shared<IntegratedActionModelEuler>(runningDAM, Scalar(1e-2));

	terminalActionModel =
	   boost::make_shared<IntegratedActionModelEuler>(terminalDAM, Scalar(1e-2));
}

int main(int argc, char* argv[]) {
	
	typedef CppAD::AD<CppAD::cg::CG<double> > ADScalar;
    
    std::size_t horizon_length = 100;
    boost::shared_ptr<crocoddyl::ActionModelAbstract > runningModelD, terminalModelD;
	boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<ADScalar> > runningModelAD, terminalModelAD;

	build_bipedal_action_models(runningModelD, terminalModelD);
	build_bipedal_action_models(runningModelAD, terminalModelAD);

	boost::shared_ptr<crocoddyl::ActionModelAbstract> runningModelCG = 
       boost::make_shared<crocoddyl::ActionModelCodeGen>(runningModelAD, runningModelD,"pyrene_running");

	boost::shared_ptr<crocoddyl::ActionModelAbstract > terminalModelCG = 
	   boost::make_shared<crocoddyl::ActionModelCodeGen >(terminalModelAD, terminalModelD,"pyrene_running");
    
    Eigen::VectorXd x_rand = runningModelCG->get_state()->rand();
    Eigen::VectorXd u_rand = Eigen::VectorXd::Random(runningModelCG->get_nu());
    
    std::vector<Eigen::VectorXd> x_guess(horizon_length+1, x_rand);
    std::vector<Eigen::VectorXd> u_guess(horizon_length, u_rand);
	//Create Shooting Problem
	std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract > > runningModelsCG(horizon_length, runningModelCG);
	boost::shared_ptr<crocoddyl::ShootingProblem>  shooting_problem = boost::make_shared<crocoddyl::ShootingProblem>(x_rand, runningModelsCG, terminalModelCG);
    boost::shared_ptr<crocoddyl::SolverFDDP>  ddp = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
    
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract > > runningModelsD(horizon_length, runningModelD);
	boost::shared_ptr<crocoddyl::ShootingProblem>  shooting_problemD = boost::make_shared<crocoddyl::ShootingProblem>(x_rand, runningModelsD, terminalModelD);
	boost::shared_ptr<crocoddyl::SolverFDDP>  ddpD = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problemD);

    ddp->solve(x_guess,u_guess,1,false);
    ddpD->solve(x_guess,u_guess,1,false);
    
    std::cout << "ddp codegen cost = " << ddp->get_cost() << std::endl;
    std::cout << "ddp classical cost = " << ddpD->get_cost() << std::endl;
	return 0;
}
