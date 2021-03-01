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
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/core/codegen/action-base.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/core/costs/control.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"

template <typename Scalar>
void build_bipedal_action_models(boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& runningActionModel,
                            boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& terminalActionModel) {
	typedef typename crocoddyl::MathBaseTpl<Scalar>::VectorXs VectorXs;
	typedef typename crocoddyl::MathBaseTpl<Scalar>::Vector2s Vector2s;
	typedef typename crocoddyl::FramePlacementTpl<Scalar> FramePlacement;
	typedef typename crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<Scalar>
	  DifferentialActionModelContactFwdDynamics;
	typedef typename crocoddyl::IntegratedActionModelEulerTpl<Scalar> IntegratedActionModelEuler;
	typedef typename crocoddyl::ActuationModelFloatingBaseTpl<Scalar> ActuationModelFloatingBase;
	typedef typename crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
	typedef typename crocoddyl::ContactModelMultipleTpl<Scalar> ContactModelMultiple;
	typedef typename crocoddyl::CostModelAbstractTpl<Scalar> CostModelAbstract;
	typedef typename crocoddyl::ContactModelAbstractTpl<Scalar> ContactModelAbstract;
	typedef typename crocoddyl::CostModelStateTpl<Scalar> CostModelState;
	typedef typename crocoddyl::CostModelControlTpl<Scalar> CostModelControl;
	typedef typename crocoddyl::ContactModel6DTpl<Scalar> ContactModel6D;
	
	
	const std::string RF = "right_sole_link";
    const std::string LF = "left_sole_link";
	pinocchio::ModelTpl<double> modeld;
	pinocchio::urdf::buildModel("/opt/openrobots/share/example-robot-data/robots/talos_data/robots/talos_reduced.urdf",pinocchio::JointModelFreeFlyer(), modeld);
    std::vector<pinocchio::JointIndex> locked_joints_id;
	pinocchio::ModelTpl<Scalar> model_full(modeld.cast<Scalar>()), model;
	
	// Compute locked joints id
	std::vector<std::string> controlled_joints_names{"root_joint",
		                                             "leg_left_1_joint",
		                                             "leg_left_2_joint",
		                                             "leg_left_3_joint",
		                                             "leg_left_4_joint",
		                                             "leg_left_5_joint",
		                                             "leg_left_6_joint",
		                                             "leg_right_1_joint",
		                                             "leg_right_2_joint",
		                                             "leg_right_3_joint",
		                                             "leg_right_4_joint",
		                                             "leg_right_5_joint",
		                                             "leg_right_6_joint",
		                                             "torso_1_joint",
		                                             "torso_2_joint",
		                                             "arm_left_1_joint",
		                                             "arm_left_2_joint",
		                                             "arm_left_3_joint",
		                                             "arm_left_4_joint",
		                                             "arm_right_1_joint",
		                                             "arm_right_2_joint",
		                                             "arm_right_3_joint",
		                                             "arm_right_4_joint"};
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
    VectorXs default_state(model.nq);
    default_state << model.referenceConfigurations["half_sitting"],VectorXs::Zero(model.nv);
	boost::shared_ptr<crocoddyl::StateMultibodyTpl<Scalar> > state =
	  boost::make_shared<crocoddyl::StateMultibodyTpl<Scalar> >(boost::make_shared<pinocchio::ModelTpl<Scalar> >(model));	
	
	boost::shared_ptr<ActuationModelFloatingBase> actuation = boost::make_shared<ActuationModelFloatingBase>(state);									
	
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
    
    // Define contact model
	boost::shared_ptr<ContactModelMultiple> contact_models =
	  boost::make_shared<ContactModelMultiple>(state, actuation->get_nu());
    
    pinocchio::FrameIndex RF_id = model.getFrameId(RF);
	FramePlacement RFref(RF_id, data.oMf[RF_id]);
	boost::shared_ptr<ContactModelAbstract> support_contact_RF =
	  boost::make_shared<ContactModel6D>(state, RFref, actuation->get_nu(), Vector2s(Scalar(0.), Scalar(4.)));
	contact_models->addContact(model.frames[RF_id].name, support_contact_RF);
    
    pinocchio::FrameIndex LF_id = model.getFrameId(LF);
	FramePlacement LFref(LF_id, data.oMf[LF_id]);
	boost::shared_ptr<ContactModelAbstract> support_contact_LF =
	  boost::make_shared<ContactModel6D>(state, LFref, actuation->get_nu(), Vector2s(Scalar(0.), Scalar(4.)));
	contact_models->addContact(model.frames[LF_id].name, support_contact_LF);
	
	// Next, we need to create an action model for running and terminal knots. The
	// forward dynamics (computed using ABA) are implemented
	// inside DifferentialActionModelFullyActuated.
	boost::shared_ptr<DifferentialActionModelContactFwdDynamics> runningDAM =
	   boost::make_shared<DifferentialActionModelContactFwdDynamics>(state, actuation, contact_models,runningCostModel);

    boost::shared_ptr<DifferentialActionModelContactFwdDynamics> terminalDAM =
	   boost::make_shared<DifferentialActionModelContactFwdDynamics>(state, actuation, contact_models,terminalCostModel);

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
       boost::make_shared<crocoddyl::ActionModelCodeGen>(runningModelAD, runningModelD,"pyrene_running_in_contact");

	boost::shared_ptr<crocoddyl::ActionModelAbstract > terminalModelCG = 
	   boost::make_shared<crocoddyl::ActionModelCodeGen >(terminalModelAD, terminalModelD,"pyrene_running_in_contact");
    
    Eigen::VectorXd x_rand = runningModelCG->get_state()->rand();
    Eigen::VectorXd u_rand = Eigen::VectorXd::Random(runningModelCG->get_nu());
    
    std::vector<Eigen::VectorXd> x_guess(horizon_length+1, x_rand);
    std::vector<Eigen::VectorXd> u_guess(horizon_length, u_rand);
	//Create Shooting Problem
	std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract > > runningModelsCG(horizon_length, runningModelCG);
	boost::shared_ptr<crocoddyl::ShootingProblem>  shooting_problem = boost::make_shared<crocoddyl::ShootingProblem>(x_rand, runningModelsCG, terminalModelCG);

	boost::shared_ptr<crocoddyl::SolverFDDP>  ddp = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);

    ddp->solve(x_guess,u_guess,1,false);
    
    std::cout << "ddp cost = " << ddp->get_cost() << std::endl;
	return 0;
}
