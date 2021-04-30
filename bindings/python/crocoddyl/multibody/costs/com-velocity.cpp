///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "crocoddyl/multibody/costs/com-velocity.hpp"
#include "python/crocoddyl/utils/deprecate.hpp"

namespace crocoddyl {
namespace python {

void exposeCostCoMVelocity() {
  bp::class_<CostModelCoMVelocity, bp::bases<CostModelAbstract> >(
      "CostModelCoMVelocity",
      "This cost function defines a residual vector as r = v - vref, with v and vref as the current and reference "
      "CoM velocity, respetively.",
      bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActivationModelAbstract>, Eigen::Vector3d, int>(
          bp::args("self", "state", "activation", "vref", "nu"),
          "Initialize the CoM velocity cost model.\n\n"
          ":param state: state of the multibody system\n"
          ":param activation: activation model\n"
          ":param cref: reference CoM velocity\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActivationModelAbstract>, Eigen::Vector3d>(
          bp::args("self", "state", "activation", "vref"),
          "Initialize the CoM velocity cost model.\n\n"
          "The default nu is obtained from state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param activation: activation model\n"
          ":param vref: reference CoM velocity"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d, int>(
          bp::args("self", "state", "vref", "nu"),
          "Initialize the CoM velocity cost model.\n\n"
          "We use ActivationModelQuad as a default activation model (i.e. a=0.5*||r||^2).\n"
          ":param state: state of the multibody system\n"
          ":param vref: reference CoM velocity\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d>(
          bp::args("self", "state", "vref"),
          "Initialize the CoM velocity cost model.\n\n"
          "We use ActivationModelQuad as a default activation model (i.e. a=0.5*||r||^2), and nu is obtained from "
          "state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param vref: reference CoM velocity"))
      .def<void (CostModelCoMVelocity::*)(const boost::shared_ptr<CostDataAbstract>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &CostModelCoMVelocity::calc, bp::args("self", "data", "x", "u"),
          "Compute the CoM velocity cost.\n\n"
          ":param data: cost data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (CostModelCoMVelocity::*)(const boost::shared_ptr<CostDataAbstract>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&)>("calc", &CostModelAbstract::calc,
                                                                                     bp::args("self", "data", "x"))
      .def<void (CostModelCoMVelocity::*)(const boost::shared_ptr<CostDataAbstract>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &CostModelCoMVelocity::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the derivatives of the CoM velocity cost.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (CostModelCoMVelocity::*)(const boost::shared_ptr<CostDataAbstract>&,
                                          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &CostModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &CostModelCoMVelocity::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the CoM velocity cost data.\n\n"
           "Each cost model has its own data that needs to be allocated. This function\n"
           "returns the allocated data for a predefined cost.\n"
           ":param data: shared data\n"
           ":return cost data.")
      .add_property("reference", &CostModelCoMVelocity::get_reference<Eigen::Vector3d>,
                    &CostModelCoMVelocity::set_reference<Eigen::Vector3d>, "reference CoM velocity")
      .add_property("vref",
                    bp::make_function(&CostModelCoMVelocity::get_reference<Eigen::Vector3d>,
                                      deprecated<>("Deprecated. Use reference.")),
                    bp::make_function(&CostModelCoMVelocity::set_reference<Eigen::Vector3d>,
                                      deprecated<>("Deprecated. Use reference.")),
                    "reference CoM velocity");
}

}  // namespace python
}  // namespace crocoddyl
