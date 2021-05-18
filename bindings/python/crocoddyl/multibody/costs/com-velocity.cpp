///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "crocoddyl/multibody/costs/com-velocity.hpp"
#include "python/crocoddyl/utils/deprecate.hpp"

namespace crocoddyl {
namespace python {

void exposeCostCoMVelocity() {  // TODO: Remove once the deprecated update call has been removed in a future
                                // release
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  bp::register_ptr_to_python<boost::shared_ptr<CostModelCoMVelocity> >();

  bp::class_<CostModelCoMVelocity, bp::bases<CostModelResidual> >(
      "CostModelCoMVelocity",
      "This cost function defines a residual vector as r = v - vref, with v and vref as the current and reference "
      "CoM velocity, respetively.",
      bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActivationModelAbstract>, Eigen::Vector3d, int>(
          bp::args("self", "state", "activation", "vref", "nu"),
          "Initialize the CoM velocity cost model.\n\n"
          ":param state: state of the multibody system\n"
          ":param activation: activation model\n"
          ":param vref: reference CoM velocity\n"
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
      .add_property("reference", &CostModelCoMVelocity::get_reference<Eigen::Vector3d>,
                    &CostModelCoMVelocity::set_reference<Eigen::Vector3d>, "reference CoM velocity")
      .add_property("vref",
                    bp::make_function(&CostModelCoMVelocity::get_reference<Eigen::Vector3d>,
                                      deprecated<>("Deprecated. Use reference.")),
                    bp::make_function(&CostModelCoMVelocity::set_reference<Eigen::Vector3d>,
                                      deprecated<>("Deprecated. Use reference.")),
                    "reference CoM velocity");

#pragma GCC diagnostic pop
}

}  // namespace python
}  // namespace crocoddyl
