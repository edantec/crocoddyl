///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/costs/com-velocity.hpp"

namespace crocoddyl {

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                         boost::shared_ptr<ActivationModelAbstract> activation,
                                                         const Vector3s& vref, const std::size_t nu)
    : Base(state, activation, boost::make_shared<ResidualModelCoMVelocity>(state, vref, nu)), vref_(vref) {
  std::cerr << "Deprecated CostModelCoMVelocity: Use ResidualModelCoMVelocity with CostModelResidual" << std::endl;
  if (activation_->get_nr() != 3) {
    throw_pretty("Invalid argument: "
                 << "nr is equals to 3");
  }
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                         boost::shared_ptr<ActivationModelAbstract> activation,
                                                         const Vector3s& vref)
    : Base(state, activation, boost::make_shared<ResidualModelCoMVelocity>(state, vref)), vref_(vref) {
  std::cerr << "Deprecated CostModelCoMVelocity: Use ResidualModelCoMVelocity with CostModelResidual" << std::endl;
  if (activation_->get_nr() != 3) {
    throw_pretty("Invalid argument: "
                 << "nr is equals to 3");
  }
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref,
                                                         const std::size_t nu)
    : Base(state, boost::make_shared<ResidualModelCoMVelocity>(state, vref, nu)), vref_(vref) {
  std::cerr << "Deprecated CostModelCoMVelocity: Use ResidualModelCoMVelocity with CostModelResidual" << std::endl;
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref)
    : Base(state, boost::make_shared<ResidualModelCoMVelocity>(state, vref)), vref_(vref) {
  std::cerr << "Deprecated CostModelCoMVelocity: Use ResidualModelCoMVelocity with CostModelResidual" << std::endl;
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::~CostModelCoMVelocityTpl() {}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::set_referenceImpl(const std::type_info& ti, const void* pv) {
  if (ti == typeid(Vector3s)) {
    vref_ = *static_cast<const Vector3s*>(pv);
    ResidualModelCoMVelocity* residual = static_cast<ResidualModelCoMVelocity*>(residual_.get());
    residual->set_reference(vref_);
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be Vector3s)");
  }
}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::get_referenceImpl(const std::type_info& ti, void* pv) {
  if (ti == typeid(Vector3s)) {
    Eigen::Map<Vector3s> ref_map(static_cast<Vector3s*>(pv)->data());
    ResidualModelCoMVelocity* residual = static_cast<ResidualModelCoMVelocity*>(residual_.get());
    vref_ = residual->get_reference();
    ref_map[0] = vref_[0];
    ref_map[1] = vref_[1];
    ref_map[2] = vref_[2];
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be Vector3s)");
  }
}

}  // namespace crocoddyl
