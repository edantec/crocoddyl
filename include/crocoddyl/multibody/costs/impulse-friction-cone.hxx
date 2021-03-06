///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/costs/impulse-friction-cone.hpp"

namespace crocoddyl {

template <typename Scalar>
CostModelImpulseFrictionConeTpl<Scalar>::CostModelImpulseFrictionConeTpl(
    boost::shared_ptr<StateMultibody> state, boost::shared_ptr<ActivationModelAbstract> activation,
    const FrameFrictionCone& fref)
    : Base(state, activation, boost::make_shared<ResidualModelContactFrictionCone>(state, fref.id, fref.cone, 0)),
      fref_(fref) {
  std::cerr << "Deprecated CostModelImpulseFrictionCone: Use ResidualModelContactFrictionCone with "
               "CostModelResidual class"
            << std::endl;
  if (activation_->get_nr() != fref_.cone.get_nf() + 1) {
    throw_pretty("Invalid argument: "
                 << "nr is equals to " << fref_.cone.get_nf() + 1);
  }
}

template <typename Scalar>
CostModelImpulseFrictionConeTpl<Scalar>::CostModelImpulseFrictionConeTpl(boost::shared_ptr<StateMultibody> state,
                                                                         const FrameFrictionCone& fref)
    : Base(state, boost::make_shared<ResidualModelContactFrictionCone>(state, fref.id, fref.cone, 0)), fref_(fref) {
  std::cerr << "Deprecated CostModelImpulseFrictionCone: Use ResidualModelContactFrictionCone with "
               "CostModelResidual class"
            << std::endl;
}

template <typename Scalar>
CostModelImpulseFrictionConeTpl<Scalar>::~CostModelImpulseFrictionConeTpl() {}

template <typename Scalar>
void CostModelImpulseFrictionConeTpl<Scalar>::set_referenceImpl(const std::type_info& ti, const void* pv) {
  if (ti == typeid(FrameFrictionCone)) {
    fref_ = *static_cast<const FrameFrictionCone*>(pv);
    ResidualModelContactFrictionCone* residual = static_cast<ResidualModelContactFrictionCone*>(residual_.get());
    residual->set_id(fref_.id);
    residual->set_reference(fref_.cone);
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be FrameFrictionCone)");
  }
}

template <typename Scalar>
void CostModelImpulseFrictionConeTpl<Scalar>::get_referenceImpl(const std::type_info& ti, void* pv) {
  if (ti == typeid(FrameFrictionCone)) {
    FrameFrictionCone& ref_map = *static_cast<FrameFrictionCone*>(pv);
    ResidualModelContactFrictionCone* residual = static_cast<ResidualModelContactFrictionCone*>(residual_.get());
    fref_.id = residual->get_id();
    fref_.cone = residual->get_reference();
    ref_map = fref_;
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be FrameFrictionCone)");
  }
}

}  // namespace crocoddyl
