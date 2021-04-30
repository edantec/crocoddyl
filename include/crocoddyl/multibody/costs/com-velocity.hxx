///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/costs/com-velocity.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"

namespace crocoddyl {

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                         boost::shared_ptr<ActivationModelAbstract> activation,
                                                         const Vector3s& vref, const std::size_t nu)
    : Base(state, activation, nu), vref_(vref), pin_model_(*state->get_pinocchio()) {
  if (activation_->get_nr() != 3) {
    throw_pretty("Invalid argument: "
                 << "nr is equals to 3");
  }
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                         boost::shared_ptr<ActivationModelAbstract> activation,
                                                         const Vector3s& vref)
    : Base(state, activation), vref_(vref), pin_model_(*state->get_pinocchio()) {
  if (activation_->get_nr() != 3) {
    throw_pretty("Invalid argument: "
                 << "nr is equals to 3");
  }
}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref,
                                                         const std::size_t nu)
    : Base(state, 3, nu), vref_(vref), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref)
    : Base(state, 3), vref_(vref), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
CostModelCoMVelocityTpl<Scalar>::~CostModelCoMVelocityTpl() {}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::calc(const boost::shared_ptr<CostDataAbstract>& data,
                                           const Eigen::Ref<const VectorXs>&x, const Eigen::Ref<const VectorXs>&u) {
  // Compute the cost residual give the reference CoMPosition velocity
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());
  
  pinocchio::centerOfMass(pin_model_,*d->pinocchio,q,v);
  data->r = d->pinocchio->vcom[0] - vref_;

  // Compute the cost
  activation_->calc(data->activation, data->r);
  data->cost = data->activation->a_value;
}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::calcDiff(const boost::shared_ptr<CostDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>&x, const Eigen::Ref<const VectorXs>&u) {
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

  const std::size_t nv = state_->get_nv();
  activation_->calcDiff(data->activation, data->r);
  pinocchio::getCenterOfMassVelocityDerivatives(pin_model_,*d->pinocchio,d->vcom_dq);
  
  data->Rx.leftCols(nv) = d->vcom_dq;
  data->Rx.rightCols(nv) = d->pinocchio->Jcom;
  data->Lx.head(nv).noalias() = d->vcom_dq.transpose() * data->activation->Ar;
  data->Lx.tail(nv).noalias() = d->pinocchio->Jcom.transpose() * data->activation->Ar;
  d->Arr_Jcom.noalias() = data->activation->Arr * d->pinocchio->Jcom;
  d->Arr_vcom_dq.noalias() = data->activation->Arr * d->vcom_dq;
  data->Lxx.topLeftCorner(nv, nv).noalias() = d->vcom_dq.transpose() * d->Arr_vcom_dq;
  data->Lxx.bottomLeftCorner(nv, nv).noalias() = d->pinocchio->Jcom.transpose() * d->Arr_vcom_dq;
  data->Lxx.topRightCorner(nv, nv).noalias() = d->vcom_dq.transpose() * d->Arr_Jcom;
  data->Lxx.bottomRightCorner(nv, nv).noalias() = d->pinocchio->Jcom.transpose() * d->Arr_Jcom;
}

template <typename Scalar>
boost::shared_ptr<CostDataAbstractTpl<Scalar> > CostModelCoMVelocityTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::set_referenceImpl(const std::type_info& ti, const void* pv) {
  if (ti == typeid(Vector3s)) {
    vref_ = *static_cast<const Vector3s*>(pv);
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be Vector3s)");
  }
}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::get_referenceImpl(const std::type_info& ti, void* pv) const {
  if (ti == typeid(Vector3s)) {
    Eigen::Map<Vector3s> ref_map(static_cast<Vector3s*>(pv)->data());
    ref_map[0] = vref_[0];
    ref_map[1] = vref_[1];
    ref_map[2] = vref_[2];
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be Vector3s)");
  }
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& CostModelCoMVelocityTpl<Scalar>::get_vref() const {
  return vref_;
}

template <typename Scalar>
void CostModelCoMVelocityTpl<Scalar>::set_vref(const Vector3s& vref_in) {
  vref_ = vref_in;
}

}  // namespace crocoddyl
