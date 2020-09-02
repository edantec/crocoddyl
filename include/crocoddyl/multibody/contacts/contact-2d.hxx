///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

namespace crocoddyl {

template <typename Scalar>
ContactModel2DTpl<Scalar>::ContactModel2DTpl(boost::shared_ptr<StateMultibody> state, const FrameTranslation& xref,
                                             const std::size_t& nu, const Vector2s& gains)
    : Base(state, 2, nu), xref_(xref), gains_(gains) {}

template <typename Scalar>
ContactModel2DTpl<Scalar>::ContactModel2DTpl(boost::shared_ptr<StateMultibody> state, const FrameTranslation& xref,
                                             const Vector2s& gains)
    : Base(state, 2), xref_(xref), gains_(gains) {}

template <typename Scalar>
ContactModel2DTpl<Scalar>::~ContactModel2DTpl() {}

template <typename Scalar>
void ContactModel2DTpl<Scalar>::calc(const boost::shared_ptr<ContactDataAbstract>& data,
                                     const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  pinocchio::updateFramePlacement(*state_->get_pinocchio().get(), *d->pinocchio, xref_.frame);
  d->v = pinocchio::getFrameVelocity(*state_->get_pinocchio().get(), *d->pinocchio, xref_.frame);
  const Vector3s& vw = d->v.angular();
  const Vector3s& vv = d->v.linear();

  pinocchio::getFrameJacobian(*state_->get_pinocchio().get(), *d->pinocchio, xref_.frame, pinocchio::LOCAL, d->fJf);
  d->Jc.row(0) = d->fJf.row(0);
  d->Jc.row(1) = d->fJf.row(2);

  d->a = pinocchio::getFrameAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, xref_.frame);
  d->a0[0] = d->a.linear()[0] + vw[1]*vv[2] - vw[2]*vv[1];
  d->a0[1] = d->a.linear()[1] + vw[0]*vv[1] - vw[1]*vv[0];

  if (gains_[0] != 0.) {
    d->a0[0] += gains_[0] * (d->pinocchio->oMf[xref_.frame].translation() - xref_.oxf)[0];
    d->a0[1] += gains_[0] * (d->pinocchio->oMf[xref_.frame].translation() - xref_.oxf)[2];
  }
  if (gains_[1] != 0.) {
    d->a0[0] += gains_[1] * vv[0];
    d->a0[1] += gains_[1] * vv[2];
  }
}

template <typename Scalar>
void ContactModel2DTpl<Scalar>::calcDiff(const boost::shared_ptr<ContactDataAbstract>& data,
                                         const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  pinocchio::getJointAccelerationDerivatives(*state_->get_pinocchio().get(), *d->pinocchio, d->joint, pinocchio::LOCAL,
                                             d->v_partial_dq, d->a_partial_dq, d->a_partial_dv, d->a_partial_da);
  const std::size_t& nv = state_->get_nv();
  pinocchio::skew(d->vv, d->vv_skew);
  pinocchio::skew(d->vw, d->vw_skew);
  d->fXjdv_dq.noalias() = d->fXj * d->v_partial_dq;
  d->fXjda_dq.noalias() = d->fXj * d->a_partial_dq;
  d->fXjda_dv.noalias() = d->fXj * d->a_partial_dv;
      
  d->da0_dx.leftCols(nv).row(0).noalias() = d->fXjda_dq.row(0) +
      d->vw_skew.row(0) * d->fXjdv_dq.template topRows<3>() -
      d->vv_skew.row(0) * d->fXjdv_dq.template bottomRows<3>();
  d->da0_dx.leftCols(nv).row(1).noalias() = d->fXjda_dq.row(2) +
      d->vw_skew.row(2) * d->fXjdv_dq.template topRows<3>() -
      d->vv_skew.row(2) * d->fXjdv_dq.template bottomRows<3>();

  d->da0_dx.rightCols(nv).row(0).noalias() = d->fXjda_dv.row(0) + d->vw_skew.row(0) * d->fJf.template topRows<3>() - d->vv_skew.row(0) * d->fJf.template bottomRows<3>();
  d->da0_dx.rightCols(nv).row(1).noalias() = d->fXjda_dv.row(2) + d->vw_skew.row(2) * d->fJf.template topRows<3>() - d->vv_skew.row(2) * d->fJf.template bottomRows<3>();
  
  if (gains_[0] != 0.) {
    d->oRf = d->pinocchio->oMf[xref_.frame].rotation();
    MatrixXs oRf2D(2,2);
    oRf2D(0,0) = d->oRf(0,0);
    oRf2D(1,0) = d->oRf(2,0);
    oRf2D(0,1) = d->oRf(0,2);
    oRf2D(1,1) = d->oRf(2,2);
    d->da0_dx.leftCols(nv).noalias() += gains_[0] * oRf2D * d->Jc;
  }
  if (gains_[1] != 0.) {
    d->da0_dx.leftCols(nv).row(0).noalias() += gains_[1] * d->fXj.row(0) * d->v_partial_dq;
    d->da0_dx.leftCols(nv).row(1).noalias() += gains_[1] * d->fXj.row(2) * d->v_partial_dq;
    d->da0_dx.rightCols(nv).row(0).noalias() += gains_[1] * d->fXj.row(0) * d->a_partial_da;
    d->da0_dx.rightCols(nv).row(1).noalias() += gains_[1] * d->fXj.row(2) * d->a_partial_da;
  }
}

template <typename Scalar>
void ContactModel2DTpl<Scalar>::updateForce(const boost::shared_ptr<ContactDataAbstract>& data,
                                            const VectorXs& force) {
  if (force.size() != 2) {
    throw_pretty("Invalid argument: "
                 << "lambda has wrong dimension (it should be 2)");
  }
  Data* d = static_cast<Data*>(data.get());
  VectorXs force_bis(3);
  force_bis << force[0],0.0,force[1];
  data->f = d->jMf.act(pinocchio::ForceTpl<Scalar>(force_bis, Vector3s::Zero()));
}

template <typename Scalar>
boost::shared_ptr<ContactDataAbstractTpl<Scalar> > ContactModel2DTpl<Scalar>::createData(
    pinocchio::DataTpl<Scalar>* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const FrameTranslationTpl<Scalar>& ContactModel2DTpl<Scalar>::get_xref() const {
  return xref_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector2s& ContactModel2DTpl<Scalar>::get_gains() const {
  return gains_;
}

}  // namespace crocoddyl
