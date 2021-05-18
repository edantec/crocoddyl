///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_COSTS_COM_VELOCITY_HPP_
#define CROCODDYL_MULTIBODY_COSTS_COM_VELOCITY_HPP_

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/residuals/com-velocity.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/core/utils/exception.hpp"

namespace crocoddyl {

/**
 * @brief CoM velocity cost
 *
 * This cost function defines a residual vector as \f$\mathbf{r}=\mathbf{v_{c}}-\mathbf{v_{c}}^*\f$, where
 * \f$\mathbf{v_{c}},\mathbf{v_{c}}^*\in~\mathbb{R}^3\f$ are the current and reference CoM velocity, respetively. Note that the
 * dimension of the residual vector is obtained from 3.
 *
 * Both cost and residual derivatives are computed analytically.
 * For the computation of the cost Hessian, we use the Gauss-Newton approximation, e.g.
 * \f$\mathbf{l_{xx}} = \mathbf{l_{x}}^T \mathbf{l_{x}} \f$.
 *
 * As described in CostModelAbstractTpl(), the cost value and its derivatives are calculated by `calc` and `calcDiff`,
 * respectively.
 *
 * \sa `CostModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class CostModelCoMVelocityTpl : public CostModelResidualTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelResidualTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef CostDataAbstractTpl<Scalar> CostDataAbstract;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef ResidualModelCoMVelocityTpl<Scalar> ResidualModelCoMVelocity;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;

  /**
   * @brief Initialize the CoM velocity cost model
   *
   * @param[in] state       State of the multibody system
   * @param[in] activation  Activation model
   * @param[in] vref        Reference CoM velocity
   * @param[in] nu          Dimension of the control vector
   */
  CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                          boost::shared_ptr<ActivationModelAbstract> activation, const Vector3s& vref,
                          const std::size_t nu);

  /**
   * @brief Initialize the CoM velocity cost model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state       State of the multibody system
   * @param[in] activation  Activation model
   * @param[in] vref        Reference CoM velocity
   */
  CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                          boost::shared_ptr<ActivationModelAbstract> activation, const Vector3s& vref);

  /**
   * @brief Initialize the CoM velocity cost model
   *
   * We use `ActivationModelQuadTpl` as a default activation model (i.e. \f$a=\frac{1}{2}\|\mathbf{r}\|^2\f$).
   *
   * @param[in] state  State of the multibody system
   * @param[in] vref   Reference CoM velocity
   * @param[in] nu     Dimension of the control vector
   */
  CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref, const std::size_t nu);

  /**
   * @brief Initialize the CoM velocity cost model
   *
   * We use `ActivationModelQuadTpl` as a default activation model (i.e. \f$a=\frac{1}{2}\|\mathbf{r}\|^2\f$).
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] cref   Reference CoM velocity
   */
  CostModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref);
  virtual ~CostModelCoMVelocityTpl();

 protected:
  /**
   * @brief Modify the CoM velocity reference
   */
  virtual void set_referenceImpl(const std::type_info& ti, const void* pv);

  /**
   * @brief Return the CoM velocity reference
   */
  virtual void get_referenceImpl(const std::type_info& ti, void* pv);

  using Base::activation_;
  using Base::nu_;
  using Base::residual_;
  using Base::state_;
  using Base::unone_;

 private:
  Vector3s vref_;  //!< Reference CoM velocity.
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/costs/com-velocity.hxx"

#endif  // CROCODDYL_MULTIBODY_COSTS_COM_VELOCITY_HPP_
