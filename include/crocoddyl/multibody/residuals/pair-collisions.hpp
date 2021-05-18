///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_
#define CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/state-base.hpp"
#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

namespace crocoddyl {

template <typename _Scalar>
class ResidualModelPairCollisionsTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelAbstractTpl<Scalar> Base;
  typedef ResidualDataPairCollisionsTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef pinocchio::GeometryModel GeometryModel;
  
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  ResidualModelPairCollisionsTpl(boost::shared_ptr<StateMultibody> state,
                             const std::size_t& nu,
                             boost::shared_ptr<GeometryModel> geom_model,
                             const pinocchio::PairIndex& pair_id, // const std::size_t col_id, // The id of the pair of colliding objects
                             const pinocchio::JointIndex& joint_id); // Used to calculate the Jac at the joint
  
  virtual ~ResidualModelPairCollisionsTpl();

  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  boost::shared_ptr<pinocchio::GeometryModel > geom_model_;
  boost::shared_ptr<typename StateMultibody::PinocchioModel> pin_model_;
  pinocchio::PairIndex pair_id_;
  pinocchio::JointIndex joint_id_;
};

template <typename _Scalar>
struct ResidualDataPairCollisionsTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3xs Matrix3xs;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Matrix6s Matrix6s;
  typedef typename MathBase::Vector6s Vector6s;

  template <template <typename Scalar> class Model>
  ResidualDataPairCollisionsTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
      : Base(model, data),
        geom_data(pinocchio::GeometryData(model->get_geomModel())),
        J(Matrix6xs::Zero(6, model->get_state()->get_nv())) {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty("Invalid argument: the shared data should be derived from DataCollectorMultibody");
    }
    // Avoids data casting at runtime
    pinocchio = d->pinocchio;    
  }

  pinocchio::GeometryData geom_data;
  pinocchio::DataTpl<Scalar>* pinocchio;

  Matrix6xs J;
  
  using Base::shared;
  using Base::activation;
  using Base::cost;
  using Base::Lx;
  using Base::Lxx;
  using Base::r;
  using Base::Rx;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/residuals/pair-collisions.hxx"

#endif  // CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_
