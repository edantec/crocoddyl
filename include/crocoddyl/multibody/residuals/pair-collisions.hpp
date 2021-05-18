///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_
#define CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_

#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/core/utils/exception.hpp"
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
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataPairCollisionsTpl<Scalar> Data;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef pinocchio::GeometryModel GeometryModel;
  
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;


  
  ResidualModelPairCollisionsTpl(boost::shared_ptr<StateMultibody> state,
                             const std::size_t& nu,
                             boost::shared_ptr<GeometryModel> geom_model,
                             const pinocchio::PairIndex& pair_id, // const std::size_t col_id, // The id of the pair of colliding objects
                             const pinocchio::JointIndex& joint_id);

 
  virtual ~ResidualModelPairCollisionsTpl();


  virtual void calc(const boost::shared_ptr<ResidualDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);
  
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract *const data);
  const pinocchio::GeometryModel& get_geometryModel() const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  typename StateMultibody::PinocchioModel pin_model_;  //!< Pinocchio model used for internal computations
  boost::shared_ptr<pinocchio::GeometryModel > geom_model_;
  pinocchio::PairIndex pair_id_;
  pinocchio::JointIndex joint_id_;
};

template <typename _Scalar>
struct ResidualDataPairCollisionsTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  typedef typename MathBase::Matrix6xs Matrix6xs;

  template <template <typename Scalar> class Model>
  ResidualDataPairCollisionsTpl(Model<Scalar> *const model, DataCollectorAbstract *const data)
   : Base(model, data),
     geom_data(pinocchio::GeometryData(model->get_geometryModel())),
     J(Matrix6xs::Zero(6, model->get_state()->get_nv())) {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar> *d = dynamic_cast<DataCollectorMultibodyTpl<Scalar> *>(shared);
    if (d == NULL) {
      throw_pretty("Invalid argument: the shared data should be derived from DataCollectorActMultibodyTpl");
    }
    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }
  pinocchio::GeometryData geom_data;
  pinocchio::DataTpl<Scalar>* pinocchio;
  Matrix6xs J;                                     //!< Pinocchio data
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/residuals/pair-collisions.hxx"

#endif  // CROCODDYL_MULTIBODY_RESIDUALS_PAIR_COLLISIONS_HPP_
