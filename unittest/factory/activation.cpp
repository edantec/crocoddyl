///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "activation.hpp"
#include "crocoddyl/core/activations/quadratic.hpp"
#include "crocoddyl/core/activations/quadratic-ref.hpp"
#include "crocoddyl/core/activations/quadratic-flat-exp.hpp"
#include "crocoddyl/core/activations/quadratic-flat-log.hpp"
#include "crocoddyl/core/activations/norm2-barrier.hpp"
#include "crocoddyl/core/activations/smooth-1norm.hpp"
#include "crocoddyl/core/activations/smooth-2norm.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic-barrier.hpp"
#include "crocoddyl/core/utils/exception.hpp"

namespace crocoddyl {
namespace unittest {

const std::vector<ActivationModelTypes::Type> ActivationModelTypes::all(ActivationModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, ActivationModelTypes::Type type) {
  switch (type) {
    case ActivationModelTypes::ActivationModelQuad:
      os << "ActivationModelQuad";
      break;
    case ActivationModelTypes::ActivationModelQuadRef:
      os << "ActivationModelQuadRef";
      break;
    case ActivationModelTypes::ActivationModelQuadFlatExp:
      os << "ActivationModelQuadFlatExp";
      break;
    case ActivationModelTypes::ActivationModelQuadFlatLog:
      os << "ActivationModelQuadFlatLog";
      break;
    case ActivationModelTypes::ActivationModelNorm2Barrier:
      os << "ActivationModelNorm2Barrier";
      break;
    case ActivationModelTypes::ActivationModelSmooth1Norm:
      os << "ActivationModelSmooth1Norm";
      break;
    case ActivationModelTypes::ActivationModelSmooth2Norm:
      os << "ActivationModelSmooth2Norm";
      break;
    case ActivationModelTypes::ActivationModelWeightedQuad:
      os << "ActivationModelWeightedQuad";
      break;
    case ActivationModelTypes::ActivationModelQuadraticBarrier:
      os << "ActivationModelQuadraticBarrier";
      break;
    case ActivationModelTypes::ActivationModelWeightedQuadraticBarrier:
      os << "ActivationModelWeightedQuadraticBarrier";
      break;
    case ActivationModelTypes::NbActivationModelTypes:
      os << "NbActivationModelTypes";
      break;
    default:
      break;
  }
  return os;
}

ActivationModelFactory::ActivationModelFactory() {}
ActivationModelFactory::~ActivationModelFactory() {}

boost::shared_ptr<crocoddyl::ActivationModelAbstract> ActivationModelFactory::create(
    ActivationModelTypes::Type activation_type, std::size_t nr) const {
  boost::shared_ptr<crocoddyl::ActivationModelAbstract> activation;
  Eigen::VectorXd lb = Eigen::VectorXd::Random(nr);
  Eigen::VectorXd ub = lb + Eigen::VectorXd::Ones(nr) + Eigen::VectorXd::Random(nr);
  Eigen::VectorXd weights = 0.1 * Eigen::VectorXd::Random(nr);
  double threshold = fabs(Eigen::VectorXd::Random(1)[0]);
  double alpha = fabs(Eigen::VectorXd::Random(1)[0]);
  double eps = fabs(Eigen::VectorXd::Random(1)[0]);

  switch (activation_type) {
    case ActivationModelTypes::ActivationModelQuad:
      activation = boost::make_shared<crocoddyl::ActivationModelQuad>(nr);
      break;
    case ActivationModelTypes::ActivationModelQuadRef:
      activation = boost::make_shared<crocoddyl::ActivationModelQuadRef>(lb);
      break;
    case ActivationModelTypes::ActivationModelQuadFlatExp:
      activation = boost::make_shared<crocoddyl::ActivationModelQuadFlatExp>(nr, alpha);
      break;
    case ActivationModelTypes::ActivationModelQuadFlatLog:
      activation = boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(nr, alpha);
      break;
    case ActivationModelTypes::ActivationModelNorm2Barrier:
      activation = boost::make_shared<crocoddyl::ActivationModelNorm2Barrier>(nr, threshold);
      break;
    case ActivationModelTypes::ActivationModelSmooth1Norm:
      activation = boost::make_shared<crocoddyl::ActivationModelSmooth1Norm>(nr, eps);
      break;
    case ActivationModelTypes::ActivationModelSmooth2Norm:
      activation = boost::make_shared<crocoddyl::ActivationModelSmooth2Norm>(nr, eps);
      break;
    case ActivationModelTypes::ActivationModelWeightedQuad:
      activation = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(weights);
      break;
    case ActivationModelTypes::ActivationModelQuadraticBarrier:
      activation = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(crocoddyl::ActivationBounds(lb, ub));
      break;
    case ActivationModelTypes::ActivationModelWeightedQuadraticBarrier:
      activation = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
          crocoddyl::ActivationBounds(lb, ub), weights);
      break;
    default:
      throw_pretty(__FILE__ ":\n Construct wrong ActivationModelTypes::Type");
      break;
  }
  return activation;
}

}  // namespace unittest
}  // namespace crocoddyl
