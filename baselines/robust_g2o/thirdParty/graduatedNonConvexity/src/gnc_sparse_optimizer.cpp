// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "gnc_sparse_optimizer.hpp"
#include "gnc_kernel_impl.hpp"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <iterator>

#include "g2o/config.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/ownership.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/robust_kernel.h"

#include <boost/math/distributions/chi_squared.hpp>

namespace g2o {
using namespace std;

double Chi2inv(const double alpha, const size_t dofs) 
{
  boost::math::chi_squared_distribution<double> chi2(dofs);
  return boost::math::quantile(chi2, alpha);
}

GNCSparseOptimizer::GNCSparseOptimizer(GncLossType lossType)
    : SparseOptimizer()
{
    lossType_ = lossType;
    alpha_ = 0.99;
}

GNCSparseOptimizer::GNCSparseOptimizer() 
{
    GNCSparseOptimizer(GncLossType::TLS);
}

GNCSparseOptimizer::~GNCSparseOptimizer() {}

// Default GNC optimization
int GNCSparseOptimizer::optimizeX(int iterations, bool online) 
{
  // Compute solution for the convex approximation of the problem
  //int total_iterations = defaultOptimize(online);

  // Compute GNC parameters from the current state of the system
  computeActiveErrors();
  double prev_cost = activeChi2();
  double cost = 0.0;
  double mu = initializeMu();
  double best_mu = mu;
  initializeKernels(mu);

  // Degenerate case where all residuals are below the threshold
  if (mu <= 0.0 ) 
    return 0;

  for (int gnc_iter = 0; gnc_iter < iterations; ++gnc_iter)
  {
    // Optimize the cost with the current robust kernels
    int total_iterations = defaultOptimize(online);

    // Compute current cost
    computeActiveErrors();
    cost = activeRobustChi2();

    /**/
    std::cout << "GNC iteration " << gnc_iter + 1 << " : mu = " << mu 
              << ", cost = " << std::setprecision(12) << cost 
              << ", prev_cost = " << std::setprecision(12) << prev_cost 
              << ", GN iterations = "  << total_iterations << std::endl;
    /**/
    
    // Check convergence
    if ( checkConvergence(mu, cost, prev_cost) )
      return gnc_iter + 1;

    // Update all the variables for the next iteration
    mu = updateMu(mu);
    updateKernels(mu);
    prev_cost = cost;
  }

  return iterations;
}

// Optimization that uses the previous estimate only if it improves the cost
int GNCSparseOptimizer::optimizeY(int iterations, bool online) 
{
  // Compute solution for the convex approximation of the problem
  double cost_init_guess = activeChi2();
  push();

  //int total_iterations = defaultOptimize(online);

  // Compute GNC parameters from the current state of the system
  computeActiveErrors();
  double prev_cost = activeChi2();

  if (cost_init_guess < prev_cost)
  {
    pop();
    prev_cost = cost_init_guess;
    push();
    computeActiveErrors();
  }
  else
  {
    // Discard the initial guess and push the new solution
    discardTop();
    push();
  }

  double cost = 0.0;
  double mu = initializeMu();
  double best_mu = mu;
  initializeKernels(mu);

  // Degenerate case where all residuals are below the threshold
  if (mu <= 0.0 ) 
    return 0;

  for (int gnc_iter = 0; gnc_iter < iterations; ++gnc_iter)
  {
    // Optimize the cost with the current robust kernels
    int total_iterations = defaultOptimize(online);

    // Compute current cost
    computeActiveErrors();
    cost = activeRobustChi2();
    double non_robust_cost = activeChi2();

    /**/
    std::cout << "GNC iteration " << gnc_iter + 1 << " : mu = " << mu 
              << ", cost = " << std::setprecision(12) << cost
              << ", non_robust_cost = " << std::setprecision(12) << non_robust_cost
              << ", prev_cost = " << std::setprecision(12) << prev_cost 
              << ", GN iterations = "  << total_iterations << std::endl;
    /**/

    if (cost > prev_cost)
    {
      pop();
      cost = prev_cost;
      push();
      computeActiveErrors();
    }
    else
    {
      // Discard the initial guess and push the new solution
      discardTop();
      push();
    }

    // Check convergence
    if ( checkConvergence(mu, cost, prev_cost) )
      return gnc_iter + 1;

    // Update all the variables for the next iteration
    mu = updateMu(mu);
    updateKernels(mu);
    prev_cost = cost;
  }

  return iterations;
}

void GNCSparseOptimizer::setInnerIterations(int iterations) 
{
  inner_iters_ = iterations;
  return;
}

void GNCSparseOptimizer::setMuStep(double mu_step)
{
  muStep_ = mu_step;
  return;
}


// Function needs to be called after initializeOptimization
void GNCSparseOptimizer::setKnownInliers(EdgeContainer& inliers)
{
  int counter = 0;
  for (size_t idx = 0; idx < _activeEdges.size(); ++idx)
  {
    OptimizableGraph::Edge* e_active = _activeEdges[idx];
    EdgeContainer::const_iterator it_first = lower_bound(inliers.begin(), inliers.end(), 
                                                         e_active, EdgeIDCompare());
    // If the activeEdge is in the inliers, we don't need to robustify it as we want it to
    // have as much influence as possible
    if ( it_first != inliers.end() && e_active->internalId() == (*it_first)->internalId() ) continue;

    // Vector of edges to be robustified
    egrad_.push_back(e_active);
    ++counter;
  }

  return;
}

int GNCSparseOptimizer::defaultOptimize(bool online) 
{
  return SparseOptimizer::optimize(inner_iters_, online);
}

// The residual might be the chi2 instead of the squaredNorm of the error
double GNCSparseOptimizer::initializeMu() const 
{
  double mu_init = 0.0;

  // initialize mu to the value specified in Remark 5 in GNC paper
  switch (lossType_)
  {
    // Setting the mu_init as in Remark 5: For GM the robust kernel is 
    // approximable as a convex function when mu goes to infinity: in practice
    // it is set like this
    case GncLossType::GM:
      for (int idx = 0; idx < static_cast<int>(_activeEdges.size()); ++idx)
        mu_init = std::max(mu_init, 2 * _activeEdges[idx]->chi2() / Chi2inv(alpha_, _activeEdges[idx]->dimension()));
        if (mu_init < 10.0) mu_init = 256; 
      return mu_init;
    // Doing the same for DCS because similar
    case GncLossType::DCS:
      for (int idx = 0; idx < static_cast<int>(_activeEdges.size()); ++idx)
        mu_init = std::max(mu_init, 2 * _activeEdges[idx]->chi2() / Chi2inv(alpha_, _activeEdges[idx]->dimension()));
      return mu_init;
    // surrogate cost is convex for mu close to zero. initialize as in remark 5 in GNC paper.
    // degenerate case: 2 * rmax_sq - params_.barcSq < 0 (handled in the main loop)
    // according to remark mu = params_.barcSq / (2 * rmax_sq - params_.barcSq) = params_.barcSq/ excessResidual
    // however, if the denominator is 0 or negative, we return mu = -1 which leads to termination of the main GNC loop.
    // Since barcSq_ can be different for each factor, we look for the minimimum (positive) quantity in remark 5 in GNC paper
    case GncLossType::TLS:
      mu_init = std::numeric_limits<double>::infinity();
      for (int idx = 0; idx < static_cast<int>(_activeEdges.size()); ++idx)
      {
        double chi2_th = Chi2inv(alpha_, _activeEdges[idx]->dimension());
        double mu_denom = (2 * _activeEdges[idx]->chi2() - chi2_th); 
        mu_init = mu_denom > 0.0 ? std::min(mu_init, chi2_th / mu_denom) : mu_init; // Updating only if positive
      }
      if (mu_init >= 0 && mu_init < 1e-6) mu_init = 1e-6; 
      if (std::isinf(mu_init)) mu_init = -1.0;

      return mu_init;
    default:
      throw std::runtime_error("GncOptimizer::initializeMu: called with unknown loss type.");
  }

  return mu_init;
}

void GNCSparseOptimizer::initializeKernels(const double mu)
{
  switch (lossType_)
  {
    case GncLossType::GM:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuGM* rk = new RobustMuGM();
        rk->setMu(mu);
        rk->setDelta(Chi2inv(alpha_, e->dimension()));
        e->setRobustKernel(rk);
      }
      return; 
    case GncLossType::TLS:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuTLS* rk = new RobustMuTLS();
        rk->setMu(mu);
        rk->setDelta(Chi2inv(alpha_, e->dimension()));
        e->setRobustKernel(rk);
      } 
      return;
    case GncLossType::DCS:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuDCS* rk = new RobustMuDCS();
        rk->setMu(mu);
        rk->setDelta(Chi2inv(alpha_, e->dimension()));
        e->setRobustKernel(rk);
      } 
      return;
    default:
      throw std::runtime_error("GncOptimizer::initializeKernels: called with unknown loss type.");
  }

  return;
}

double GNCSparseOptimizer::updateMu(const double mu) const 
{
  switch (lossType_)
  {
    // reduce mu, but saturate at 1 (original cost is recovered for mu -> 1)
    case GncLossType::GM:
      return std::max(1.0, mu / muStep_);
    // increases mu at each iteration (original cost is recovered for mu -> inf)
    case GncLossType::TLS:
      return mu * muStep_;
    case GncLossType::DCS:
      return std::max(1.0, mu / muStep_);
    default:
      throw std::runtime_error("GncOptimizer::updateMu: called with unknown loss type.");
  }

  return 0.0;
}

bool GNCSparseOptimizer::checkConvergence(const double mu, const double cost, const double prev_cost) const
{
  return checkMuConvergence(mu) || checkCostConvergence(cost, prev_cost) || checkKernelConvergence();
}


bool GNCSparseOptimizer::checkMuConvergence(const double mu) const
{
  switch (lossType_)
  {
    // mu=1 recovers the original GM function
    case GncLossType::GM:
      return std::fabs(mu - 1.0) < 1e-9;
    // for TLS there is no stopping condition on mu (it must tend to infinity)
    case GncLossType::TLS:
      return false;
    // mu=1 recovers the original DCS function
    case GncLossType::DCS:
      return std::fabs(mu - 1.0) < 1e-9;
    default:
      throw std::runtime_error("GncOptimizer::checkMuConvergence: called with unknown loss type.");
  }

  return false;
}

// TODO: Check if enabling this improves speed while maintaining accuracy
bool GNCSparseOptimizer::checkCostConvergence(const double cost, const double prev_cost) const
{
  //return std::fabs(cost - prev_cost) / std::max(prev_cost, 1e-7) < 1e-4;
  return false;
}

bool GNCSparseOptimizer::isEdgeInlier(const int idx)
{
  // Check index bounds
  if (idx < 0 || idx >= static_cast<int>(egrad_.size()))
    throw std::runtime_error("GncOptimizer::isEdgeInlier: index out of bounds.");

  OptimizableGraph::Edge* e = egrad_[idx];
  RobustKernel* rk = e->robustKernel();
  Vector3 rho;
  rk->robustify(e->chi2(), rho);
  double weight = rho[1];
  //double residual = computeResidual(e);

  return weight > 0.9; // consider inlier if weight is closer to 1 than to 0
  
}

// TODO: FINISH THIS FUNCTION
bool GNCSparseOptimizer::checkKernelConvergence() const
{
  switch(lossType_)
  {
    case GncLossType::TLS:
    {
      // Check convergence of weights to binary values.
      double max_weight = 0.0;
      double tot = 0.0;
      //std::cout << "Checking kernel convergence over " << egrad_.size() << " edges." << std::endl;
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustKernel* rk = e->robustKernel();
        Vector3 rho;
        rk->robustify(e->chi2(), rho);
        
        // Converged when all weights are close to binary values
        double weight = rho[1];
        if (weight < 0.0)
          std::cout << "Edge " << idx << " weight: " << weight << " delta:" << rk->delta() << std::endl;
        
          tot += weight;
        max_weight = std::max(max_weight, weight);
        if ( std::fabs(weight - std::round(weight)) < weightsTol_ ) 
          continue;

        return false;
      }
      // Return true only if it gets through all edges successfully
      //std::cout << "GNC TLS: Kernel weights have converged to binary values: " << max_weight 
      //<< " (total weight: " << tot << ")" << std::endl;
      return true;
    }
    // no kernel convergence criterion for GM
    case GncLossType::GM:
      return false;
    case GncLossType::DCS:
      return false;
    default:
      throw std::runtime_error("GncOptimizer::checkKernelConvergence: called with unknown loss type.");
  }

  return false;
}

void GNCSparseOptimizer::updateKernels(const double mu)
{
  switch(lossType_)
  {
    case GncLossType::GM:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuGM* rk = static_cast<RobustMuGM*>(e->robustKernel());
        rk->setMu(mu);
        e->computeError();
      }
      return; 
    case GncLossType::TLS:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuTLS* rk = static_cast<RobustMuTLS*>(e->robustKernel());
        rk->setMu(mu);
        e->computeError();
      } 
      return;
    case GncLossType::DCS:
      for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
      {
        OptimizableGraph::Edge* e = egrad_[idx];
        RobustMuDCS* rk = static_cast<RobustMuDCS*>(e->robustKernel());
        rk->setMu(mu);
        e->computeError();
      } 
      return;
    default:
      throw std::runtime_error("GncOptimizer::updateKernels: called with unknown loss type.");
  }

  return;
}

void GNCSparseOptimizer::resetKernels()
{
  for (int idx = 0; idx < static_cast<int>(egrad_.size()); ++idx)
  {
    OptimizableGraph::Edge* e = egrad_[idx];
    e->setRobustKernel(nullptr);
  }

  egrad_.clear();
  return;
}

void GNCSparseOptimizer::setAlpha(const double alpha)
{
  alpha_ = alpha;
  return;
}

// Computing residual for re-weighted least squares technique
double GNCSparseOptimizer::computeResidual(OptimizableGraph::Edge* edge)
{
  edge->computeError();
  Eigen::VectorXd error(edge->dimension());
  for (size_t v_idx = 0; v_idx < error.size(); ++v_idx)
    error[v_idx] = edge->errorData()[v_idx];

  return error.squaredNorm();
}

void GNCSparseOptimizer::clear()
{
  egrad_.clear();
  SparseOptimizer::clear();
  return;
}


}  // namespace g2o
