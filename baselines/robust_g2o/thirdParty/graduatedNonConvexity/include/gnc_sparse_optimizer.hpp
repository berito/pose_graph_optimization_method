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

#ifndef G2O_GRAPH_OPTIMIZER_GNC_H_
#define G2O_GRAPH_OPTIMIZER_GNC_H_

#include "g2o/core/batch_stats.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/g2o_core_api.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_optimizer.h"

namespace g2o {

enum GncLossType {
  GM /*Geman McClure*/,
  TLS /*Truncated least squares*/,
  DCS /*Dynamic Covariance Scaling*/
};

class G2O_CORE_API GNCSparseOptimizer : public SparseOptimizer {

  friend class ActivePathCostFunction;

  // Attention: _solver & _statistics is own by SparseOptimizer and will be
  // deleted in its destructor.
public:
  GNCSparseOptimizer(GncLossType lossType);
  GNCSparseOptimizer();
  ~GNCSparseOptimizer();

  int optimizeX(int iterations, bool online);
  int optimizeY(int iterations, bool online);

  void setKnownInliers(EdgeContainer& inliers);
  void setInnerIterations(int iterations);
  void setMuStep(double mu_step);
  void setAlpha(const double alpha);
  int defaultOptimize(bool online);
  bool isEdgeInlier(const int idx);
  void resetKernels();
  void clear();

 private:
    
    double initializeMu() const;
    void initializeKernels(const double mu);
    double updateMu(const double mu) const;
    bool checkMuConvergence(const double mu) const;
    bool checkConvergence(const double mu, const double cost, const double prev_cost) const;
    bool checkCostConvergence(const double cost, const double prev_cost) const;
    bool checkKernelConvergence() const;    
    void updateKernels(const double mu);
    double computeResidual(OptimizableGraph::Edge* edge);

    EdgeContainer egrad_; // edges with robust estimator
    double inlier_th_;
    int inner_iters_ = 100;

    double alpha_;

    GncLossType lossType_ = GncLossType::TLS;  ///< Default loss
    double muStep_ = 1.4;  ///< Multiplicative factor to reduce/increase the mu in gnc
    double weightsTol_ = 1e-4;  ///< If the weights are within weightsTol from being binary, stop iterating (only for TLS)
};
}  // namespace g2o

#endif