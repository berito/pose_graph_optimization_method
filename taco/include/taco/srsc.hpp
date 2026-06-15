#pragma once

#include "ipc/consensus_utils.hpp"

// SR-SC — Selective Randomized Switchable Constraints (TACO Module B, thesis §5.3.4, Alg. 7).
// Retrospective recovery pass: re-judges the unrevised batch on a minimal trusted subgraph using
// switchable constraints under randomly-perturbed priors, then promotes loops that survive the vote.
// Reuses the vertigo switchable types vendored in baselines/robust_g2o/thirdParty/switchableConstraints.
template <class EDGE, class VERTEX> class SR_SC
{
public :

    SR_SC(g2o::SparseOptimizer& problem, const Config& cfg);

    // Revise one unrevised batch. Inputs: the batch Ē_l and the current inlier labels (consensus set).
    // Returns the loops promoted to inlier this run (subset of the batch). Empty if nothing flips.
    std::vector<EDGE*> revise(const std::vector<EDGE*>& unrevised,
                              const std::vector<EDGE*>& inliers);

private :

    g2o::SparseOptimizer* _problem;
    double _s_factor;
    double _eps;          // random prior-flip fraction ε (= 0.3)
    int    _max_iter;     // voting rounds maxIter (= 20)
    double _inlier_th;    // promotion threshold inlierTh (= 0.7)
};
