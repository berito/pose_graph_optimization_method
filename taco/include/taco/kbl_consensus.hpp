#pragma once

#include "ipc/consensus_utils.hpp"

template <class EDGE, class VERTEX> class KBL_IPC
{
public :

    KBL_IPC(g2o::SparseOptimizer& open_loop_problem, const Config& cfg);
    ~KBL_IPC();

    bool agreementCheck(EDGE* loop_candidate);
    bool removeEdgeFromCnS(EDGE* edge);
    void addEdgeToCnS(EDGE* edge);

    const std::vector<EDGE*>& getMaxConsensusSet() const { return _max_consensus_set; }

private :

    // Full transitive minimal independent subgraph (== IPC, used when _use_kbest is false / k>=size).
    std::pair<int, int> computeIndependentSubgraph(const EDGE& edge_candidate,
                                                   g2o::OptimizableGraph::EdgeSet& eset);

    // kBL subgraph: the k accepted loops with highest IoU vs the candidate (Eq. 5.6).
    std::pair<int, int> computeKBestSubgraph(const EDGE& edge_candidate,
                                             g2o::OptimizableGraph::EdgeSet& eset);

    g2o::SparseOptimizer* _problem;

    std::vector<EDGE*> _max_consensus_set;
    std::vector<EDGE*> _odom_edges;

    double _fast_reject_th;
    int _fast_reject_iter_base;
    double _slow_reject_th;
    int _slow_reject_iter_base;
    double _s_factor;

    bool _use_kbest;
    int _k;
};
