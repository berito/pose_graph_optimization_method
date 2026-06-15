#pragma once

#include "ipc/consensus_utils.hpp"

// DC-IPC: correlation-aware variant of IPC. Loop closures in the same correlation group are
// decided jointly via discrete-continuous alternation (brief §4). λ=0 → per-edge IPC.
template <class EDGE, class VERTEX> class DC_IPC
{
public :

    DC_IPC(g2o::SparseOptimizer& open_loop_problem, const Config& cfg);
    ~DC_IPC();

    // Per-candidate test (== kBL-IPC; kept as λ=0 / singleton baseline / fallback).
    bool agreementCheck(EDGE* loop_candidate);

    // Group-joint test (brief §4): returns one accept/reject flag per member, aligned to `group`.
    std::vector<bool> agreementCheckGroup(const std::vector<EDGE*>& group);

    void setLambda(double lambda) { _lambda = lambda; }

    bool removeEdgeFromCnS(EDGE* edge);
    void addEdgeToCnS(EDGE* edge);

    const std::vector<EDGE*>& getMaxConsensusSet() const { return _max_consensus_set; }

private :

    std::pair<int, int> computeIndependentSubgraph(const EDGE& edge_candidate,
                                                   g2o::OptimizableGraph::EdgeSet& eset);
    std::pair<int, int> computeKBestSubgraph(const EDGE& edge_candidate,
                                             g2o::OptimizableGraph::EdgeSet& eset);

    g2o::SparseOptimizer* _problem;

    std::vector<EDGE*> _max_consensus_set;
    std::vector<EDGE*> _odom_edges;

    double _fast_reject_th;
    int    _fast_reject_iter_base;
    double _slow_reject_th;
    int    _slow_reject_iter_base;
    double _s_factor;
    double _lambda = 0.0;   // correlation-reward strength (brief eq. term 4); 0 → IPC

    bool _use_kbest;
    int  _k;

    // Exact discrete enumeration cap (brief §4 note on |L|>16 fallback).
    static constexpr int MAX_ENUM_SIZE = 16;
};
