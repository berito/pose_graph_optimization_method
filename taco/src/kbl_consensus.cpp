#include "taco/kbl_consensus.hpp"

using namespace std;
using namespace g2o;


template <class EDGE, class VERTEX>
KBL_IPC<EDGE, VERTEX>::KBL_IPC(g2o::SparseOptimizer& open_loop_problem, const Config& cfg)
{
    _problem = &open_loop_problem;
    getProblemOdom<EDGE>(*_problem, _odom_edges);
    std::sort(_odom_edges.begin(), _odom_edges.end(), cmpEdgesID);

    _s_factor = cfg.s_factor;
    robustifyVoters<EDGE>(0, _odom_edges.size(), _s_factor, _odom_edges);
    propagateGuess<EDGE, VERTEX>(*_problem, 0, _odom_edges.size(), _odom_edges);
    store<VERTEX>(*_problem);

    _max_consensus_set.clear();

    _fast_reject_th = cfg.fast_reject_th;
    _fast_reject_iter_base = cfg.fast_reject_iter_base;
    _slow_reject_th = cfg.slow_reject_th;
    _slow_reject_iter_base = cfg.slow_reject_iter_base;

    _use_kbest = cfg.use_best_k_buddies;
    _k = cfg.k_buddies;
}

template <class EDGE, class VERTEX>
KBL_IPC<EDGE, VERTEX>::~KBL_IPC()
{
    _problem->clear();
    _max_consensus_set.clear();
}

template <class EDGE, class VERTEX>
bool KBL_IPC<EDGE, VERTEX>::agreementCheck(EDGE* loop_candidate)
{
    OptimizableGraph::EdgeSet eset_independent;
    pair<int, int> clust_ext = (_use_kbest && _k > 0)
        ? computeKBestSubgraph(*loop_candidate, eset_independent)
        : computeIndependentSubgraph(*loop_candidate, eset_independent);

    bool intersection_found = (eset_independent.size() > 0);
    double reject_th = intersection_found ? _slow_reject_th : _fast_reject_th;
    int reject_iter_base = intersection_found ? _slow_reject_iter_base : _fast_reject_iter_base;

    for ( size_t j = clust_ext.first ; j < clust_ext.second; eset_independent.insert(_odom_edges[j++]) );
    eset_independent.insert(loop_candidate);

    store<VERTEX>(*_problem);
    fixComplementary(*_problem, clust_ext.first, clust_ext.second);

    bool is_agreeing = isAgreeingWithCurrentState<EDGE>(*_problem, eset_independent, reject_th, reject_iter_base);
    if (!is_agreeing)
    {
        restore<VERTEX>(*_problem);
        return false;
    }

    discard<VERTEX>(*_problem);
    _max_consensus_set.push_back(loop_candidate);
    propagateCurrentGuess<EDGE, VERTEX>(*_problem, clust_ext.second, _odom_edges);

    return true;
}

template <class EDGE, class VERTEX>
bool KBL_IPC<EDGE, VERTEX>::removeEdgeFromCnS(EDGE* edge)
{
    int v0_id = min(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    int v1_id = max(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    for (auto it = _max_consensus_set.begin(); it != _max_consensus_set.end(); ++it)
    {
        EDGE* trusty_edge = *it;
        int vt0_id = min(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        int vt1_id = max(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        if (v0_id != vt0_id || v1_id != vt1_id ) continue;
        _max_consensus_set.erase(it);
        return true;
    }
    return false;
}

template <class EDGE, class VERTEX>
void KBL_IPC<EDGE, VERTEX>::addEdgeToCnS(EDGE* edge)
{
    int v0_id = min(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    int v1_id = max(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    for (const auto& trusty_edge : _max_consensus_set)
    {
        int vt0_id = min(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        int vt1_id = max(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        if (v0_id == vt0_id && v1_id == vt1_id) return;
    }
    _max_consensus_set.push_back(edge);
    std::sort(_max_consensus_set.begin(), _max_consensus_set.end(), cmpEdgesTime);
    return;
}

template <class EDGE, class VERTEX>
pair<int, int> KBL_IPC<EDGE, VERTEX>::computeIndependentSubgraph(const EDGE& edge_candidate,
                                                                OptimizableGraph::EdgeSet& eset)
{
    int v0_id = min(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());
    int v1_id = max(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());
    pair<int, int> extremes(v0_id, v1_id);

    bool found_new_extremes = true;
    vector<bool> included_edges(_max_consensus_set.size(), false);
    while (found_new_extremes)
    {
        found_new_extremes = false;
        for (size_t edge_cidx = 0; edge_cidx < _max_consensus_set.size(); ++edge_cidx)
        {
            if (included_edges[edge_cidx]) continue;
            int vt0_id = min(_max_consensus_set[edge_cidx]->vertices()[0]->id(),
                             _max_consensus_set[edge_cidx]->vertices()[1]->id());
            int vt1_id = max(_max_consensus_set[edge_cidx]->vertices()[0]->id(),
                             _max_consensus_set[edge_cidx]->vertices()[1]->id());
            int intersection = min(vt1_id, extremes.second) - max(vt0_id, extremes.first);
            if (intersection <= 0) continue;
            extremes.first = min(extremes.first, vt0_id);
            extremes.second = max(extremes.second, vt1_id);
            included_edges[edge_cidx] = true;
            found_new_extremes = true;
            eset.insert(_max_consensus_set[edge_cidx]);
        }
    }
    return extremes;
}

template <class EDGE, class VERTEX>
pair<int, int> KBL_IPC<EDGE, VERTEX>::computeKBestSubgraph(const EDGE& edge_candidate,
                                                          OptimizableGraph::EdgeSet& eset)
{
    int a = min(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());
    int b = max(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());

    vector<pair<double, size_t>> scored;   // (IoU, consensus-set index)
    for (size_t i = 0; i < _max_consensus_set.size(); ++i)
    {
        int c = min(_max_consensus_set[i]->vertices()[0]->id(), _max_consensus_set[i]->vertices()[1]->id());
        int d = max(_max_consensus_set[i]->vertices()[0]->id(), _max_consensus_set[i]->vertices()[1]->id());

        int inter = min(b, d) - max(a, c);          // node-span overlap (IPC convention)
        if (inter <= 0) continue;                    // only intersecting loops are candidates
        int uni = max(b, d) - min(a, c);
        double iou = uni > 0 ? (double)inter / (double)uni : 0.0;
        scored.push_back(make_pair(iou, i));
    }

    std::sort(scored.begin(), scored.end(),
              [](const pair<double, size_t>& x, const pair<double, size_t>& y){ return x.first > y.first; });

    size_t keep = min((size_t)_k, scored.size());
    pair<int, int> extremes(a, b);
    for (size_t r = 0; r < keep; ++r)
    {
        EDGE* e = _max_consensus_set[scored[r].second];
        int c = min(e->vertices()[0]->id(), e->vertices()[1]->id());
        int d = max(e->vertices()[0]->id(), e->vertices()[1]->id());
        extremes.first = min(extremes.first, c);
        extremes.second = max(extremes.second, d);
        eset.insert(e);
    }
    return extremes;
}


template class KBL_IPC<EdgeSE2, VertexSE2>;
template class KBL_IPC<EdgeSE3, VertexSE3>;
