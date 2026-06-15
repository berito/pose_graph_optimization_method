#include "dc_ipc/dc_consensus.hpp"
#include <glog/logging.h>
#include <cstdint>
#include <numeric>

using namespace std;
using namespace g2o;

// ── Constructor / Destructor ─────────────────────────────────────────────────

template <class EDGE, class VERTEX>
DC_IPC<EDGE, VERTEX>::DC_IPC(g2o::SparseOptimizer& open_loop_problem, const Config& cfg)
{
    _problem = &open_loop_problem;
    getProblemOdom<EDGE>(*_problem, _odom_edges);
    std::sort(_odom_edges.begin(), _odom_edges.end(), cmpEdgesID);

    _s_factor = cfg.s_factor;
    robustifyVoters<EDGE>(0, _odom_edges.size(), _s_factor, _odom_edges);
    propagateGuess<EDGE, VERTEX>(*_problem, 0, _odom_edges.size(), _odom_edges);
    store<VERTEX>(*_problem);

    _max_consensus_set.clear();

    _fast_reject_th       = cfg.fast_reject_th;
    _fast_reject_iter_base = cfg.fast_reject_iter_base;
    _slow_reject_th       = cfg.slow_reject_th;
    _slow_reject_iter_base = cfg.slow_reject_iter_base;

    _use_kbest = cfg.use_best_k_buddies;
    _k         = cfg.k_buddies;
    // _lambda stays 0.0 until setLambda() is called.
}

template <class EDGE, class VERTEX>
DC_IPC<EDGE, VERTEX>::~DC_IPC()
{
    _problem->clear();
    _max_consensus_set.clear();
}

// ── Per-edge agreement check (IPC / kBL-IPC baseline) ────────────────────────
// Identical to ipc/src/consensus.cpp agreementCheck; retained as the λ=0 path.

template <class EDGE, class VERTEX>
bool DC_IPC<EDGE, VERTEX>::agreementCheck(EDGE* loop_candidate)
{
    OptimizableGraph::EdgeSet eset_independent;
    pair<int, int> clust_ext = (_use_kbest && _k > 0)
        ? computeKBestSubgraph(*loop_candidate, eset_independent)
        : computeIndependentSubgraph(*loop_candidate, eset_independent);

    bool intersection_found = (eset_independent.size() > 0);
    double reject_th        = intersection_found ? _slow_reject_th        : _fast_reject_th;
    int    reject_iter_base = intersection_found ? _slow_reject_iter_base : _fast_reject_iter_base;

    for (size_t j = clust_ext.first; j < clust_ext.second; eset_independent.insert(_odom_edges[j++]));
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

// ── Group-joint agreement check (DC-IPC contribution, brief §4) ──────────────
//
// Minimises the 4-term objective over poses X and binary labels ℓ on the free
// loop set L = {group members} by alternating:
//   (a) CONTINUOUS — g2o optimize with ℓ=1 edges active   (brief §4 step 5a)
//   (b) DISCRETE   — exact 2^|L| enumeration (or per-edge fallback) minimising
//                    Σ_{ℓ=1} r_i + Σ_{ℓ=0} κ + λ Σ_{pairs} 1[ℓ_i≠ℓ_j]  (brief eq. terms 2–4)
//
// λ=0, singleton group → identical to agreementCheck (correctness gate).

template <class EDGE, class VERTEX>
vector<bool> DC_IPC<EDGE, VERTEX>::agreementCheckGroup(const vector<EDGE*>& group)
{
    // ── λ=0 / singleton fast path ─────────────────────────────────────────────
    // Brief §3: "reduces to IPC at λ=0" — per-edge decisions decouple exactly.
    if (_lambda == 0.0 || group.size() == 1)
    {
        vector<bool> verdict;
        verdict.reserve(group.size());
        for (EDGE* e : group)
            verdict.push_back(agreementCheck(e));
        return verdict;
    }

    const size_t G = group.size();

    // ── Step 1-2: union independent subgraphs of all group members (brief §4 steps 1-2) ──
    // Collect CnS loop edges that intersect ANY member, and compute the joint [lo, hi] span.
    pair<int,int> clust_ext(INT_MAX, INT_MIN);
    OptimizableGraph::EdgeSet cns_in_subgraph;  // trusted CnS loops inside the union span

    // First pass: compute per-member subgraph and union the spans / edge sets.
    for (EDGE* e : group)
    {
        OptimizableGraph::EdgeSet eset_m;
        pair<int,int> ext_m = (_use_kbest && _k > 0)
            ? computeKBestSubgraph(*e, eset_m)
            : computeIndependentSubgraph(*e, eset_m);

        clust_ext.first  = min(clust_ext.first,  ext_m.first);
        clust_ext.second = max(clust_ext.second, ext_m.second);
        for (auto* ge : eset_m) cns_in_subgraph.insert(ge);
    }
    // Also expand span to include each group member's own endpoints.
    for (EDGE* e : group)
    {
        int a = min(e->vertices()[0]->id(), e->vertices()[1]->id());
        int b = max(e->vertices()[0]->id(), e->vertices()[1]->id());
        clust_ext.first  = min(clust_ext.first,  a);
        clust_ext.second = max(clust_ext.second, b);
    }

    // Determine reject threshold: use slow if ANY member had an intersection.
    bool any_intersection = !cns_in_subgraph.empty();
    double kappa        = any_intersection ? _slow_reject_th        : _fast_reject_th;
    int    iter_base    = any_intersection ? _slow_reject_iter_base : _fast_reject_iter_base;
    int    iters        = ((size_t)(clust_ext.second - clust_ext.first) > 100)
                          ? iter_base * 5 : iter_base;  // same rule as isAgreeingWithCurrentState

    // ── Step 3: build sets ────────────────────────────────────────────────────
    // L = group members (free labels), odom_eset = odom edges in span.
    OptimizableGraph::EdgeSet odom_eset;
    for (size_t j = clust_ext.first; j < clust_ext.second; odom_eset.insert(_odom_edges[j++]));

    // ── Step 4: store + fix complementary (brief §4 step 4) ──────────────────
    store<VERTEX>(*_problem);
    fixComplementary(*_problem, clust_ext.first, clust_ext.second);

    // ── Step 5: Discrete-Continuous alternation (brief §4 step 5) ────────────
    const int MAX_ROUNDS = 10;
    vector<bool> labels(G, true);   // start: all group members labelled inlier (ℓ=1)
    vector<bool> prev_labels(G, false);

    // Helper: rebuild the active edge set given current labels.
    // Active = odom (always) + trusted CnS loops (ℓ fixed 1) + free loops with ℓ=1.
    auto buildActiveSet = [&]() -> OptimizableGraph::EdgeSet {
        OptimizableGraph::EdgeSet es = odom_eset;
        for (auto* ce : cns_in_subgraph) es.insert(ce);
        for (size_t i = 0; i < G; ++i)
            if (labels[i]) es.insert(group[i]);
        return es;
    };

    for (int round = 0; round < MAX_ROUNDS; ++round)
    {
        prev_labels = labels;

        // (a) CONTINUOUS: optimize poses with current labels.
        {
            OptimizableGraph::EdgeSet active = buildActiveSet();
            _problem->initializeOptimization(active);
            _problem->optimize(iters);
        }

        // (b) DISCRETE: compute per-member residuals at current X.
        vector<double> residuals(G);
        for (size_t i = 0; i < G; ++i)
        {
            group[i]->computeError();
            residuals[i] = group[i]->chi2();
        }

        if (G <= (size_t)MAX_ENUM_SIZE)
        {
            // Exact enumeration over 2^G assignments (brief §4: "exact enumeration of 2^|L|").
            // Cost(ℓ) = Σ_i [ℓ_i·r_i + (1-ℓ_i)·κ]  +  λ·Σ_{i<j} 1[ℓ_i≠ℓ_j]
            // All pairs within the group are coupled (brief §2: "ALL pairs within group ∈ C").
            double best_cost = 1e300;
            uint32_t best_mask = 0;
            uint32_t N = 1u << G;
            for (uint32_t mask = 0; mask < N; ++mask)
            {
                double cost = 0.0;
                for (size_t i = 0; i < G; ++i)
                {
                    bool li = (mask >> i) & 1u;
                    cost += li ? residuals[i] : kappa;
                }
                if (_lambda > 0.0)
                {
                    for (size_t i = 0; i < G; ++i)
                        for (size_t j = i + 1; j < G; ++j)
                        {
                            bool li = (mask >> i) & 1u;
                            bool lj = (mask >> j) & 1u;
                            if (li != lj) cost += _lambda;
                        }
                }
                if (cost < best_cost) { best_cost = cost; best_mask = mask; }
            }
            for (size_t i = 0; i < G; ++i)
                labels[i] = (best_mask >> i) & 1u;
        }
        else
        {
            // |L|>16 fallback: per-edge threshold, λ coupling ignored.
            LOG(WARNING) << "DC-IPC: group size " << G
                         << " > MAX_ENUM_SIZE=" << MAX_ENUM_SIZE
                         << "; falling back to per-edge threshold (λ ignored).";
            for (size_t i = 0; i < G; ++i)
                labels[i] = (residuals[i] < kappa);
        }

        // Check convergence.
        if (labels == prev_labels) break;
    }

    // ── Step 6: accept / reject each member (brief §4 step 6) ────────────────
    // If all are rejected restore poses; if at least one is accepted keep the
    // optimised state for the accepted members and propagate forward from the
    // maximum accepted high endpoint.
    bool any_accepted = false;
    for (bool l : labels) if (l) { any_accepted = true; break; }

    if (!any_accepted)
    {
        restore<VERTEX>(*_problem);
        return vector<bool>(G, false);
    }

    // At least one accepted: discard the saved snapshot and commit.
    discard<VERTEX>(*_problem);

    for (size_t i = 0; i < G; ++i)
        if (labels[i]) addEdgeToCnS(group[i]);

    // Propagate from the END of the optimized subgraph (== IPC), preserving the
    // optimized poses inside [lo, clust_ext.second]; only nodes beyond it are re-chained.
    propagateCurrentGuess<EDGE, VERTEX>(*_problem, clust_ext.second, _odom_edges);

    return vector<bool>(labels.begin(), labels.end());
}

// ── CnS bookkeeping ───────────────────────────────────────────────────────────

template <class EDGE, class VERTEX>
bool DC_IPC<EDGE, VERTEX>::removeEdgeFromCnS(EDGE* edge)
{
    int v0_id = min(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    int v1_id = max(edge->vertices()[0]->id(), edge->vertices()[1]->id());
    for (auto it = _max_consensus_set.begin(); it != _max_consensus_set.end(); ++it)
    {
        EDGE* trusty_edge = *it;
        int vt0_id = min(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        int vt1_id = max(trusty_edge->vertices()[0]->id(), trusty_edge->vertices()[1]->id());
        if (v0_id != vt0_id || v1_id != vt1_id) continue;
        _max_consensus_set.erase(it);
        return true;
    }
    return false;
}

template <class EDGE, class VERTEX>
void DC_IPC<EDGE, VERTEX>::addEdgeToCnS(EDGE* edge)
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
}

// ── Independent / kBL subgraph computation ───────────────────────────────────

template <class EDGE, class VERTEX>
pair<int, int> DC_IPC<EDGE, VERTEX>::computeIndependentSubgraph(const EDGE& edge_candidate,
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
            extremes.first  = min(extremes.first,  vt0_id);
            extremes.second = max(extremes.second, vt1_id);
            included_edges[edge_cidx] = true;
            found_new_extremes = true;
            eset.insert(_max_consensus_set[edge_cidx]);
        }
    }
    return extremes;
}

template <class EDGE, class VERTEX>
pair<int, int> DC_IPC<EDGE, VERTEX>::computeKBestSubgraph(const EDGE& edge_candidate,
                                                         OptimizableGraph::EdgeSet& eset)
{
    int a = min(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());
    int b = max(edge_candidate.vertices()[0]->id(), edge_candidate.vertices()[1]->id());

    vector<pair<double, size_t>> scored;
    for (size_t i = 0; i < _max_consensus_set.size(); ++i)
    {
        int c = min(_max_consensus_set[i]->vertices()[0]->id(), _max_consensus_set[i]->vertices()[1]->id());
        int d = max(_max_consensus_set[i]->vertices()[0]->id(), _max_consensus_set[i]->vertices()[1]->id());
        int inter = min(b, d) - max(a, c);
        if (inter <= 0) continue;
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
        extremes.first  = min(extremes.first,  c);
        extremes.second = max(extremes.second, d);
        eset.insert(e);
    }
    return extremes;
}

// ── Explicit instantiations ───────────────────────────────────────────────────

template class DC_IPC<EdgeSE2, VertexSE2>;
template class DC_IPC<EdgeSE3, VertexSE3>;
