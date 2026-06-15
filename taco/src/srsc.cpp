#include "taco/srsc.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "switchableConstraints/include/vertex_switchLinear.hpp"
#include "switchableConstraints/include/edge_switchPrior.hpp"
#include "switchableConstraints/include/edge_se2Switchable.hpp"
#include "switchableConstraints/include/edge_se3Switchable.hpp"

#include <random>
#include <set>

using namespace std;
using namespace g2o;

// ---- measurement conversion helpers (2D vs 3D) --------------------------

static EdgeSE2Switchable* makeSwitchable(EdgeSE2* src, VertexSE2* va, VertexSE2* vb,
                                         VertexSwitchLinear* sw)
{
    EdgeSE2Switchable* e = new EdgeSE2Switchable();
    e->setVertex(0, va);
    e->setVertex(1, vb);
    e->setVertex(2, sw);
    e->setMeasurement(src->measurement());
    e->setInformation(src->information());
    return e;
}

static EdgeSE3Switchable* makeSwitchable(EdgeSE3* src, VertexSE3* va, VertexSE3* vb,
                                         VertexSwitchLinear* sw)
{
    EdgeSE3Switchable* e = new EdgeSE3Switchable();
    e->setVertex(0, va);
    e->setVertex(1, vb);
    e->setVertex(2, sw);
    Eigen::Isometry3d iso = src->measurement();
    e->setMeasurement(SE3Quat(Eigen::Quaterniond(iso.rotation()), iso.translation()));
    e->setInformation(src->information());
    return e;
}

// ---- constructor ---------------------------------------------------------

template <class EDGE, class VERTEX>
SR_SC<EDGE, VERTEX>::SR_SC(g2o::SparseOptimizer& problem, const Config& cfg)
{
    _problem   = &problem;
    _s_factor  = cfg.s_factor;
    _eps       = 0.3;
    _max_iter  = 20;
    _inlier_th = 0.7;
}

// ---- helpers -------------------------------------------------------------

// Build pair (min_id, max_id) for an edge.
static pair<int,int> edgeSpan(g2o::OptimizableGraph::Edge* e)
{
    int a = e->vertices()[0]->id();
    int b = e->vertices()[1]->id();
    return { min(a,b), max(a,b) };
}

// Return true if edge pair is in the inlier set (by vertex ids).
template <class EDGE>
static bool isInlier(EDGE* e, const vector<EDGE*>& inliers)
{
    auto sp = edgeSpan(e);
    for (EDGE* ie : inliers)
        if (edgeSpan(ie) == sp) return true;
    return false;
}

// Compute node-span [lo, hi] for a set of batch loops (the seeds).
// Then expand it transitively through trusted edges whose span intersects.
// Returns {lo, hi} and fills trusted_in with the trusted edges inside the span.
template <class EDGE>
static pair<int,int> buildSpan(const vector<EDGE*>& batch,
                                const vector<EDGE*>& trusted,
                                vector<EDGE*>& trusted_in)
{
    // Seed from batch spans.
    int lo = INT_MAX, hi = INT_MIN;
    for (EDGE* e : batch)
    {
        auto sp = edgeSpan(e);
        lo = min(lo, sp.first);
        hi = max(hi, sp.second);
    }

    // Transitively expand through trusted loops (same logic as IPC's computeIndependentSubgraph).
    vector<bool> included(trusted.size(), false);
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (size_t i = 0; i < trusted.size(); ++i)
        {
            if (included[i]) continue;
            auto sp = edgeSpan(trusted[i]);
            int inter = min(sp.second, hi) - max(sp.first, lo);
            if (inter <= 0) continue;
            lo = min(lo, sp.first);
            hi = max(hi, sp.second);
            included[i] = true;
            trusted_in.push_back(trusted[i]);
            changed = true;
        }
    }
    return { lo, hi };
}

// Create and configure a self-contained LM optimizer (does not share the g2o
// algorithm/solver of the global problem).
static SparseOptimizer* makeSubOptimizer()
{
    typedef BlockSolverX BlockSolverType;
    typedef LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto blockSolver  = g2o::make_unique<BlockSolverType>(std::move(linearSolver));
    OptimizationAlgorithm* algo = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

    SparseOptimizer* opt = new SparseOptimizer();
    opt->setAlgorithm(algo);
    opt->setVerbose(false);
    return opt;
}

// ---- revise() ------------------------------------------------------------

// Algorithm 7: one SR-SC trigger.
template <class EDGE, class VERTEX>
vector<EDGE*> SR_SC<EDGE, VERTEX>::revise(const vector<EDGE*>& unrevised,
                                          const vector<EDGE*>& inliers)
{
    if (unrevised.empty()) return {};

    // 1. trusted set = inliers \ batch (by vertex-pair key).
    set<pair<int,int>> batch_keys;
    for (EDGE* e : unrevised) batch_keys.insert(edgeSpan(e));

    vector<EDGE*> trusted;
    for (EDGE* e : inliers)
        if (!batch_keys.count(edgeSpan(e)))
            trusted.push_back(e);

    // 2. Build trusted subgraph span.
    vector<EDGE*> trusted_in;
    auto span = buildSpan<EDGE>(unrevised, trusted, trusted_in);
    int lo = span.first;
    int hi = span.second;

    // Collect odom edges inside [lo, hi) from _problem.
    vector<EDGE*> odom_in_span;
    for (auto it = _problem->edges().begin(); it != _problem->edges().end(); ++it)
    {
        EDGE* e = dynamic_cast<EDGE*>(*it);
        if (!e) continue;
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        if (abs(a - b) != 1) continue;      // odom edges only
        int mn = min(a,b), mx = max(a,b);
        if (mn >= lo && mx <= hi) odom_in_span.push_back(e);
    }

    // 3. Build self-contained subproblem.
    SparseOptimizer* sub = makeSubOptimizer();

    // Pose vertices: node ids lo..hi, seeded from global estimates.
    for (int id = lo; id <= hi; ++id)
    {
        VERTEX* gv = dynamic_cast<VERTEX*>(_problem->vertex(id));
        if (!gv) continue;
        VERTEX* sv = new VERTEX();
        sv->setId(id);
        sv->setEstimate(gv->estimate());
        sv->setFixed(id == lo);   // gauge
        sub->addVertex(sv);
    }

    // Odometry edges (plain EDGE).
    for (EDGE* e : odom_in_span)
    {
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        VERTEX* va = dynamic_cast<VERTEX*>(sub->vertex(a));
        VERTEX* vb = dynamic_cast<VERTEX*>(sub->vertex(b));
        if (!va || !vb) continue;
        EDGE* ne = new EDGE();
        ne->setVertex(0, va);
        ne->setVertex(1, vb);
        ne->setMeasurement(e->measurement());
        ne->setInformation(e->information());   // already ×s_factor on _problem during the online phase
        sub->addEdge(ne);
    }

    // Trusted loop edges (plain EDGE, fixed geometry).
    for (EDGE* e : trusted_in)
    {
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        VERTEX* va = dynamic_cast<VERTEX*>(sub->vertex(a));
        VERTEX* vb = dynamic_cast<VERTEX*>(sub->vertex(b));
        if (!va || !vb) continue;
        EDGE* ne = new EDGE();
        ne->setVertex(0, va);
        ne->setVertex(1, vb);
        ne->setMeasurement(e->measurement());
        ne->setInformation(e->information());
        sub->addEdge(ne);
    }

    // Switch vertices + switchable loop + prior — one per batch edge.
    // Switch vertex ids start above the largest pose id to avoid collisions.
    int sw_id_base = hi + 1;
    const double SW_PRIOR_WEIGHT = 1.0;  // §5.3.4: unit weight on the switch prior

    vector<VertexSwitchLinear*> sw_verts(unrevised.size(), nullptr);
    vector<EdgeSwitchPrior*>    sw_priors(unrevised.size(), nullptr);

    for (size_t i = 0; i < unrevised.size(); ++i)
    {
        EDGE* e = unrevised[i];
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        VERTEX* va = dynamic_cast<VERTEX*>(sub->vertex(a));
        VERTEX* vb = dynamic_cast<VERTEX*>(sub->vertex(b));
        if (!va || !vb) continue;   // endpoints outside span — skip

        double gamma = isInlier(e, inliers) ? 1.0 : 0.0;

        VertexSwitchLinear* sw = new VertexSwitchLinear();
        sw->setId(sw_id_base + (int)i);
        sw->setEstimate(gamma);
        sub->addVertex(sw);
        sw_verts[i] = sw;

        // Switchable loop edge.
        auto* se = makeSwitchable(e, va, vb, sw);
        sub->addEdge(se);

        // Switch prior.
        EdgeSwitchPrior* prior = new EdgeSwitchPrior();
        prior->setVertex(0, sw);
        prior->setMeasurement(gamma);
        Eigen::Matrix<double,1,1> info;
        info(0,0) = SW_PRIOR_WEIGHT;
        prior->setInformation(info);
        sub->addEdge(prior);
        sw_priors[i] = prior;
    }

    // 4. Randomized voting (Alg. 7, lines 6-13).
    // Seed derived from span so runs are dataset-deterministic (reproducible).
    mt19937 rng(static_cast<uint32_t>(lo) * 1000u + static_cast<uint32_t>(hi));
    bernoulli_distribution flip(_eps);

    vector<int> acc(unrevised.size(), 0);

    for (int r = 0; r < _max_iter; ++r)
    {
        // Set priors from current inlier labels, then randomly flip ε fraction.
        for (size_t i = 0; i < unrevised.size(); ++i)
        {
            if (!sw_verts[i]) continue;
            double gamma = isInlier(unrevised[i], inliers) ? 1.0 : 0.0;
            if (flip(rng)) gamma = 1.0 - gamma;

            sw_priors[i]->setMeasurement(gamma);
            sw_verts[i]->setEstimate(gamma);   // reset to prior before optimize
        }

        sub->initializeOptimization();
        sub->optimize(50);

        for (size_t i = 0; i < unrevised.size(); ++i)
            if (sw_verts[i] && sw_verts[i]->estimate() > 0.5)
                ++acc[i];
    }

    // 5. Promote: avgVote > inlierTh AND currently not an inlier.
    vector<EDGE*> promoted;
    for (size_t i = 0; i < unrevised.size(); ++i)
    {
        if (!sw_verts[i]) continue;
        double avg = (double)acc[i] / (double)_max_iter;
        if (avg > _inlier_th && !isInlier(unrevised[i], inliers))
            promoted.push_back(unrevised[i]);
    }

    delete sub;
    return promoted;
}


template class SR_SC<EdgeSE2, VertexSE2>;
template class SR_SC<EdgeSE3, VertexSE3>;
