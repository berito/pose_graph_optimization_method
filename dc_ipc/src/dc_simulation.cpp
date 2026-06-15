#include "dc_ipc/dc_simulation.hpp"
#include <fstream>
#include <sstream>
#include <map>

using namespace std;
using namespace g2o;

static const int SRSC_BATCH_R = 10;

// ── loadGroups ────────────────────────────────────────────────────────────────

template <class EDGE>
vector<vector<EDGE*>> loadGroups(const string& dataset_path, const vector<EDGE*>& loops)
{
    // Index loops by (min_id, max_id).
    map<pair<int,int>, EDGE*> by_span;
    for (EDGE* e : loops)
    {
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        by_span[{min(a,b), max(a,b)}] = e;
    }

    string gpath = dataset_path.substr(0, dataset_path.find_last_of('.')) + ".groups";
    ifstream in(gpath);
    if (!in.good())
    {
        // No sidecar → singleton groups (correlation-blind baseline).
        vector<vector<EDGE*>> singletons;
        for (EDGE* e : loops) singletons.push_back({e});
        return singletons;
    }

    vector<vector<EDGE*>> groups;
    string line;
    set<pair<int,int>> claimed;
    while (getline(in, line))
    {
        istringstream ss(line);
        vector<int> ids; int v;
        while (ss >> v) ids.push_back(v);
        if (ids.size() < 2) continue;
        vector<EDGE*> g;
        for (size_t i = 0; i + 1 < ids.size(); i += 2)
        {
            pair<int,int> key(min(ids[i],ids[i+1]), max(ids[i],ids[i+1]));
            auto it = by_span.find(key);
            if (it != by_span.end()) { g.push_back(it->second); claimed.insert(key); }
        }
        if (!g.empty()) groups.push_back(g);
    }
    // Any loop not named in the sidecar becomes its own singleton group.
    for (EDGE* e : loops)
    {
        int a = e->vertices()[0]->id(), b = e->vertices()[1]->id();
        if (!claimed.count({min(a,b), max(a,b)})) groups.push_back({e});
    }
    return groups;
}

// ── Main driver ───────────────────────────────────────────────────────────────
// Processes loop closures in arrival time order, but decides each correlation
// group atomically (brief §4): when the first member of a group is reached, the
// whole group is decided together via agreementCheckGroup.

template <class T, class EDGE, class VERTEX>
void simulating_incremental_data_dc(const Config& cfg,
                                    SparseOptimizer& open_loop_problem,
                                    const vector<EDGE*>& loops,
                                    double lambda)
{
    vector<T> poses_gt;
    readSolutionFile(poses_gt, cfg.ground_truth);
    double s_factor = cfg.s_factor;

    int tot_hypothesis = loops.size();
    vector<int> bucket(tot_hypothesis, 1);
    vector<pair<bool, EDGE*>> gt_loops;

    for (size_t idx = 0; idx < cfg.canonic_inliers; gt_loops.push_back(make_pair(true,  loops[idx++])));
    for (size_t idx = cfg.canonic_inliers; idx < loops.size(); gt_loops.push_back(make_pair(false, loops[idx++])));
    sort(gt_loops.begin(), gt_loops.end(), cmpTime);

    // Build ordered loop list and load correlation groups.
    vector<EDGE*> ordered; for (auto& p : gt_loops) ordered.push_back(p.second);
    vector<vector<EDGE*>> groups = loadGroups<EDGE>(cfg.dataset, ordered);

    DC_IPC<EDGE, VERTEX> dc(open_loop_problem, cfg);
    dc.setLambda(lambda);

    SR_SC<EDGE, VERTEX> srsc(open_loop_problem, cfg);

    // Map edge → index in gt_loops for bucket writes.
    map<EDGE*, size_t> idx_of;
    for (size_t i = 0; i < gt_loops.size(); ++i) idx_of[gt_loops[i].second] = i;

    // Map edge → which group it belongs to (by group index).
    // Used to fire the group decision when the first member in time-order arrives.
    map<EDGE*, size_t> edge_to_group;
    for (size_t gi = 0; gi < groups.size(); ++gi)
        for (EDGE* e : groups[gi])
            edge_to_group[e] = gi;

    set<size_t> decided_groups;  // group indices already processed

    vector<pair<size_t, EDGE*>> unrevised;

    double avg_time = 0.0;
    cout << "Starting DC-IPC simulation -> Displaying relative status :" << endl;
    cout << "S = " << s_factor
         << " | TH = " << cfg.fast_reject_th
         << " | lambda = " << lambda
         << " | kBL = " << (cfg.use_best_k_buddies ? cfg.k_buddies : 0)
         << " | recovery = " << (cfg.use_recovery ? "on" : "off")
         << " | groups = " << groups.size() << " (loops=" << tot_hypothesis << ")" << endl;

    for (size_t candidate_id = 0; candidate_id < (size_t)tot_hypothesis; ++candidate_id)
    {
        EDGE* cand_edge = gt_loops[candidate_id].second;

        // If this edge's group has already been decided, skip (verdict in bucket).
        size_t gi = edge_to_group[cand_edge];
        if (decided_groups.count(gi)) continue;

        decided_groups.insert(gi);
        const vector<EDGE*>& grp = groups[gi];

        chrono::steady_clock::time_point begin = chrono::steady_clock::now();
        vector<bool> verdicts = dc.agreementCheckGroup(grp);
        chrono::steady_clock::time_point end = chrono::steady_clock::now();

        // Write verdicts for every member of the group.
        for (size_t mi = 0; mi < grp.size(); ++mi)
        {
            size_t midx = idx_of[grp[mi]];
            bucket[midx] = verdicts[mi] ? 1 : 0;
            if (cfg.use_recovery)
                unrevised.push_back(make_pair(midx, grp[mi]));
        }

        // SR-SC retrospective pass (inherited from TACO — fires every SRSC_BATCH_R decisions).
        if (cfg.use_recovery && (int)unrevised.size() >= SRSC_BATCH_R)
        {
            vector<EDGE*> batch; for (auto& p : unrevised) batch.push_back(p.second);
            vector<EDGE*> promoted = srsc.revise(batch, dc.getMaxConsensusSet());
            for (EDGE* e : promoted)
                for (auto& p : unrevised)
                    if (p.second == e && bucket[p.first] == 0)
                    {
                        bucket[p.first] = 1;
                        dc.addEdgeToCnS(e);
                    }
            unrevised.clear();
        }

        avg_time += chrono::duration_cast<chrono::microseconds>(end - begin).count() / 1000000.0;
        printProgress((double)(candidate_id + 1) / (double)tot_hypothesis);
    }
    cout << "\nCompleted!" << endl;

    // ── Final global optimization ─────────────────────────────────────────────
    vector<EDGE*> odom_edges;
    getProblemOdom<EDGE>(open_loop_problem, odom_edges);
    propagateGuess<EDGE, VERTEX>(open_loop_problem, 0, odom_edges.size(), odom_edges);

    OptimizableGraph::EdgeSet eset_gl;
    for (size_t i = 0; i < odom_edges.size(); ++i)
    {
        odom_edges[i]->setInformation(odom_edges[i]->information() / s_factor);
        eset_gl.insert(odom_edges[i]);
    }
    for (size_t i = 0; i < gt_loops.size(); ++i)
        if (bucket[i] == 1) eset_gl.insert(gt_loops[i].second);

    open_loop_problem.initializeOptimization(eset_gl);
    open_loop_problem.optimize(1000);

    // ── Metrics ───────────────────────────────────────────────────────────────
    int tp = 0, fp = 0, tn = 0, fn = 0;
    for (size_t c = 0; c < gt_loops.size(); ++c)
        if      ( gt_loops[c].first && bucket[c] == 1) ++tp;
        else if ( gt_loops[c].first && bucket[c] == 0) ++fn;
        else if (!gt_loops[c].first && bucket[c] == 1) ++fp;
        else if (!gt_loops[c].first && bucket[c] == 0) ++tn;

    float precision = tp / (float)(tp + fp);
    float recall    = tp / (float)(tp + fn);
    cout << "Size of MAX consistent set = " << dc.getMaxConsensusSet().size() << endl;
    cout << "Avg Time x group = " << avg_time / groups.size() << " [s]\n";
    cout << "Precision = " << precision << "\nRecall = " << recall << endl;

    // ── Write outputs ─────────────────────────────────────────────────────────
    ofstream outfile(cfg.output.c_str());
    for (size_t it = 0; it < open_loop_problem.vertices().size(); ++it)
        writeVertex(outfile, dynamic_cast<VERTEX*>(open_loop_problem.vertex(it)));
    outfile.close();

    string out2 = cfg.output.substr(0, cfg.output.size() - 3) + "PR";
    outfile.open(out2.c_str());
    outfile << precision << " " << recall << endl;
    outfile << avg_time << " " << avg_time / groups.size() << endl;
    outfile.close();
}

// ── Explicit instantiations ───────────────────────────────────────────────────

template vector<vector<EdgeSE2*>> loadGroups<EdgeSE2>(const string&, const vector<EdgeSE2*>&);
template vector<vector<EdgeSE3*>> loadGroups<EdgeSE3>(const string&, const vector<EdgeSE3*>&);
template void simulating_incremental_data_dc<Eigen::Isometry2d, EdgeSE2, VertexSE2>(const Config&, SparseOptimizer&, const vector<EdgeSE2*>&, double);
template void simulating_incremental_data_dc<Eigen::Isometry3d, EdgeSE3, VertexSE3>(const Config&, SparseOptimizer&, const vector<EdgeSE3*>&, double);
