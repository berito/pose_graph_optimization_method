#include "taco/taco_simulation.hpp"

using namespace std;
using namespace g2o;

static const int SRSC_BATCH_R = 10;   // thesis R: trigger SR-SC every 10 detections (§5.4)

template <class T, class EDGE, class VERTEX>
void simulating_incremental_data_taco(const Config& cfg,
                                      SparseOptimizer& open_loop_problem,
                                      const vector<EDGE*>& loops)
{
    vector<T> poses_gt;
    readSolutionFile(poses_gt, cfg.ground_truth);
    double s_factor = cfg.s_factor;

    int tot_hypothesis = loops.size();
    vector<int> bucket(tot_hypothesis, 1);
    vector<pair<bool, EDGE*>> gt_loops;
    vector<EDGE*> sorted_loops;

    for (size_t idx = 0 ; idx < cfg.canonic_inliers; gt_loops.push_back(make_pair(true, loops[idx++])));
    for (size_t idx = cfg.canonic_inliers ; idx < loops.size(); gt_loops.push_back(make_pair(false, loops[idx++])));
    sort(gt_loops.begin(), gt_loops.end(), cmpTime);

    KBL_IPC<EDGE, VERTEX> kbl(open_loop_problem, cfg);
    SR_SC<EDGE, VERTEX> srsc(open_loop_problem, cfg);

    vector<pair<size_t, EDGE*>> unrevised;   // Ē_l : (candidate_id, edge) since last SR-SC trigger

    double avg_time = 0.0;
    cout << "Starting TACO simulation -> Displaying relative status : " << endl;
    cout << "S = " << s_factor << " | TH = " << cfg.fast_reject_th
         << " | kBL = " << (cfg.use_best_k_buddies ? cfg.k_buddies : 0)
         << " | recovery = " << (cfg.use_recovery ? "on" : "off") << endl;

    for ( size_t candidate_id = 0 ; candidate_id < tot_hypothesis ; ++candidate_id )
    {
        chrono::steady_clock::time_point begin = chrono::steady_clock::now();
        bool consistent = kbl.agreementCheck(gt_loops[candidate_id].second);
        chrono::steady_clock::time_point end = chrono::steady_clock::now();

        bucket[candidate_id] = consistent ? 1 : 0;
        sorted_loops.push_back(gt_loops[candidate_id].second);
        unrevised.push_back(make_pair(candidate_id, gt_loops[candidate_id].second));

        if (cfg.use_recovery && (int)unrevised.size() >= SRSC_BATCH_R)
        {
            vector<EDGE*> batch; for (auto& p : unrevised) batch.push_back(p.second);
            const vector<EDGE*>& inliers = kbl.getMaxConsensusSet();
            vector<EDGE*> promoted = srsc.revise(batch, inliers);

            for (EDGE* e : promoted)
                for (auto& p : unrevised)
                    if (p.second == e && bucket[p.first] == 0)
                    {
                        bucket[p.first] = 1;
                        kbl.addEdgeToCnS(e);
                    }
            unrevised.clear();
        }

        chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
        avg_time += delta_time.count() / 1000000.0;
        printProgress((double)(candidate_id + 1) / (double)tot_hypothesis);
    }
    cout << "\nCompleted!" << endl;

    vector<EDGE*> odom_edges;
    getProblemOdom<EDGE>(open_loop_problem, odom_edges);
    propagateGuess<EDGE, VERTEX>(open_loop_problem, 0, odom_edges.size(), odom_edges);

    OptimizableGraph::EdgeSet eset_gl;
    for ( size_t i = 0 ; i < odom_edges.size(); eset_gl.insert(odom_edges[i++]) )
        odom_edges[i]->setInformation(odom_edges[i]->information() / s_factor);
    for ( size_t i = 0 ; i < gt_loops.size(); ++i )
        if ( bucket[i] == 1 )
            eset_gl.insert(gt_loops[i].second);

    open_loop_problem.initializeOptimization(eset_gl);
    open_loop_problem.optimize(1000);

    int tp = 0, fp = 0, tn = 0, fn = 0;
    for ( size_t counter = 0 ; counter < gt_loops.size() ; ++counter )
        if ( gt_loops[counter].first && bucket[counter] == 1 ) ++tp;
        else if ( gt_loops[counter].first && bucket[counter] == 0 ) ++fn;
        else if ( !gt_loops[counter].first && bucket[counter] == 1 ) ++fp;
        else if ( !gt_loops[counter].first && bucket[counter] == 0 ) ++tn;

    float precision = tp / (float)(tp + fp);
    float recall = tp / (float)(tp + fn);

    const vector<EDGE*> max_consensus_set = kbl.getMaxConsensusSet();
    cout << "Size of MAX consistent set = " << max_consensus_set.size() << endl;
    cout << "Avg Time x test = " << avg_time / tot_hypothesis << " [s]\n";
    cout << "Precision = " << precision << endl;
    cout << "Recall = " << recall << endl;

    ofstream outfile;
    outfile.open(cfg.output.c_str());
    for (size_t it = 0; it < open_loop_problem.vertices().size(); ++it )
    {
        VERTEX* v = dynamic_cast<VERTEX*>(open_loop_problem.vertex(it));
        writeVertex(outfile, v);
    }
    outfile.close();

    string out2 = cfg.output.substr(0, cfg.output.size() - 3) + "PR";
    outfile.open(out2.c_str());
    outfile << precision << " " << recall << endl;
    outfile << avg_time << " " << avg_time / tot_hypothesis << endl;
    outfile.close();

    return;
}

template void simulating_incremental_data_taco<Eigen::Isometry2d, EdgeSE2, VertexSE2>(const Config& cfg,
                                                                                      SparseOptimizer& open_loop_problem,
                                                                                      const vector<EdgeSE2*>& loops);
template void simulating_incremental_data_taco<Eigen::Isometry3d, EdgeSE3, VertexSE3>(const Config& cfg,
                                                                                      SparseOptimizer& open_loop_problem,
                                                                                      const vector<EdgeSE3*>& loops);
