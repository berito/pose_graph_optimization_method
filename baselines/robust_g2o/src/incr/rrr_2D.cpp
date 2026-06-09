#include "utils.hpp"
#include "rrr/include/RRR.hpp"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

using namespace g2o;
using namespace std;

typedef RRR<G2O_Interface<VertexSE2, EdgeSE2>> RRR_2D_G2O;

G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char **argv)
{

	string cfg_file;
	CommandArgs arg;
	arg.param("cfg", cfg_file, "",
			  "Configuration File(.yaml)");
	arg.parseArgs(argc, argv);

	Config cfg;
  	readConfig(cfg_file, cfg);
    string input_dataset = cfg.dataset;
	int inliers = cfg.canonic_inliers;
	int batch_size = cfg.batch_size;
	int clusteringThreshold = 10;
	int nIter = cfg.maxiters; // 4

	SparseOptimizer optimizer;
	auto linearSolver = make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
	linearSolver->setBlockOrdering(false);
	auto blockSolver = make_unique<BlockSolverX>(move(linearSolver));
	OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton(move(blockSolver));
	optimizer.setAlgorithm(solver);
	optimizer.load(input_dataset.c_str());
	odometryInitialization<EdgeSE2, VertexSE2>(optimizer);

	// GETTING INLIER AND OUTLIER LABELS + SETTING EXPERIMENTS AS IF IT WAS INCREMENTAL EXPERIMENT
	OptimizableGraph::EdgeContainer loop_edges, odom_edges;
	getLoopEdges<EdgeSE2, VertexSE2>(optimizer, loop_edges);
	getOdometryEdges<EdgeSE2, VertexSE2>(optimizer, odom_edges);
	vector<pair<bool, OptimizableGraph::Edge*>> loops_w_label;
	for (size_t idx = 0 ; idx < cfg.canonic_inliers; loops_w_label.push_back(make_pair(true, loop_edges[idx++])));
	for (size_t idx = cfg.canonic_inliers ; idx < loop_edges.size(); loops_w_label.push_back(make_pair(false, loop_edges[idx++])));
	sort(loops_w_label.begin(), loops_w_label.end(), cmpTime);

	vector<string> gt_loops;
	for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  	{
    	EdgeSE2* edge_odom = dynamic_cast<EdgeSE2*>(*it_e);
		/* Odom edge loop closure */ 
		if ( edge_odom != nullptr && abs(edge_odom->vertices()[1]->id() - edge_odom->vertices()[0]->id()) > 1  )
		{
			string key = to_string(edge_odom->vertices()[0]->id()) + "-" + to_string(edge_odom->vertices()[1]->id());
			gt_loops.push_back(key);
		}
  	}

	/* Initialized RRR with the parameters defined */
  	RRR_2D_G2O rrr(clusteringThreshold, nIter);
	rrr.setIncrOptimizer(&optimizer);

	// INCREMENTAL EXPERIMENT
	int last_odom_idx = 0;
	double avg_time = 0.0; int n_optimization = 0;
	for ( size_t e_it = 0 ; e_it < loops_w_label.size() ; ++e_it )
	{
		int max_vid = 0;
		vector<HyperGraph::Edge *> test_loops, test_odom;
		for (size_t batch_it = 0; batch_it < batch_size && e_it + batch_it < loops_w_label.size() ; ++batch_it)
		{
			OptimizableGraph::Edge* el = loops_w_label[e_it + batch_it].second;
			int v0 = el->vertices()[0]->id();
			int v1 = el->vertices()[1]->id();
			max_vid = max(max_vid, v0);
			max_vid = max(max_vid, v1);
			test_loops.push_back(el);
		}
		e_it += batch_size - 1;
		for (size_t eo_it = last_odom_idx ; eo_it < max_vid ; test_odom.push_back(odom_edges[eo_it++]));
		last_odom_idx = max_vid;

		// Optimizing the problem until the last vertex
		chrono::steady_clock::time_point begin = chrono::steady_clock::now();
		rrr.incrRobustify(test_odom, test_loops);
		chrono::steady_clock::time_point end = chrono::steady_clock::now();
		chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
		avg_time += delta_time.count() / 1000000.0;
		++n_optimization;

		printProgress((double)(e_it + 1) / (double)loops_w_label.size());
	}
	cout << endl;
	
	rrr.removeIncorrectLoops();
	optimizer.vertex(0)->setFixed(true);
	optimizer.initializeOptimization();
  	optimizer.optimize(100);

	cout << "Optimtization Concluded!" << endl;
	ofstream outfile;
	string output_file_trj = cfg.output;
	outfile.open(output_file_trj.c_str()); 
	for (size_t it = 0; it < optimizer.vertices().size(); ++it )
	{
		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer.vertex(it));
		writeVertex(outfile, v);
	}
	outfile.close();

	vector<string> est_loops;		
	for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  	{
    	EdgeSE2* edge_odom = dynamic_cast<EdgeSE2*>(*it_e);
		if ( edge_odom != nullptr && abs(edge_odom->vertices()[1]->id() - edge_odom->vertices()[0]->id()) > 1  )
		{
			string key = to_string(edge_odom->vertices()[0]->id()) + "-" + to_string(edge_odom->vertices()[1]->id());
			est_loops.push_back(key);
		}
  	}

	int tp  = 0; int tn = 0; int fp  = 0; int fn = 0; 
	for ( size_t idx = 0; idx < inliers; ++idx)
	{
		auto it = find(est_loops.begin(), est_loops.end(), gt_loops[idx]);
		if ( it != est_loops.end() ) ++tp;
		else ++fn;
	}

	for ( size_t idx = inliers; idx < gt_loops.size(); ++idx)
	{
		auto it = find(est_loops.begin(), est_loops.end(), gt_loops[idx]);
		if ( it == est_loops.end() ) ++tn;
		else ++fp;
	}

	float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  	float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 

	cout << "Total Time of Completion " << avg_time << " [s]" << endl;
  	cout << "Avg time per batch " << avg_time / n_optimization << " [s]" << endl;
	cout << "Canonic Inliers = " << cfg.canonic_inliers << endl;
	cout << "Prec = " << precision << endl;
	cout << "Rec = " << recall << endl;
	cout << "TP = " << tp << endl;
	cout << "TN = " << tn << endl;
	cout << "FP = " << fp << endl;
	cout << "FN = " << fn << endl;


	string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
	outfile.open(output_file_pr.c_str());
	outfile << precision << " " << recall << endl;
  	outfile << avg_time << " " << avg_time / n_optimization << endl;
	outfile.close();
	
	return 0;
}
