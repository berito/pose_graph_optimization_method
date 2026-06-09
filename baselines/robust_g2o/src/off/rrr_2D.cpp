#include "utils.hpp"
#include "rrr/include/RRR.hpp"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

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
	int maxIterations = cfg.maxiters;
  	int inliers = cfg.canonic_inliers;

	int clusteringThreshold = 10;
	int nIter = 4;

	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
	linearSolver->setBlockOrdering(false);
	auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
	g2o::OptimizationAlgorithmGaussNewton *solverGauss = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
	//g2o::OptimizationAlgorithmLevenberg *solverGauss = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
	optimizer.setAlgorithm(solverGauss);
	optimizer.load(input_dataset.c_str());
	odometryInitialization<EdgeSE2, VertexSE2>(optimizer);
	optimizer.push();

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
  	chrono::steady_clock::time_point begin = chrono::steady_clock::now();
	RRR_2D_G2O rrr(clusteringThreshold, nIter);
	rrr.setOptimizer(&optimizer);
	rrr.robustify();
	rrr.removeIncorrectLoops();
  	//optimizer.setVerbose(true);
	optimizer.pop();
	optimizer.vertex(0)->setFixed(true);
	optimizer.initializeOptimization();
  	optimizer.optimize(maxIterations);
  	chrono::steady_clock::time_point end = chrono::steady_clock::now();
  	chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

	std::cout << "Optimtization Concluded!" << std::endl;
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
	float dt = delta_time.count() / 1000000.0;

	std::cout << "Canonic Inliers = " << cfg.canonic_inliers << std::endl;
	std::cout << "Prec = " << precision << std::endl;
	std::cout << "Rec = " << recall << std::endl;
	std::cout << "TP = " << tp << std::endl;
	std::cout << "TN = " << tn << std::endl;
	std::cout << "FP = " << fp << std::endl;
	std::cout << "FN = " << fn << std::endl;


	string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
	outfile.open(output_file_pr.c_str());
	outfile << precision << " " << recall << endl;
	outfile << dt << endl;
	outfile.close();
	
	return 0;
}
