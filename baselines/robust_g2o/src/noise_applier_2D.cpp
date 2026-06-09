#include "utils.hpp"
#include <random>

using namespace std;
using namespace g2o;

// we use the 2D
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv) 
{

  // Command line parsing
  string inputFilename;
  CommandArgs arg;
  
  arg.param("input-graph", inputFilename, "",
            "Trajectory with few loops used for seeding");
  arg.parseArgs(argc, argv);
  
  // Loading the g2o file
  SparseOptimizer optimizer;
  ifstream ifs(inputFilename.c_str());
  if (!ifs) 
  {
      cerr << "unable to open " << inputFilename << endl;
      return 0;
  }
  optimizer.load(ifs);

  auto it_e0 = optimizer.edges().begin();
  EdgeSE2* e0 = dynamic_cast<EdgeSE2*>(*it_e0);
  //Eigen::Matrix3d cov_o = e0->information().inverse();
  Eigen::Matrix3d cov_o = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d cov_l = Eigen::Matrix3d::Zero();
  std::cout << "Information matrix of first edge: \n" << cov_o << std::endl;

  double sigmao_x = 0.0; double sigmao_y = 0.0; double sigmao_theta = 0.0;
  //double sigmal_x = 0.09; double sigmal_y = 0.08; double sigmal_theta = 0.01;
  double sigmal_x = 0.02; double sigmal_y = 0.005; double sigmal_theta = 0.0005;
  std::default_random_engine generator;
  std::normal_distribution<double> dist_x(0.0, sigmal_x);
  std::normal_distribution<double> dist_y(0.0, sigmal_y);
  std::normal_distribution<double> dist_theta(0.0, sigmal_theta);
  cov_o(0,0) = sigmal_x; cov_o(1,1) = sigmal_y; cov_o(2,2) = sigmal_theta;
  cov_l(0,0) = sigmal_x; cov_l(1,1) = sigmal_y; cov_l(2,2) = sigmal_theta;
  std::cout << "New covariance matrix for odometry edges: \n" << cov_o << std::endl;
  std::cout << "New covariance matrix for loop edges: \n" << cov_l << std::endl;


  for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge_odom = dynamic_cast<EdgeSE2*>(*it_e);

        if (edge_odom == nullptr ) continue;

        auto v_fn = dynamic_cast<VertexSE2*>(edge_odom->vertices()[1]);
        auto v_st = dynamic_cast<VertexSE2*>(edge_odom->vertices()[0]);

        int id_st = v_st->id();
        int id_fn = v_fn->id();

        if ( id_fn - id_st > 1 ) 
        {
          //edge_odom->setInformation(cov_l.inverse());
          continue;
        }

        //auto est = v_st->estimate() * edge_odom->measurement();
        auto true_motion = edge_odom->measurement();
        SE2 noise(dist_x(generator), dist_y(generator), dist_theta(generator));
        auto noisy_motion = true_motion  * noise;
        //est = est * noise;
        auto est = v_st->estimate() * noisy_motion; 
        v_fn->setEstimate(est);
        edge_odom->setMeasurement(noisy_motion);
        //edge_odom->setInformation(cov_o.inverse());
        Eigen::Matrix3d cov_e = edge_odom->information().inverse();
        double sigmax = cov_e(0,0);
        double sigmay = cov_e(1,1);
        double sigmat = cov_e(2,2);
        //sigmao_x = sigmao_x > sigmax ? sigmao_x : sigmax;
        //sigmao_y = sigmao_y > sigmay ? sigmao_y : sigmay;
        //sigmao_theta = sigmao_theta > sigmat ? sigmao_theta : sigmat; 
        edge_odom->setInformation(cov_o.inverse());

    }

  std::cout << "Final standard deviations: \n";
  std::cout << "sigmao_x: " << sigmao_x << ", sigmao_y: " << sigmao_y << ", sigmao_theta: " << sigmao_theta << std::endl;
  
  string outputFilename = "noise_graph.g2o";
  optimizer.save(outputFilename.c_str());

  return 0;
}
