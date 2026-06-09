#include "metric_eval.hpp"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen)

int main(int argc, char** argv) 
{
  if ( argc < 4 )
    throw std::runtime_error("ERROR, one of the inputs files is missing :\n- g2o_solution\n- gt_solution\n- results_file\n");

  std::string sol_path = argv[1];
  std::string gt_path  = argv[2];
  std::string out_path = argv[3];
  std::string graph_path = argv[4];
    
  std::vector<Eigen::Isometry3d> computed_poses, gt_poses;
  bool is3D = true;
  if ( is3D )
  {
    readSolutionFile3D(computed_poses, sol_path);
    readSolutionFile3D(gt_poses, gt_path);
  }
  else
  {
    readSolutionFile2D(computed_poses, sol_path);
    readSolutionFile2D(gt_poses, gt_path);
  }

  double valid_trajectory = 0.f;
  double rmse_translation = 0.f;
  double local_energy = 0.f;

  g2o::SparseOptimizer optimizer;
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("dl_var", solverProperty));
  
  //Loading the g2o file
  std::ifstream ifs(graph_path.c_str());
  optimizer.load(ifs);
  evaluateSectionRpe(computed_poses, gt_poses, optimizer, 
                     rmse_translation, local_energy, valid_trajectory);
 
  double percentage = 0.2;
  double fps_rpe_costum = 0.f;
  evaluateFPSRpe(computed_poses, gt_poses, 0.1, fps_rpe_costum);

  // Testing the FPS code
  double fps_rpe = 0.f;
  int samples = percentage * gt_poses.size();
  evaluateFPSRpeVanilla(computed_poses, gt_poses, samples, fps_rpe);
  std::cout << "FPS RPE VANILLA = " << fps_rpe << std::endl;

  double global_energy = 0.f;
  evaluateEnergyDeformation(computed_poses, gt_poses, optimizer, global_energy);

  double total_energy = local_energy + global_energy;

  std::cout << "Total Energy: " << total_energy << std::endl;

  std::string results_file_content = "";
  results_file_content += "LOC_E " + std::to_string(local_energy) + "\n";
  results_file_content += "GLO_E " + std::to_string(global_energy) + "\n";
  results_file_content += "FPS_RPE " + std::to_string(fps_rpe) + "\n";
  results_file_content += "VALID_PERC " + std::to_string(valid_trajectory) + "\n";
  results_file_content += "RPE_VAL " + std::to_string(rmse_translation) + "\n";
  
  std::cout << results_file_content;
  std::ofstream out(out_path.c_str());
  out << results_file_content;
  out.close();


  return 0;
}