#include "metric_eval.hpp"

int main(int argc, char** argv) 
{
  if ( argc < 4 )
    throw std::runtime_error("ERROR, one of the inputs files is missing :\n- g2o_solution\n- gt_solution\n- results_file\n");

  std::string sol_path = argv[1];
  std::string gt_path  = argv[2];
  std::string out_path = argv[3];
    
  std::vector<Eigen::Isometry3d> computed_poses, gt_poses;
  bool is3D = false;
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

  double ate_rotation     = 0.f; double ate_translation  = 0.f;
  double rpe_rotation     = 0.f; double rpe_translation  = 0.f;
  double rmse_translation = 0.f;


  evaluateRpe(computed_poses, gt_poses, rpe_rotation, rpe_translation);
  evaluateAte(computed_poses, gt_poses, ate_rotation, ate_translation, rmse_translation);
  
  std::string results_file_content = "";
  results_file_content += "ATE_T " + std::to_string(ate_translation) + " ";
  //results_file_content += "ATE_R " + std::to_string(ate_rotation) + "\n";
  results_file_content += "ABS_T " + std::to_string(rmse_translation) + " ";
  //results_file_content += "RPE_R " + std::to_string(rpe_rotation) + "\n";
  results_file_content += "RPE_T " + std::to_string(rpe_translation) + " F\n";

  std::cout << results_file_content;

  std::ofstream out(out_path.c_str());
  out << results_file_content;
  out.close();

  return 0;
}