#include "utils.hpp"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv) 
{

  // Command line parsing
  int maxIterations = 100;
  string outputFilename;
  string open_loop_trajectory;
  string seeded_trajectory;
  string inputFilename;
  CommandArgs arg;
  
  arg.param("st", seeded_trajectory, "",
            "Trajectory with few loops used for seeding");
  arg.param("o", outputFilename, "res.txt",
            "G2o file with ehnanced loops");
  arg.parseArgs(argc, argv);

  typedef Eigen::Isometry3d PoseType;
  typedef EdgeSE3 EdgeType;
  typedef VertexSE3 VertexType;
  
  vector<PoseType> init_poses;
  vector<VertexType*> v_poses;

  std::cout << "Reading file: " << seeded_trajectory << std::endl;


  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  setProblem<PoseType, EdgeType, VertexType>(seeded_trajectory, optimizer, init_poses, v_poses);
  optimizer.vertex(0)->setFixed(true);
  std::cout << "Starting optimization : " << std::endl;
  optimizer.initializeOptimization();
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.setVerbose(true);
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  // Getting loop closure edges and using them as seed to create 
  // new artificial loop edges
  std::vector<std::pair<int, int>> sloop_edges; // seeds
  for ( auto it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it )
  {
    auto edge = dynamic_cast<EdgeType*>(*it);
    if ( edge == nullptr ) continue;

    int id1 = edge->vertices()[0]->id();
    int id2 = edge->vertices()[1]->id();

    if ( id2 - id1 == 1 ) continue;

    sloop_edges.push_back(std::make_pair(id1, id2));
  }

  // Creating new loop edges
  int n_extra_id1 = 5;
  int n_extra_id2 = 4;
  
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
  auto edge_tmp = dynamic_cast<EdgeType*>(*optimizer.edges().begin());
  info = edge_tmp->information();
  std::cout << "Optimtization Concluded!" << std::endl;
  
  /**/
  for (size_t ide = 0; ide < sloop_edges.size(); ++ide)
  {
    int id1 = sloop_edges[ide].first;
    int id2 = sloop_edges[ide].second;
    for (size_t count_id1 = 0; count_id1 < n_extra_id1; ++count_id1)
    {
        int id1_ext = id1 - count_id1;

        for (size_t count_id2 = 0; count_id2 < n_extra_id2; ++count_id2)
        {
            // Only happens once
            if ( count_id1 == 0 && count_id2 == 0 ) continue;

            int id2_ext = id2 + count_id2;
            auto v1 =  dynamic_cast<VertexType*>(optimizer.vertex(id1_ext));
            auto v2 =  dynamic_cast<VertexType*>(optimizer.vertex(id2_ext));

            // Checking if the vertices exist
            if ( v1 == nullptr || v2 == nullptr ) continue;

            // i >> j
            Eigen::Isometry3d wposei = v1->estimate();
            Eigen::Isometry3d wposej = v2->estimate();

            // Creating the edge
            EdgeType* edge = new EdgeType();
            edge->vertices()[0] = v1;
            edge->vertices()[1] = v2;
            edge->setMeasurement(wposei.inverse() * wposej);
            edge->setInformation(info);
            optimizer.addEdge(edge);
        }
    }
  }

  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexType* v = dynamic_cast<VertexType*>(optimizer.vertex(it));
      v->setEstimate(init_poses[v->id()]);
  }
  optimizer.save("ehanced_gt.g2o");

  cout << "Second optimization finished!" << endl;
  /**

  ofstream outfile;
  outfile.open(outputFilename.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexType* v = dynamic_cast<VertexType*>(optimizer.vertex(it));
      writeVertex(outfile, v);
  }
  outfile.close();
  /**/

  return 0;
}
