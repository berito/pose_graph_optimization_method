#pragma once

#include "dc_ipc/dc_consensus.hpp"
#include "taco/srsc.hpp"

// Loads the correlation-group sidecar "<dataset>.groups": one group per non-empty line, each line a
// space-separated list of vertex-id pairs "a1 b1 a2 b2 ..." identifying the loop edges in that group.
// Missing file -> every loop is its own singleton group (== correlation-blind IPC).
template <class EDGE>
std::vector<std::vector<EDGE*>> loadGroups(const std::string& dataset_path,
                                           const std::vector<EDGE*>& loops);

// lambda: correlation-reward strength (brief eq. term 4); 0.0 → IPC equivalent.
template <class T, class EDGE, class VERTEX>
void simulating_incremental_data_dc(const Config& cfg,
                                    g2o::SparseOptimizer& open_loop_problem,
                                    const std::vector<EDGE*>& loops,
                                    double lambda = 0.0);
