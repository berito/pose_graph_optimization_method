#pragma once

#include "taco/kbl_consensus.hpp"
#include "taco/srsc.hpp"

template <class T, class EDGE, class VERTEX>
void simulating_incremental_data_taco(const Config& cfg,
                                      g2o::SparseOptimizer& open_loop_problem,
                                      const std::vector<EDGE*>& loops);
