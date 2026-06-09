#pragma once

#include "g2o/core/g2o_core_api.h"
#include "g2o/core/robust_kernel.h"

namespace g2o {

class G2O_CORE_API RobustDCSX : public RobustKernel {
  public:
    void robustify(number_t error, Vector3& rho) const;
};



}  // end namespace g2o