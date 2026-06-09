#pragma once

#include "g2o/core/g2o_core_api.h"
#include "g2o/core/robust_kernel.h"

namespace g2o {

class G2O_CORE_API RobustMuTLS : public RobustKernel {
  public:
    void robustify(number_t error, Vector3& rho) const;
    void setMu(number_t mu);
    number_t getMu() const { return _mu; }

  protected:
    number_t _mu;
};

class G2O_CORE_API RobustMuGM : public RobustKernel 
{
  public:
    void robustify(number_t error, Vector3& rho) const;
    void setMu(number_t mu);
    number_t getMu() const { return _mu; }

  protected:
    number_t _mu;
};

class G2O_CORE_API RobustMuDCS : public RobustKernel 
{
  public:
    void robustify(number_t error, Vector3& rho) const;
    void setMu(number_t mu);
    number_t getMu() const { return _mu; }

  protected:
    number_t _mu;
};


}  // end namespace g2o