#include <cmath>
#include "gnc_kernel_impl.hpp"


namespace g2o 
{


void RobustMuTLS::robustify(double e2, Vector3& rho) const 
{ 
  // _delta = chi2 threshold for this kernel
  const double upper_bound = _delta * (1.0 + _mu) / _mu;
  const double lower_bound = _delta * _mu / (1.0 + _mu);

  if (e2 <= lower_bound) 
  {
    rho[0] = e2;
    rho[1] = 1.;
    rho[2] = 0.;
  } 
  else if (e2 > lower_bound && e2 < upper_bound) 
  {  
    rho[0] = 2 * std::sqrt(_delta * _mu * e2 * (_mu + 1.0)) - _mu * (_delta + e2);
    rho[1] = std::sqrt(_delta * _mu * ((_mu + 1.0)) / e2)  - _mu;
    rho[2] = -2.0 * std::sqrt(_delta * _mu * ((_mu + 1.0))) / (e2 * e2);
  }
  else if (e2 >= upper_bound) 
  {   
    rho[0] = _delta;
    rho[1] = 0.;
    rho[2] = 0.;
  }
}

void RobustMuTLS::setMu(number_t mu) 
{
  _mu = mu;
  return;
}

void RobustMuGM::robustify(double e2, Vector3& rho) const 
{
  // _delta = chi2 threshold for this kernel
  const double k = _mu * _delta;
  const double aux = 1. / (k + e2);
  rho[0] = k * e2 * aux;
  rho[1] = k * k * aux * aux;
  rho[2] = -2. * rho[1] * aux;
}

void RobustMuGM::setMu(number_t mu) 
{
  _mu = mu;
  return;
}

void RobustMuDCS::robustify(double e2, Vector3& rho) const 
{ 
  // _delta = chi2 threshold for this kernel
  const double bound = _delta * _mu;
  const double sq_mu = sqrt(_mu);

  rho[0] = e2;
  rho[1] = 1.;
  rho[2] = 0.;
   
  if (e2 > bound) 
  {   
    const number_t dcs_w = 2 * _mu * _delta / (e2 + _mu * _delta);
    const number_t e4 = e2 * e2;
    const number_t chi2 = _delta;
    const number_t chi4 = _delta * _delta;
    
    rho[0] = (_mu * chi4 * e2 + chi2 *e4) / ((e2 + sq_mu * chi2) * (e2 + sq_mu * chi2));
    rho[1] = dcs_w * dcs_w;
    rho[2] = 0.0;
  }
}

void RobustMuDCS::setMu(number_t mu) 
{
  _mu = mu;
  return;
}

}// end namespace g2o