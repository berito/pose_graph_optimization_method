#include <cmath>
#include "dcs_kernel_impl.hpp"


namespace g2o 
{


void RobustDCSX::robustify(double e2, Vector3& rho) const 
{
    if (e2 < _delta) 
    {
        rho[0] = e2;
        rho[1] = 1.0;
        rho[2] = 0.0;
    } else 
    {
        const number_t dcs_w = 2 * _delta / (e2 + _delta);
        const number_t e4 = e2 * e2;
        const number_t chi2 = _delta;
        const number_t chi4 = _delta * _delta;
        
        rho[0] = (chi4 * e2 + chi2 *e4) / ((e2 + chi2) * (e2 + chi2));
        rho[1] = dcs_w * dcs_w;
        rho[2] = 0.0;
    }
}


}// end namespace g2o