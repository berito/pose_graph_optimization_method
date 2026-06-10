// Barron's general & adaptive robust loss (Barron, "A General and Adaptive Robust
// Loss Function", CVPR 2019) as a GTSAM mEstimator + helpers for adapting the shape
// parameter alpha from the residual distribution.
//
// WHY THIS FILE EXISTS: the IPC paper compares against ADAPT [5] ("implemented inside
// g2o" by the author) but the author did NOT release that code, and ADAPT is not a
// standard library kernel (it needs IRLS, not a fixed kernel — see Chebrolu et al.
// RA-L'21 arXiv:2004.14938, AEROS Frontiers'22 arXiv:2110.02018). We therefore
// reimplement it: Barron loss here, IRLS + alpha-adaptation in adapt_{2D,3D}.cpp.
//
// On a (whitened) scalar residual r, shape alpha (<=2), scale c. Normalised so that
// weight(0)=1, loss(0)=0 (full weight for inliers near zero).
#pragma once
#include <gtsam/linear/LossFunctions.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

namespace barron {

// rho(r): Barron loss value.
inline double lossVal(double r, double alpha, double c) {
  const double eps = 1e-6;
  double u2 = (r / c) * (r / c);
  if (std::fabs(alpha - 2.0) < eps) return 0.5 * u2;                  // L2 limit
  if (std::fabs(alpha) < eps)       return std::log(0.5 * u2 + 1.0);  // Cauchy (alpha->0)
  double b = std::fabs(alpha - 2.0);
  return (b / alpha) * (std::pow(u2 / b + 1.0, alpha / 2.0) - 1.0);
}

// w(r) = rho'(r)/r : IRLS weight (GTSAM mEstimator convention). weight(0)=1.
inline double weightVal(double r, double alpha, double c) {
  const double eps = 1e-6;
  double u2 = (r / c) * (r / c);
  if (std::fabs(alpha - 2.0) < eps) return 1.0;                       // L2 limit
  if (std::fabs(alpha) < eps)       return 1.0 / (0.5 * u2 + 1.0);    // Cauchy (alpha->0)
  double b = std::fabs(alpha - 2.0);
  return std::pow(u2 / b + 1.0, alpha / 2.0 - 1.0);
}

// Truncated partition function Z(alpha,c) = int_{-T}^{T} exp(-rho) dx, T=10c
// (trapezoid). Truncation keeps Z finite for alpha<0 where the loss saturates — a
// pragmatic choice (NOTE: verify against the paper at S3/G1).
inline double partition(double alpha, double c) {
  const int N = 1000;
  double T = 10.0 * c, dx = (2.0 * T) / N, Z = 0.0;
  for (int i = 0; i <= N; ++i) {
    double x = -T + i * dx;
    double wq = (i == 0 || i == N) ? 0.5 : 1.0;
    Z += wq * std::exp(-lossVal(x, alpha, c)) * dx;
  }
  return std::max(Z, 1e-12);
}

// Robust residual scale via MAD (median absolute deviation; 1.4826 -> sigma-consistent).
inline double madScale(std::vector<double> res) {
  if (res.empty()) return 1.0;
  std::sort(res.begin(), res.end());
  double med = res[res.size() / 2];
  std::vector<double> dev;
  dev.reserve(res.size());
  for (double r : res) dev.push_back(std::fabs(r - med));
  std::sort(dev.begin(), dev.end());
  double mad = dev[dev.size() / 2];
  return std::max(1.4826 * mad, 1e-3);
}

// Pick alpha from a grid minimising the residuals' negative log-likelihood
// NLL(alpha) = sum_i rho(r_i;alpha,c) + N*log Z(alpha,c)  (Chebrolu-style MLE).
inline double adaptAlpha(const std::vector<double>& res, double c) {
  static const double grid[] = {2.0, 1.0, 0.5, 0.0, -1.0, -2.0, -5.0, -10.0};
  double bestA = 1.0, bestNLL = std::numeric_limits<double>::infinity();
  for (double a : grid) {
    double nll = res.size() * std::log(partition(a, c));
    for (double r : res) nll += lossVal(r, a, c);
    if (nll < bestNLL) { bestNLL = nll; bestA = a; }
  }
  return bestA;
}

// GTSAM mEstimator wrapping Barron's loss for a FIXED (alpha, c). GTSAM performs the
// inner IRLS reweighting using weight(); the outer loop (in the tester) adapts alpha/c.
class Barron : public gtsam::noiseModel::mEstimator::Base {
 public:
  typedef boost::shared_ptr<Barron> shared_ptr;

  Barron(double alpha = 1.0, double c = 1.0, const ReweightScheme reweight = Block)
      : Base(reweight), alpha_(alpha), c_(c) {}
  ~Barron() override {}

  double weight(double distance) const override { return weightVal(distance, alpha_, c_); }
  double loss(double distance) const override { return lossVal(distance, alpha_, c_); }
  void print(const std::string& s) const override {
    std::cout << s << "barron (alpha=" << alpha_ << ", c=" << c_ << ")\n";
  }
  bool equals(const Base& expected, double tol = 1e-8) const override {
    const Barron* p = dynamic_cast<const Barron*>(&expected);
    return p != nullptr && std::fabs(alpha_ - p->alpha_) < tol && std::fabs(c_ - p->c_) < tol;
  }
  static shared_ptr Create(double alpha, double c, const ReweightScheme reweight = Block) {
    return shared_ptr(new Barron(alpha, c, reweight));
  }
  double shape() const { return alpha_; }
  double scale() const { return c_; }

 protected:
  double alpha_, c_;
};

}  // namespace barron
