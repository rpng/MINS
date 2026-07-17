/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MINS_CHI_SQUARE_H
#define MINS_CHI_SQUARE_H

#include <cmath>

namespace mins {

/// Regularized lower incomplete gamma P(a,x). Series below a+1, continued fraction above.
inline double gamma_p(double a, double x) {
  if (x <= 0.0)
    return 0.0;
  double gln = std::lgamma(a);
  if (x < a + 1.0) {
    // series representation
    double ap = a, del = 1.0 / a, sum = del;
    for (int n = 0; n < 500; n++) {
      ap += 1.0;
      del *= x / ap;
      sum += del;
      if (std::fabs(del) < std::fabs(sum) * 1e-16)
        break;
    }
    return sum * std::exp(-x + a * std::log(x) - gln);
  }
  // continued fraction for the upper Q(a,x), then P = 1 - Q (Lentz)
  double b = x + 1.0 - a, c = 1e300, d = 1.0 / b, h = d;
  for (int i = 1; i < 500; i++) {
    double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < 1e-300)
      d = 1e-300;
    c = b + an / c;
    if (std::fabs(c) < 1e-300)
      c = 1e-300;
    d = 1.0 / d;
    double del = d * c;
    h *= del;
    if (std::fabs(del - 1.0) < 1e-16)
      break;
  }
  return 1.0 - std::exp(-x + a * std::log(x) - gln) * h;
}

/// chi-square CDF with k degrees of freedom.
inline double chi_square_cdf(double x, double k) { return gamma_p(0.5 * k, 0.5 * x); }

/// Inverse chi-square CDF: the x where P(chi2_k <= x) = p. Replaces boost::math::quantile(chi_squared(k), p).
/// Safeguarded Newton (bisection fallback) on the CDF -- robust for any k, converges in ~10 iters.
inline double chi_square_quantile(double p, double k) {
  if (p <= 0.0)
    return 0.0;
  // bracket the root by expanding from the mean (= k)
  double lo = 0.0, hi = (k > 0.0 ? k : 1.0);
  while (chi_square_cdf(hi, k) < p)
    hi *= 2.0;
  double x = 0.5 * (lo + hi);
  double lngam = std::lgamma(0.5 * k);
  for (int i = 0; i < 200; i++) {
    double err = chi_square_cdf(x, k) - p;
    (err > 0.0 ? hi : lo) = x;
    // chi-square pdf; Newton step only if it stays inside the bracket
    double pdf = std::exp((0.5 * k - 1.0) * std::log(x) - 0.5 * x - 0.5 * k * M_LN2 - lngam);
    double next = (pdf > 0.0) ? x - err / pdf : 0.5 * (lo + hi);
    if (!(next > lo && next < hi))
      next = 0.5 * (lo + hi);
    if (std::fabs(next - x) < 1e-13 * std::fabs(x))
      return next;
    x = next;
  }
  return x;
}

} // namespace mins

#endif // MINS_CHI_SQUARE_H
