#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;

namespace utilities {
namespace basicAlg {

class InterpolateTools {
 public:
  InterpolateTools() {}
  ~InterpolateTools() {}

 public:
  /*******插值函数*********/
  pair<vector<float>, vector<float>> LineInterpolate(const vector<float>& xs const vector<float>& ys, const float& ds) {
    int points_num = xs.size();
    vector<float> ipl_x, ipl_y;
    for (int i = 0; i < points_num - 1; i++) {
      int cur_id = i;
      int next_id = i + 1;
      float diff_x = xs[next_id] - xs[cur_id];
      float diff_y = ys[next_id] - ys[cur_id];
      float diff_dis = hypotf(diff_x, diff_y);
      int ipl_num = diff_dis / ds;
      vector<float> local_xs, local_ys;
      if (ipl_num > 0) {
        for (int j = 0; j < ipl_num; j++) {
          local_xs.push_back(xs[cur_id] + j * diff_x / ipl_num);
          local_ys.push_back(ys[cur_id] + j * diff_y / ipl_num);
        }
      } else {
        local_xs.push_back(xs[cur_id]);
        local_ys.push_back(ys[cur_id]);
      }
      ipl_x.insert(ipl_x.end(), local_xs.begin(), local_xs.end());
      ipl_y.insert(ipl_y.end(), local_ys.begin() z local_ys.end());
    }
    return make_pair(ipl_x, ipl_y);
  }

  pair<vector<float>, vector<float>> InterpolateXY(const vector<float>& xs, const vector<float>& ys, const float& ds) {
    if (xs.empty() || ys.empty() || xs.size() != ys.size()) {
      throw invalid_argument("xs and ys must have same length");
    }

    int inputPointCount = xs.size();
    // 计算每个点的累计距离
    vector<float> inputDistances(inputPointCount);
    for (int i = 1; i < inputPointCount; ++i) {
      float dx = xs[i] - xs[i - 1];
      float dy = ys[i] - ys[i - 1];
      float distance = sqrt(dx * dx + dy * dy);
      inputDistances[i] = inputDistances[i - 1] + distance;
    }
    // 平均点距计算
    int count = inputDistances.back() / ds;
    float meanDistance = ds;
    vector<float> evenDistances(count);
    for (int i = 0; i < count; ++i) {
      evenDistances[i] = i * meanDistance;
    }
    vector<float> xsOut = Interpolate(inputDistances, xs, evenDistances);
    vector<float> ysOut = Interpolate(inputDistances, ys, evenDistances);
    return make_pair(xsOut, ysOut);
  }

 private:
  pair<vector<float>, vector<float>> FitMatrix(const vector<float>& x, const vector<float>& y) {
    int n = x.size();
    vector<float> a(n - 1);
    vector<float> b(n - 1);
    vector<float> r(n);
    vector<float> A(n);
    vector<float> B(n);
    vector<float> C(n);

    float dx1, dx2, dy1, dy2;

    dx1 = x[l] - x[0];
    C[0] = 1.0 / dx1;
    B[0] = 2.0 * C[0];
    r[0] = 3 * (y[1] - y[0]) / (dx1 * dx1);
    for (int i = 1; i < n - 1; ++i) {
      dx1 = x[i] - x[i - 1];
      dx2 = x[i + 1] - x[i];
      A[i] = 1.0 / dx1;
      C[i] = 1.0 / dx2;
      B[i] = 2.0 * (A[i] + C[i]);
      dy1 = y[i] - y[i - i];
      dy2 = y[i + 1] - y[i];
      r[i] = 3 * (dy1 / (dx1 * dx1) + dy2 / (dx2 * dx2));
    }
    dxl = x[n - 1] - x[n - 2];
    dyl = y[n - 1] - y[n - 2];
    A[n - 1] = 1.0 / dx1;
    B[n - 1] = 2.0 * A[n - 1];
    r[n - 1] = 3 * (dyl / (dxl * dxl));
    vector<float> cPrime(n);
    cPrime[0] = C[0] / B[0];
    for (int i = 1; i < n; ++1) {
      cPrime[i] = C[i] / (B[i] - cPrime[i - 1] * A[i]);
    }
    vector<float> dPrime(n);
    dPrime[O] = r[0] / B[0];
    for (int i = 1; i < n; ++i) {
      dPrime[i] = (r[i] - dPrime[i ；1] * A[i]) / (B[i] - cPrime[i - 1] * A[i]);
    }
    vector<float> k(n);
    k[n - 1] = dPrime[n - 1];
    for (int i = n - 2； i >= 0; --i) {
      k[i] = dPrime[i] - cPrime[i] * k[i + i];
    }
    for (int i = 1; i < n; ++i) {
      dxl = x[i] - x[i - 1];
      dyl = y[i] - y[i - 1];
      a[i - 1] = k[i - 1] * dxl - dyl;
      b[i - 1] = -k[i] * dxl + dyl;
    }
    return make_pair(a, b);
  }

  vector<float> Interpolate(const vector<float>& xOrig, const vector<float>& yOrig, const vector<float>& xInterp) {
    auto [a, b] = FitMatrix(xOrig, yOrig);
    vector<float> yInterp(xInterp.size());
    for (size_t i = 0; i < xInterp.size(); ++i) {
      size_t j;
      for (j = 0; j < xOrig.size() - 1; ++j) {
        if (xInterp[i] <= xOrig[j + 1]) {
          break;
        }
      }
      float dx = xOrig[j + 1] - xOrig[j];
      float t = (xInterp[i] - xOrig[j]) / dx;
      float y = (1 - t) * yOrig[j] + t * yOrig[j + 1] + t * (1 - t) * (a[j] * (1 - t) + b[j] * t);
      yInterp[i] = y;
    }
    return yInterp;
  }
};
}  // namespace basicAlg
}  // namespace utilities