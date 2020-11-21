#include "Plane.h"
#include "Matrix.h"

// signed distance f
double Plane::f(const RBState& state,
                const Vector& ra,
                const double t) const {

  // two components, rotational and linear

  // compute P based on the state
  Matrix R = Matrix::getRotationMatrix(state.w, t);
  Vector r = R * ra;
  Vector l = state.Xcom - t * state.Vcom;
  // get P, finally
  Vector P = r + l;

  // a simple dot product
  return normal * (P - point);
}

Vector Plane::project(const Vector &P) const {
  return Vector();
}

bool Plane::detectIntersection(const RBState& state,
                               const Vector& ra,
                               const double dt,
                               double &t) const {


  // get signed distances for points 'start' and 'end'
  double ss = f(state, ra, dt);
  double se = f(state, ra, 0);

  // check the product
  if (fabs(ss) <= 1e-5) {
    t = dt;
    return true;
  }
  if (fabs(se) <= 1e-5) {
    t = 0.0;
    return true;
  }
  if (ss * se < 0) {
    // iterative root solve
    // start - t1, end = t0
    double t0 = 0;
    double t1 = dt;
    // just to be consistent with the notation in the notes
    double f0 = se;
    double f1 = ss;

    do {
      // guess, a good one
      double t_ = (t0 + t1) / 2.0;
      double f_ = f(state, ra, t_);  // f(t_)

      if (f0 * f_ < 0) {
        f1 = f_;
        t1 = t_;
      }
      else if (f1 * f_ < 0) {
        f0 = f_;
        t0 = t_;
      }
    }
    while (fabs((t0 - t1) / dt) > 1e-5);

    // set tstar before returning
    t = (t0 + t1) / 2.0;

    return true;
  }

  // no collision
  return false;
}
