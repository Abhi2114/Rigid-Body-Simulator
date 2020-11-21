#ifndef _PLANE
#define _PLANE

#include "Vector.h"
#include "Color.h"
#include "RBState.h"

using namespace pba;

class Plane {

private:

    Vector point;
    Vector normal;

public:

  Plane(const Vector& point, const Vector& normal): point(point), normal(normal) {}

  const Vector& getPoint() const { return point; }
  const Vector& getNormal() const { return normal; }

  // signed distance of point from the plane
  double f(const RBState& state, const Vector& ra, const double t) const;

  Vector project(const Vector &P) const;

  // also include the timestep value for iterative root solve
  // t - time at which f(t) = 0
  // dt - timestep
  bool detectIntersection(const RBState& state,
                          const Vector& ra,
                          const double dt,
                          double &t) const;
};

#endif
