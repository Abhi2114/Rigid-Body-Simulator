#ifndef _VELOCITY_UPDATE
#define _VELOCITY_UPDATE

#include "Solver.h"

class VelocityUpdate : public Solver {

private:

  Vector force;

public:

  VelocityUpdate() {}

  ~VelocityUpdate() {}

  VelocityUpdate(double timestep, const Vector &force):
                 Solver(timestep), force(force) {}

  void setForce(const Vector &f) {
    force = f;
  }
               
  // leave everything else unchanged except for the velocity
  RBState operator () (const RBState &&state, double dt) const {
    RBState ustate = state;
    ustate.Vcom += force * dt * timestep;
    return ustate;
  }
};

#endif
