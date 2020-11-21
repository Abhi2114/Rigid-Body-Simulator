#ifndef OMEGA_UPDATE
#define OMEGA_UPDATE

#include "Solver.h"
#include "Vector.h"
#include "Matrix.h"

using namespace pba;

// angular velocity update solver
class OmegaUpdate : public Solver {

public:

  OmegaUpdate() {}

  virtual ~OmegaUpdate() {}

  OmegaUpdate(double timestep) : Solver(timestep){}

  RBState operator () (const RBState &&state, double dt) const {

    RBState ustate = state;
    // update angular velocity
    ustate.w += state.I.inverse() * (dt * state.tau);

    return ustate;
  }
};

#endif
