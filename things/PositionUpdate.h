#ifndef _POSITION_UPDATE
#define _POSITION_UPDATE

#include "Solver.h"

class PositionUpdate : public Solver {

public:

  PositionUpdate() {}

  ~PositionUpdate() {}

  PositionUpdate(double timestep): Solver(timestep) {}

  // only change position
  RBState operator () (const RBState &&state, double dt) const {
    RBState ustate = state;
    ustate.Xcom += state.Vcom * dt * timestep;
    return ustate;
  }
};

#endif
