#ifndef ROTATION_UPDATE
#define ROTATION_UPDATE

#include "Solver.h"
#include "Vector.h"
#include "Matrix.h"

using namespace pba;

class RotationUpdate : public Solver {

public:

  RotationUpdate() {}

  virtual ~RotationUpdate() {}

  RotationUpdate(double timestep) : Solver(timestep) {}

  RBState operator () (const RBState &&state, double dt) const {

    RBState ustate = state;
    // get new rotation matrix and multiply
    Matrix E = Matrix::getRotationMatrix(state.w, -dt);
    // multiply with previous
    ustate.R = E * ustate.R;

    return ustate;
  }
};

#endif
