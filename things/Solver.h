#ifndef _SOLVER_H
#define _SOLVER_H

#include "Vector.h"
#include "RBState.h"

using namespace pba;

// abstract Solver class
class Solver {

protected:
  // multiple of dt
  double timestep;

public:

  Solver(): timestep(0) {}

  virtual ~Solver() {}

  Solver(double timestep): timestep(timestep) {}

  // a Solver doesn't know what type of Solver it is
  // so just use a pure virtual function
  // state = tuple of position and velocity, i.e state = (position, velocity)
  virtual RBState operator () (const RBState &&state, double dt) const = 0;

};

#endif
