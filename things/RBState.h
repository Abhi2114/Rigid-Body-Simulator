#ifndef RBSTATE_H
#define RBSTATE_H

#include "Vector.h"
#include "Matrix.h"

using namespace pba;

// store state for the rigid body
struct RBState {

  Vector Xcom; // com position
  Vector Vcom; // com velocity
  Matrix R;    // rotation matrix
  Matrix I;    // MOI matrix
  Vector w;    // angular velocity
  Vector tau;  // torque
};

#endif
