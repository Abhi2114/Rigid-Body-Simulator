#ifndef _LEAPFROG
#define _LEAPFROG

#include "PositionUpdate.h"
#include "VelocityUpdate.h"
#include "OmegaUpdate.h"
#include "RotationUpdate.h"

class LeapFrog : public Solver {

private:

  PositionUpdate *ps1;
  RotationUpdate *rs1;
  VelocityUpdate *vs;
  OmegaUpdate *os;
  RotationUpdate *rs2;
  PositionUpdate *ps2;

public:

  LeapFrog() {}

  ~LeapFrog() {
    delete ps1;
    delete rs1;
    delete vs;
    delete os;
    delete rs2;
    delete ps2;
  }

  LeapFrog(PositionUpdate *p1, RotationUpdate *r1, VelocityUpdate *v,
           OmegaUpdate *o,  RotationUpdate *r2, PositionUpdate *p2):
           ps1(p1), rs1(r1), vs(v), os(o), rs2(r2), ps2(p2) {

  }

  void setForce(const Vector &f) {
    // set force for the velocity update
    vs->setForce(f);
  }

  // useful when you want to apply the solvers individually
  const PositionUpdate* getFirstPositionUpdate() const { return ps1; }
  const RotationUpdate* getFirstRotationUpdate() const { return rs1; }
  const VelocityUpdate* getVelocityUpdate() const { return vs; }
  const OmegaUpdate* getOmegaUpdate() const { return os; }
  const RotationUpdate* getSecondRotationUpdate() const { return rs2; }
  const PositionUpdate* getSecondPositionUpdate() const { return ps2; }

  // state.first = position vector
  // state.second = velocity vector
  RBState operator () (const RBState &&state, double dt) const {

    // lets not worry about this now
    return state;
  }
};

#endif
