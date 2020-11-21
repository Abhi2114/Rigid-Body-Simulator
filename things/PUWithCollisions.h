#ifndef _PU_COLLISIONS
#define _PU_COLLISIONS

#include "PositionUpdate.h"
#include "Box.h"

class PUWithCollisions : public PositionUpdate {

private:

    Box box;

    void handleCollision(RBState &pstate, RBState &nstate, double dt) const {

    }

public:

    PUWithCollisions() {}

    ~PUWithCollisions() {}

    PUWithCollisions(double timestep, Box &box): PositionUpdate(timestep), box(box) {}

    RBState operator () (const RBState &&state, double dt) const {

      // pstate -> prv state
      RBState pstate = state;

      RBState nstate;
      nstate.Xcom = pstate.Xcom + pstate.Vcom * timestep * dt;
      // handle collisions!
      handleCollision(pstate, nstate, dt);

      return nstate;
    }
};

#endif
