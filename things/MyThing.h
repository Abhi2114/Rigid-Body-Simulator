//-------------------------------------------------------
//
//  MyThing.h
//
//  PbaThing for a collection of particles
//  each doing a random walk.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "Vector.h"
#include "Matrix.h"
#include "Color.h"
#include "PbaThing.h"

#include "RBState.h"
#include "PositionUpdate.h"
#include "VelocityUpdate.h"
#include "OmegaUpdate.h"
#include "RotationUpdate.h"
#include "LeapFrog.h"

#include "PUWithCollisions.h"

#include "Box.h"

using namespace std;

namespace pba{


class MyThing: public PbaThingyDingy
{
  public:

    // Feel free to customize the name of this thing.
    MyThing(const std::string nam = "Thing!");
   ~MyThing();

    //! Initialization, including GLUT initialization.
    //! Called once at the beginning.  Could be used
    //! to set up things once.
    void Init( const std::vector<std::string>& args );

    ///////////////////////////////////////////////////////////////
    // CASCADING CALLBACK FUNCTIONS
    // The methods below are called as part of a bigger set
    // of similar calls.  Most of the other calls take place
    // in the viewer portion of this project.
    ///////////////////////////////////////////////////////////////

    //! Implements a display event
    //! This is where you code the opengl calls to display
    //! your system.
    void Display();

    //! Implements responses to keyboard events
    //! This is called when you hit a key
    void Keyboard( unsigned char key, int x, int y );

    //! Implements simulator updates during an idle period
    //! This is where the update process is coded
    //! for your dynamics problem.
    void solve();

    //! Implements reseting parameters and/or state
    //! This is called when you hit the 'r' key
    void Reset();

    //! Displays usage information on stdout
    //! If you set up actions with the Keyboard()
    //! callback, you should include a statement
    //! here as to what the keyboard option is.
    void Usage();

  private:

    ////////////////////////////////////////////////
    //
    //      PARTICLE STATE
    //
    // The state of a particle is characterized by
    // (1) particle positions
    // (2) particle velocities
    // (3) particle masses
    // (4) particle colors - useful for display
    class ParticleState
    {
      public:
        ParticleState() :
         position(Vector(0,0,0)),
      	 velocity(Vector(0,0,0)),
      	 color(Color(1,1,1,1)),
      	 mass(1.0),
         rposition(0,0,0){}

        ParticleState(Vector position,
                      Vector velocity,
                      Color color,
                      double mass,
                      Vector rposition) : position(position),
                                          velocity(velocity),
                                          color(color), mass(mass),
                                          rposition(rposition){}

       ~ParticleState(){};

       Vector position;
       Vector velocity;
       Color color;
       double mass;
       // maintain difference vector from the COM
       Vector rposition;

       Vector getForce();
    };

    // This is all of the particles in the system
    std::vector<ParticleState> particles;

    // maintain ra0 for all particles
    std::vector<Vector> r0positions;  // the ra position at time 0

    // could be defined inside RBState as well, by passing in the particles
    // to every function call

    // modify the 3 different states, I, tau and ra for the particles
    // calculate MOI every frame
    void getMOI();
    // calculate torque
    void getTau();
    // update rpositions for all particles
    void getRa();

    // collsion detection and handling!
    bool firstParticleCollision(double &t, int &planeIndex, int &particleIndex);
    void handleParticleCollision(const double t, const int planeIndex,
                                 const int particleIndex);

    // solve nasty A
    double solveA(const Vector& ns, const Vector& q);

    // rigid body state
    RBState state;

    // the total force and total mass do not change
    Vector Ft;
    double M;

    // solver!!
    LeapFrog *solver;

    // Force related parameters
    static Vector gravity;

    // position and velocity of COM of the rigid body
    Vector pcom;
    Vector vcom;

    // Box (cube) to display
    Box box;

    //
    //
    ////////////////////////////////////////////////

};


// This function constructs the MyThing and wraps it in a
// smart pointer called a PbaThing.
// You need not alter this.
pba::PbaThing CreateMyThing();








}
