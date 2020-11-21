//-------------------------------------------------------
//
//  MyThing.C
//
//  PbaThing for a collection of particles
//  each doing a random walk.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#include "MyThing.h"
#include <cstdlib>
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <assert.h>

using namespace std;

using namespace pba;

Vector MyThing::gravity = Vector(0.0, -0.01, 0.0);

MyThing::MyThing(const std::string nam) :
 PbaThingyDingy (nam),
 box (Box(Vector(-5, -5, -5), Vector(5, 5, 5), 1))
{
    Reset();
    cout << name << " constructed\n";

    // initialize solver for the animation
    PositionUpdate *ps1 = new PositionUpdate(0.5);
    RotationUpdate *rs1 = new RotationUpdate(0.5);
    VelocityUpdate *v = new VelocityUpdate(1.0, gravity);
    OmegaUpdate *o = new OmegaUpdate(1.0);
    PositionUpdate *ps2 = new PositionUpdate(0.5);
    RotationUpdate *rs2 = new RotationUpdate(0.5);
    // now construct the leapfrog
    solver = new LeapFrog(ps1, rs1, v, o, rs2, ps2);

    solver->setForce(Ft);
}

MyThing::~MyThing(){
  delete solver;
}

void MyThing::Init( const std::vector<std::string>& args ) {}

Vector r(0, 0, 0);

void MyThing::Display()
{
   // draw box
   glBegin(GL_QUADS);
   for (size_t i = 0; i < box.numWalls(); ++i) {
     if (i != 2) {
       const Color &ci = box.getWallColor(i);
       glColor3f(ci.red(), ci.green(), ci.blue());
       const std::vector<Vector> &wall = box.getWall(i);
       for (size_t p = 0; p < wall.size(); ++p) {
         glVertex3f(wall[p].X(), wall[p].Y(), wall[p].Z());
       }
     }
   }
   glEnd();

   glPointSize(2.0);
   glEnable(GL_POINT_SMOOTH);
   glEnable(GL_BLEND);
   glBegin(GL_POINTS);
   for( size_t i = 0; i < particles.size(); i++ )
   {
      const Vector& P = particles[i].position;
      const Color& ci = particles[i].color;
      glColor3f( ci.red(), ci.green(), ci.blue() );
      glVertex3f( P.X(), P.Y(), P.Z() );
   }
   glEnd();

   if (r.magnitude() != 0) {
     glColor3f( 0, 0, 0 );
     glBegin(GL_LINES);
     glVertex3f(r.X(), r.Y(), r.Z());
     glVertex3f(state.Xcom.X(), state.Xcom.Y(), state.Xcom.Z());
     glEnd();
  }

  /*
  for (size_t a = 0; a < particles.size(); ++a) {
    const Vector& P = particles[a].position;
    glColor3f( 0, 0, 0 );
    glBegin(GL_LINES);
    glVertex3f(P.X(), P.Y(), P.Z());
    glVertex3f(state.Xcom.X(), state.Xcom.Y(), state.Xcom.Z());
    glEnd();
  }
  */

}

void MyThing::Keyboard( unsigned char key, int x, int y )
{
       PbaThingyDingy::Keyboard(key, x, y);

       if (key == 'G') {
         gravity *= 1.5;
         cout << "Gravity = " << gravity.__str__() << endl;
       }
       if (key == 'g') {
         gravity /= 1.5;
         cout << "Gravity = " << gravity.__str__() << endl;
       }
       // co-efficient of restitution
       if (key == 'S') {
         double cor = box.getCor();
         cor = min(1.0, cor + 0.1);
         box.setCor(cor);
         cout << "cor = " << cor << endl;
       }
       if (key == 's') {
         double cor = box.getCor();
         cor = max(0.0, cor - 0.1);
         box.setCor(cor);
         cout << "cor = " << cor << endl;
       }
       if (key == 'E' || key == 'e')
         exit(0);
}

// will change later
Vector MyThing::ParticleState::getForce() {
  return gravity;
}

// compute MOI from the particle masses and relative positions from the COM
void MyThing::getMOI() {

  // Kronecker Delta Matrix!
  Matrix K;
  K.setIdentity();

  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
        // sum for each element of the matrix
        state.I(i, j) = 0.0;
        // for all particles
        for (size_t a = 0; a < particles.size(); ++a) {
          // mass and rel pos
          double ma = particles[a].mass;
          Vector ra = particles[a].rposition;

          state.I(i, j) += ma * (K(i, j) * (ra * ra) - ra[i] * ra[j]);
        }
    }
  }
}

void MyThing::getTau() {

  state.tau = Vector(0, 0, 0);

  for (size_t a = 0; a < particles.size(); ++a) {
    // ra and Fa
    Vector ra = particles[a].rposition;
    Vector Fa = particles[a].getForce();

    state.tau += (ra ^ Fa);
  }
}

// update ra by the new rotation matrix
void MyThing::getRa() {

  for (size_t a = 0; a < particles.size(); ++a) {
    Vector& ra = particles[a].rposition;
    ra = state.R * r0positions[a];
  }

}

// return the first particle that collides
// particleIndex - the index of the first particle that collides
bool MyThing::firstParticleCollision(double &t,
int &planeIndex, int &particleIndex) {

  // init global values
  t = std::numeric_limits<double>::min();
  particleIndex = planeIndex = -1;

  for (int a = 0; a < int(particles.size()); ++a) {

    Vector ra = particles[a].rposition;

    // some locals
    double lt;
    int lplaneIndex;

    if (box.detectIntersection(state, ra, dt * 0.5, lt, lplaneIndex) && lt > t) {
      // update
      t = lt;
      planeIndex = lplaneIndex;
      particleIndex = a;
    }
  }

  /*
  if (particleIndex != -1)
    particles[particleIndex].color.set(0, 0, 0, 1);
  */

  return particleIndex != -1;
}

// main goal - alter state
void MyThing::handleParticleCollision(const double t,
const int planeIndex, const int particleIndex) {

  // move back by t
  state.Xcom -= (t * state.Vcom);
  state.R = Matrix::getRotationMatrix(state.w, t) * state.R;

  // prepare some data for updating the linear and angular velocties
  // plane normal
  const Vector ns = box.getPlane(planeIndex).getNormal();

  // rposition of the particle 'particleIndex'
  const Vector ra = state.R * r0positions[particleIndex];

  // r = ra + state.Xcom;

  // compute A
  // compute q
  Vector q = state.I.inverse() * (ra ^ ns);

  /*
  double D = (1.0 / M + q * (state.I * q));

  // mighty A
  double A = -(2.0 * (state.Vcom * ns) +
               q * (state.I * state.w) +
               state.w * (state.I * q)) / D;
  */

  double D = 1.0 / M + ns * (q ^ ra);
  double N = -2.0 * ((state.Vcom + (state.w ^ ra)) * ns);
  double A = N / D;

  // set the new reflected velocities
  state.Vcom = state.Vcom.magnitude() * ns;
  // state.Vcom += (A / M) * ns;
  // state.w += A * q;
  state.w = -state.w;

  // update position and rotation matrix again
  state.Xcom += (t * state.Vcom);
  state.R = Matrix::getRotationMatrix(state.w, -t) * state.R;
}

void MyThing::solve()
{

  static int solves = 0.0;
  solves++;

  if (solves == 100) {
    cout << "Hit\n";
    state.tau = Vector(0.0, 0.0, 0.0);
  }

  // position update by half time step
  const PositionUpdate *ps1 = solver->getFirstPositionUpdate();
  state = (*ps1)(std::move(state), dt);

  // collsion detection
  double t;
  int planeIndex, particleIndex;

  if (firstParticleCollision(t, planeIndex, particleIndex)) {
    // handle collsion
    handleParticleCollision(t, planeIndex, particleIndex);
  }

  getRa();
  getMOI();

  // rotation update
  const RotationUpdate *rs1 = solver->getFirstRotationUpdate();
  state = (*rs1)(std::move(state), dt);

  if (firstParticleCollision(t, planeIndex, particleIndex)) {
    // handle collsion
    handleParticleCollision(t, planeIndex, particleIndex);
  }

  getRa();

  // compute new COM velocity with this new force
  const VelocityUpdate *vs = solver->getVelocityUpdate();
  // velocity update
  state = (*vs)(std::move(state), dt);

  // angular velocity update
  const OmegaUpdate *os = solver->getOmegaUpdate();
  state = (*os)(std::move(state), dt);

  // final position update
  const PositionUpdate *ps2 = solver->getSecondPositionUpdate();
  state = (*ps2)(std::move(state), dt);

  // collsion detection, after every position update
  if (firstParticleCollision(t, planeIndex, particleIndex)) {
    // handle collsion
    handleParticleCollision(t, planeIndex, particleIndex);
  }

  getRa();
  getMOI();

  // rotation update
  const RotationUpdate *rs2 = solver->getSecondRotationUpdate();
  state = (*rs2)(std::move(state), dt);

  if (firstParticleCollision(t, planeIndex, particleIndex)) {
    // handle collsion
    handleParticleCollision(t, planeIndex, particleIndex);
  }

  getRa();

  // update torque in the end
  // getTau();

  // update positions of the particles using the new com position values
  for (size_t i = 0; i < particles.size(); ++i)
    particles[i].position = particles[i].rposition + state.Xcom;
}

// return all vertex positions
std::vector<Vector> readFromOBJ(std::string filename) {

  std::vector<Vector> positions;

  std::ifstream f(filename);
  if (!f.is_open())
    return std::vector<Vector>{};

  // read obj
  while (!f.eof()) {

    char line[300];
    f.getline(line, 300);

    std::stringstream s;
    s << line;

    char type = line[0];

    if (type == 'v') {
      double x, y, z;
      s >> type >> x >> y >> z;
      positions.push_back(Vector(x, y, z));
    }
    else if (type == 'f') break;
  }

  return positions;
}

// get com from given positions and masses
Vector getCOM(const std::vector<Vector> &positions,
              const std::vector<double> &masses) {

  Vector com(0, 0, 0);
  double M = 0.0;  // total mass

  // just a simple weighted average
  for (size_t i = 0; i < positions.size(); ++i) {
    com += (masses[i] * positions[i]);
    M += masses[i];
  }

  cout << "mass = " << M << "\n";

  return (1.0 / M) * com;
}

void MyThing::Reset()
{
   // Distribute particles with random positions
   particles.clear();
   r0positions.clear();

   // read point cloud from obj
   // get all point positions
   vector<Vector> positionsobj =
   readFromOBJ("/home/asati/Documents/PBMA/RB/pbalite/models/bunny_superlo_scaled.obj");

   vector<Vector> positions;
   for (size_t i = 0; i < positionsobj.size(); ++i) {
     // if (drand48() < 0.05)
      positions.push_back(positionsobj[i]);
   }

   // apply some model view transformations!
   for (size_t i = 0; i < positions.size(); ++i) {
     // scale and then translate
     Vector& p = positions[i];
     p = 15.0 * (p + Vector(0, -0.15, 0));
   }

   // set the masses for all points to be 1 for now
   vector<double> masses(positions.size(), 1.0);

   for (size_t i = positions.size() / 3; i < positions.size() / 2; ++i)
    masses[i] = 3.0;

   // set the init state of the RB

   // also set the r positions of all particles and then copy
   // difference from COM for all objects
   state.Xcom = getCOM(positions, masses);
   state.Vcom = Vector(0, 0, 0);  // 0 com velocity at the start
   // set the start angular velocity and torque
   state.w = Vector(0, 0, 0);
   state.tau = Vector(14.0, 10.0, 5.0);
   // and the rotation matrix to identity
   state.R.setIdentity();

   vector<Vector> rpositions;
   for (size_t i = 0; i < positions.size(); ++i)
     rpositions.push_back(positions[i] - state.Xcom);

   for (size_t i = 0; i < positions.size(); ++i) {
     // same sized vectors
     Vector p = positions[i];
     Vector r = rpositions[i];
     double m = masses[i];

     particles.push_back(ParticleState(p,
                                       Vector(0, 0, 0),
                                       pba::Color(drand48(), drand48(), drand48(), 1),
                                       m,
                                       r));
     // store ra0positions as well
     r0positions.push_back(r);
   }

   // these states can only be set after we know the relative pos of the parti.
   getMOI();  // MOI

   // maintain a list of forces to be applied to each particle
   Ft = Vector(0, 0, 0);
   M = 0.0;
   // compute net force on the COM
   for (size_t i = 0; i < positions.size(); ++i) {
      Ft += particles[i].getForce();
      M += particles[i].mass;
   }

   // Ft = (1.0 / M) * Ft;

   cout << "COM = " << state.Xcom.__str__() << "\n";
   cout << "# of particles = " << particles.size() << "\n";
}

void MyThing::Usage()
{
   PbaThingyDingy::Usage();
   cout << "=== " << name << " ===\n";
   cout << "e            toggle particle emission on/off\n";
}


pba::PbaThing pba::CreateMyThing(){ return PbaThing( new MyThing() ); }
