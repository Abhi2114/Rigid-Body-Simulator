#include "Box.h"
#include <cstdlib>
#include <limits>

using namespace pba;

Box::Box(const Vector &llc, const Vector &urc, double cor) {

  // init the coeff of res
  this->cor = cor;

  for (int i = 0; i < 100; ++i) { drand48(); }

  planes.push_back(Plane(llc, Vector(1, 0, 0)));
  planes.push_back(Plane(llc + Vector(0.15, 0.15, 0.15), Vector(1, 0, 0)));
  planes.push_back(Plane(llc + Vector(0.1, 0.1, 0.1), Vector(1, 0, 0)));
  planes.push_back(Plane(llc, Vector(0, 1, 0)));
  planes.push_back(Plane(llc + Vector(0.1, 0.1, 0.1), Vector(0, 1, 0)));
  planes.push_back(Plane(llc, Vector(0, 0, 1)));
  planes.push_back(Plane(urc, Vector(-1, 0, 0)));
  planes.push_back(Plane(urc, Vector(0, -1, 0)));
  planes.push_back(Plane(urc + Vector(-0.1, -0.1, -0.1), Vector(0, -1, 0)));
  planes.push_back(Plane(urc, Vector(0, 0, -1)));

  std::vector<Vector> face;

  face.push_back(llc);
  face.push_back(Vector(llc[0], llc[1], urc[2]));
  face.push_back(Vector(llc[0], urc[1], urc[2]));
  face.push_back(Vector(llc[0], urc[1], llc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();

  face.push_back(llc);
  face.push_back(Vector(llc[0], llc[1], urc[2]));
  face.push_back(Vector(urc[0], llc[1], urc[2]));
  face.push_back(Vector(urc[0], llc[1], llc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();

  face.push_back(llc);
  face.push_back(Vector(llc[0], urc[1], llc[2]));
  face.push_back(Vector(urc[0], urc[1], llc[2]));
  face.push_back(Vector(urc[0], llc[1], llc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();

  face.push_back(urc);
  face.push_back(Vector(urc[0], urc[1], llc[2]));
  face.push_back(Vector(urc[0], llc[1], llc[2]));
  face.push_back(Vector(urc[0], llc[1], urc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();

  face.push_back(urc);
  face.push_back(Vector(urc[0], urc[1], llc[2]));
  face.push_back(Vector(llc[0], urc[1], llc[2]));
  face.push_back(Vector(llc[0], urc[1], urc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();

  face.push_back(urc);
  face.push_back(Vector(urc[0], llc[1], urc[2]));
  face.push_back(Vector(llc[0], llc[1], urc[2]));
  face.push_back(Vector(llc[0], urc[1], urc[2]));
  walls.push_back(face);
  wall_colors.push_back(Color(drand48(), drand48(), drand48(), 0));
  face.clear();
}

// set the index of the closest plane the particle collides with
bool Box::detectIntersection(const RBState& state,
                             const Vector& ra,
                             const double dt,
                             double &t,
                             int &planeIndex) const {

  t = std::numeric_limits<double>::min();  // global location

  // there is no collision so far
  planeIndex = -1;
  // go over all the faces/planes that make up the cube
  for (size_t i = 0; i < planes.size(); ++i) {
    double t_;  // local location

    const Plane &plane = planes[i];
    // check intersection
    if (plane.detectIntersection(state, ra, dt, t_) && t_ > t) {
      planeIndex = i;
      t = t_;
    }
  }

  return planeIndex != -1;
}
