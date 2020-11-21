#ifndef _BOX
#define _BOX

#include <vector>
#include "Plane.h"

class Box {

private:

    std::vector<Plane> planes;

    // opengl walls (squares), only for display purposes
    std::vector<std::vector<Vector>> walls;
    std::vector<Color> wall_colors;

    // stickiness of the box
    double cor;

public:

  Box() {}

  Box(const Vector &llc, const Vector &urc, double cor=1);
  ~Box() {};

  const std::vector<Vector>& getWall(int index) const { return walls[index]; }
  const Color& getWallColor(int index) const { return wall_colors[index]; }
  const Plane& getPlane(int index) const { return planes[index]; }

  size_t numWalls() const { return walls.size(); }
  size_t numPlanes() const { return planes.size(); }
  double getCor() const { return cor; }
  void setCor(double cor) { this->cor = cor; }

  // t and planeIndex to be used only if this returns true
  bool detectIntersection(const RBState& state,
                          const Vector& ra,
                          const double dt,
                          double &t,
                          int &planeIndex) const;

};

#endif
