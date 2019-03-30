#include "intersection.h"

bool segment_intersection(const rhoban_geometry::Segment& segment1, const rhoban_geometry::Segment& segment2,
                          rhoban_geometry::Point& intersection)
{
  const rhoban_geometry::Point& A = segment1.A;
  const rhoban_geometry::Point& B = segment1.B;
  const rhoban_geometry::Point& C = segment2.A;
  const rhoban_geometry::Point& D = segment2.B;

  Vector2d AB = B - A;
  Vector2d CD = D - C;

  double u0 = vectorial_product(Vector2d(A), AB);
  double u1 = vectorial_product(Vector2d(C), CD);

  double determinant = vectorial_product(AB, CD);

  if (determinant == 0)
  {
    return false;
  }

  intersection.x = (-CD.getX() * u0 + AB.getX() * u1) / determinant;
  intersection.y = (-CD.getY() * u0 + AB.getY() * u1) / determinant;

  if (scalar_product(AB, intersection - A) >= 0.0 and scalar_product(AB, B - intersection) >= 0.0 and
      scalar_product(CD, intersection - C) >= 0.0 and scalar_product(CD, D - intersection) >= 0.0)
  {
    return true;
  }
  return false;
}
