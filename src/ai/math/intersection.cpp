#include "intersection.h"

bool segmentIntersection(const rhoban_geometry::Segment& segment1, const rhoban_geometry::Segment& segment2,
                         rhoban_geometry::Point& intersection)
{
  const rhoban_geometry::Point& A = segment1.A;
  const rhoban_geometry::Point& B = segment1.B;
  const rhoban_geometry::Point& C = segment2.A;
  const rhoban_geometry::Point& D = segment2.B;

  Vector2d AB = B - A;
  Vector2d CD = D - C;

  double u0 = vectorialProduct(Vector2d(A), AB);
  double u1 = vectorialProduct(Vector2d(C), CD);

  double determinant = vectorialProduct(AB, CD);

  if (determinant == 0)
  {
    return false;
  }

  intersection.x = (-CD.getX() * u0 + AB.getX() * u1) / determinant;
  intersection.y = (-CD.getY() * u0 + AB.getY() * u1) / determinant;

  if (scalarProduct(AB, intersection - A) >= 0.0 and scalarProduct(AB, B - intersection) >= 0.0 and
      scalarProduct(CD, intersection - C) >= 0.0 and scalarProduct(CD, D - intersection) >= 0.0)
  {
    return true;
  }
  return false;
}
