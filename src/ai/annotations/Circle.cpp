#include "Circle.h"

namespace RhobanSSLAnnotation
{
    Circle::Circle(double centerX, double centerY, double radius, std::string color)
    :
    Annotation(color),
    centerX(centerX),
    centerY(centerY),
    radius(radius)
    {
    }

    Json::Value Circle::toJson() const
    {
        Json::Value json = this->Annotation::toJson();

        json["centerX"] = centerX;
        json["centerY"] = centerY;
        json["radius"] = radius;

        return json;
    }
}
