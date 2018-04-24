#pragma once

#include "Annotation.h"

namespace RhobanSSLAnnotation
{
    class Circle : public Annotation
    {
    public:
        Circle(double centerX, double centerY, double radius = 0.2, std::string color = "white");

        Json::Value toJson() const;

    protected:
        double centerX, centerY;
        double radius;
    };
}
