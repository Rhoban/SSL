#pragma once

#include <json/json.h>
#include <vector>

namespace RhobanSSLAnnotation
{
    class Annotations
    {
    public:
        Annotations();

        void addCircle(double x, double y, double r,
            std::string color = "white", bool dashed = false);

        void addArrow(double x, double y, double toX, double toY,
            std::string color = "white", bool dashed = false);

        void addCross(double x, double y,
            std::string color = "white", bool dashed = false);

        Json::Value toJson() const;
        std::string toJsonString();

    protected:
        Json::Value json;
    };
}
