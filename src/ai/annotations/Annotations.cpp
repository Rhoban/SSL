#include "Annotations.h"

namespace RhobanSSLAnnotation
{
    Annotations::Annotations()
    : json(Json::arrayValue)
    {
    }

    void Annotations::addCircle(double x, double y, double r,
        std::string color, bool dashed)
    {
        Json::Value annotation;

        annotation["type"] = "circle";
        annotation["color"] = color;
        annotation["dashed"] = dashed;

        annotation["x"] = x;
        annotation["y"] = y;
        annotation["r"] = r;

        json.append(annotation);
    }

    void Annotations::addArrow(double x, double y, double toX, double toY,
        std::string color, bool dashed)
    {
        Json::Value annotation;

        annotation["type"] = "arrow";
        annotation["color"] = color;
        annotation["dashed"] = dashed;

        annotation["x"] = x;
        annotation["y"] = y;
        annotation["toX"] = toX;
        annotation["toY"] = toY;

        json.append(annotation);
    }

    void Annotations::addCross(double x, double y,
        std::string color, bool dashed)
    {
        Json::Value annotation;

        annotation["type"] = "cross";
        annotation["color"] = color;
        annotation["dashed"] = dashed;

        annotation["x"] = x;
        annotation["y"] = y;

        json.append(annotation);
    }

    Json::Value Annotations::toJson() const
    {
        return json;
    }

    std::string Annotations::toJsonString()
    {
        Json::FastWriter writer;

        return writer.write(json);
    }
}
