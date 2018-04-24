#include "Annotation.h"

namespace RhobanSSLAnnotation
{
    Annotation::Annotation(std::string color)
    : color(color)
    {
    }

    std::string Annotation::getClassName() const
    {
        return "Annotation";
    }

    void Annotation::fromJson(const Json::Value & json_value, const std::string & dir_name)
    {
    }

    Json::Value Annotation::toJson() const
    {
        Json::Value json;

        json["color"] = color;

        return json;
    }
}
