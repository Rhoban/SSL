#include "Annotations.h"

namespace RhobanSSLAnnotation
{
    Annotations::Annotations()
    {
    }

    Annotations::~Annotations()
    {
        for (auto annotation : annotations) {
            delete annotation;
        }
    }

    std::string Annotations::getClassName() const
    {
        return "Annotations";
    }

    void Annotations::add(Annotation *annotation)
    {
        annotations.push_back(annotation);
    }

    void Annotations::fromJson(const Json::Value & json_value, const std::string & dir_name)
    {
    }

    Json::Value Annotations::toJson() const
    {
        Json::Value json(Json::arrayValue);

        for (auto &annotation : annotations) {
            json.append(annotation->toJson());
        }

        return json;
    }
}
