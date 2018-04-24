#pragma once

#include <rhoban_utils/serialization/json_serializable.h>

namespace RhobanSSLAnnotation
{
    class Annotation : public rhoban_utils::JsonSerializable
    {
    public:
        Annotation(std::string color = "white");

        std::string getClassName() const;

        void fromJson(const Json::Value & json_value, const std::string & dir_name);

        Json::Value toJson() const;

    protected:
        std::string color;
    };
}
