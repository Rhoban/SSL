#pragma once

#include <vector>
#include <rhoban_utils/serialization/json_serializable.h>
#include "Annotation.h"
#include "Circle.h"

namespace RhobanSSLAnnotation
{
    class Annotations : public rhoban_utils::JsonSerializable
    {
    public:
        Annotations();
        virtual ~Annotations();

        void add(Annotation *annotation);

        std::string getClassName() const;

        virtual void fromJson(const Json::Value & json_value,
                              const std::string & dir_name);

        virtual Json::Value toJson() const;

    protected:
        std::vector<Annotation*> annotations;
    };
}
