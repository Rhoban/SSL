#pragma once

#include <json/json.h>
#include <vector>
#include <math/vector2d.h>

namespace RhobanSSLAnnotation
{
    class Annotations
    {
    public:
        Annotations();

        void clear();
        void addCircle(double x, double y, double r,
            std::string color = "white", bool dashed = false);

        void addText(
            const std::string & text, double x, double y, 
            std::string color = "white"
        );
        void addText(
            const std::string & text, const rhoban_geometry::Point & point,
            std::string color = "white"
        );
        void addText(
            const std::string & text, const Vector2d & point,
            std::string color = "white"
        );

        void addArrow(
            double x, double y, double toX, double toY,
            std::string color = "white", bool dashed = false
        );
        void addArrow(
                const rhoban_geometry::Point & origin, const rhoban_geometry::Point & end,
                std::string color = "white", bool dashed = false
        );
        void addArrow(
                const Vector2d & origin, const Vector2d & end,
                std::string color = "white", bool dashed = false
        );

        void addCross(double x, double y,
            std::string color = "white", bool dashed = false);

        void addAnnotations(const Annotations& annotations );

        Json::Value toJson() const;
        std::string toJsonString() const;

        void map_positions(
            std::function< 
                rhoban_geometry::Point (
                    const rhoban_geometry::Point & p 
                ) 
            > fct
        );

    protected:
        Json::Value json;
    };
}
