#include "Annotations.h"

namespace RhobanSSLAnnotation
{
    Annotations::Annotations()
    : json(Json::arrayValue)
    {
    }

    void Annotations::clear(){
       json = Json::Value(Json::arrayValue); 
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

    void Annotations::addAnnotations(const Annotations& annotations ){
        for( unsigned int i=0; i< annotations.json.size(); i++ ){
            json.append( annotations.json[i] ); 
        }
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

    std::string Annotations::toJsonString() const
    {
        Json::FastWriter writer;

        return writer.write(json);
    }
    void Annotations::addArrow(
            const rhoban_geometry::Point & origin, const rhoban_geometry::Point & end,
            std::string color, bool dashed
    ){
        addArrow(
            origin.getX(), origin.getY(), end.getX(), end.getY(),
            color, dashed
        );
    }
    void Annotations::addArrow(
            const Vector2d & origin, const Vector2d & end,
            std::string color, bool dashed
    ){
        addArrow(
            origin.getX(), origin.getY(), end.getX(), end.getY(),
            color, dashed
        );
    }
}
