#include "Annotations.h"

namespace RhobanSSLAnnotation
{

    void Annotations::map_positions(
        std::function< 
            rhoban_geometry::Point (
                const rhoban_geometry::Point & p 
            ) 
        > fct
    ) {
        for( unsigned int i=0; i< json.size(); i++ ){
            Json::Value & annotation = json[i];
            std::string type = annotation["type"].asString();
            rhoban_geometry::Point point;
            if( type == "arrow" ){
                point = fct(
                    rhoban_geometry::Point( annotation["x"].asDouble(), annotation["y"].asDouble() )
                );
                annotation["x"] = point.getX();
                annotation["y"] = point.getY();
                point = fct(
                    rhoban_geometry::Point( annotation["toX"].asDouble(), annotation["toY"].asDouble() ) 
                );
                annotation["toX"] = point.getX();
                annotation["toY"] = point.getY();
            }else if( type == "text" or type == "cross" or type == "circle" ){
                point = fct(
                    rhoban_geometry::Point( annotation["x"].asDouble(), annotation["y"].asDouble() )
                );
                annotation["x"] = point.getX();
                annotation["y"] = point.getY();
            }else{
                std::cerr << "Unknown annotation type : " << type 
                    << "." << std::endl;
                assert(false);
            }
        }
    }

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

    void Annotations::addText(
        const std::string & text, double x, double y, 
        std::string color
    ){
        Json::Value annotation;

        annotation["type"] = "text";
        annotation["text"] = text;
        annotation["color"] = color;
        annotation["dashed"] = false;

        annotation["x"] = x;
        annotation["y"] = y;

        json.append(annotation);
    }
    void Annotations::addText(
        const std::string & text, const rhoban_geometry::Point & point,
        std::string color
    ){
        addText(
            text, point.getX(), point.getY(), color
        );
    };
    void Annotations::addText(
        const std::string & text, const Vector2d & point,
        std::string color
    ){
        addText(
            text, point.getX(), point.getY(), color
        );
    }

}
