#include "rhoban_utils/serialization/factory.h"

using namespace rhoban_utils;

class MyFactory : public Factory<JsonSerializable>{
public:
  /// Written later
  MyFactory();

  void init();
};

class E1 : public JsonSerializable {
public:
  E1() : int_value(0), double_value(1.0){}
  ~E1() {}

  std::string getClassName() const { return "E1";};

  void fromJson(const Json::Value & json_value,
                const std::string & dir_name) override
  {
    (void) dir_name;
    int_value    = rhoban_utils::read<int>   (json_value,"int_value"   );
    double_value = rhoban_utils::read<double>(json_value,"double_value");
  }
  
  Json::Value toJson() const override
  {
    Json::Value v(Json::ValueType::objectValue);
    v["int_value"] = int_value;
    v["double_value"] = double_value;
    return v;
  }
  
  int int_value;
  double double_value;
};

class E2 : public JsonSerializable {
public:
  E2() : s_value("basic_value"){}

  std::string getClassName() const { return "E2";};

  void fromJson(const Json::Value & json_value,
                const std::string & dir_name) override
  {
    (void) dir_name;
    s_value = rhoban_utils::read<std::string>(json_value,"s_value");
  }
  
  Json::Value toJson() const override
  {
    Json::Value v(Json::ValueType::objectValue);
    v["s_value"] = s_value;
    return v;
  }
  
  std::string s_value;
};

class E3 : public JsonSerializable {
public:
  E3() : bolosse("basic_value"){}

  std::string getClassName() const { return "E3";};

  void fromJson(const Json::Value & json_value,
                const std::string & dir_name) override
  {
    (void) dir_name;
    bolosse = rhoban_utils::read<std::string>(json_value,"bolosse");
  }
  
  Json::Value toJson() const override
  {
    Json::Value v(Json::ValueType::objectValue);
    v["bolosse"] = bolosse;
    return v;
  }
  
  std::string bolosse;
};

class Container : public JsonSerializable {
public:
  Container() {}

  std::string getClassName() const { return "Container";};

  void fromJson(const Json::Value & json_value,
                const std::string & dir_name) override
  {
    MyFactory f;
    elements = f.readVector(json_value, "elements", dir_name);
  }

  Json::Value toJson() const override
  {
    Json::Value v(Json::ValueType::objectValue);
    v["elements"] = Json::Value(Json::ValueType::arrayValue);
    for (size_t idx = 0; idx < elements.size(); idx++) {
      v["elements"].append(elements[idx]->toFactoryJson());
    }
    return v;
  }

  std::vector<std::unique_ptr<JsonSerializable>> elements;
};

MyFactory::MyFactory()
  : Factory()
{
  init();
}

void MyFactory::init()
{
  registerBuilder("E1",[](){return std::unique_ptr<JsonSerializable>(new E1());});
  registerBuilder("E2",[](){return std::unique_ptr<JsonSerializable>(new E2());});
  registerBuilder("E3",[](){return std::unique_ptr<JsonSerializable>(new E3());});
  registerBuilder("Container",[](){return std::unique_ptr<JsonSerializable>(new Container());});
}


int main()
{
  MyFactory f;

  // Initializing an element
  Container c;
  c.elements.push_back(f.build("E1"));
  c.elements.push_back(f.build("E2"));
  c.elements.push_back(f.build("E3"));
  c.elements.push_back(f.build("Container"));
  c.saveFile("tmp.json",true);
  
  std::unique_ptr<JsonSerializable> copy = f.buildFromJsonFile("tmp.json");
  
  copy->saveFile("tmp2.json",true);
}
