#include <iostream>
#include <map>
#include <fstream>
#include "rhoban_utils/serialization/json_serializable.h"

class ConfigModule
{
private:
  std::map<std::string, std::string> values;
  std::string name;

public:
  ConfigModule(std::string file) {
    this->loadFromFile(file);
  };

  ~ConfigModule() {};

  void loadFromFile(std::string file) {
    Json::Value json = rhoban_utils::file2Json(file);

    std::cout << "oui" << std::endl;

    this->name = file;
  };

  std::string getName() {
    return this->name;
  };

  std::string get(std::string attribute) {
    return values.at(attribute);
  };

  void set(std::string attribute, std::string value) {
    values.at(attribute) = value;
  };

  void save () {
  }
};
