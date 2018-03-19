#include <iostream>
#include <map>
#include <fstream>
#include "json.hpp"

using json = nlohmann::json;

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
    std::ifstream json_file("config.json");
    json j;
    json_file >> j;

    for (auto& element : j) {
      std::cout << element << '\n';
    }

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
