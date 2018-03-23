#ifndef _CONFIGMODULE_H
#define _CONFIGMODULE_H

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
    ConfigModule(std::string file);
    ConfigModule() = delete;
    ~ConfigModule() = default;

    void loadFromFile(std::string file);
    std::string getName();
    std::string get(std::string attribute);
    void set(std::string attribute, std::string newValue);
    void save();
};

#endif