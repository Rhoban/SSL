#ifndef _CONFIGMANAGER_H
#define _CONFIGMANAGER_H

#include <iostream>
#include <vector>
#include "ConfigModule.h"

class ConfigManager
{
private:
  std::vector<ConfigModule *> modules;

public:
  ConfigManager() = default;
  ~ConfigManager() = default;

  void loadModule(std::string file);

  std::string get(std::string odule, std::string attribute);
  void set(std::string module, std::string attribute, std::string newValue);

  void save(std::string module);
};

#endif