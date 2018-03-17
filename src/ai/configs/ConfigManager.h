#ifndef __CONFIGS__CONFIGMANAGER__H__
#define __CONFIGS__CONFIGMANAGER__H__

#include <iostream>
#include <set>
#include "ModuleJson.h"

class ConfigManager
{
  private:
    std::set<ModuleJson> modules;

  public:
    ConfigManager();
    ~ConfigManager();

    void loadModule(std::string file);

    template <T>
    T get(std::string module, std::string attribute);
    template <T>
    void set(std::string module, std::string attribute, T newValue);
    void save(std::string module);
};

#endif