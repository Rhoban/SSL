#include "ConfigManager.h"

void ConfigManager::loadModule(std::string file)
{
  ConfigModule module = ConfigModule(file);
  this->modules.push_back(module);
};

std::string ConfigManager::get(std::string module, std::string attribute)
{
  for (ConfigModule m : this->modules)
  {
    if (m.getName() == module)
    {
      return m.get(attribute);
    }
  }
};

void ConfigManager::set(std::string module, std::string attribute, std::string newValue)
{
  for (auto i : this->modules)
  {
    if (i.getName() == module)
    {
      i.set(attribute, newValue);
      break;
    }
  }
};

void ConfigManager::save(std::string module)
{
  for (auto i : this->modules)
  {
    i.save();
  }
}
