#include <iostream>
#include <set>
#include "ConfigModule.hpp"

class ConfigManager
{
  private:
    static std::set<ConfigModule> modules;

    ConfigManager();
    ~ConfigManager();

  public:

    static ConfigManager& getInstance() {
        static ConfigManager S;
        return S;
    }

    void loadModule(std::string file) {
      ConfigModule * module = new ConfigModule(file);
      this->modules.insert(module);
    };

    std::string get(std::string module, std::string attribute) {
      for(auto i : this->modules) {
        if (i.getName() == module) {
          return i.get(attribute);
        }
      }
    };

    void set(std::string module, std::string attribute, std::string value) {
      for(auto i : this->modules) {
        if (i.getName() == module) {
          i.set(attribute, value);
          break;
        }
      }
    };

    void save(std::string module) {
      for(auto i : this->modules) {
        i.save();
      }
    };
};
