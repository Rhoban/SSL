#include <iostream>
#include <vector>
#include <memory>
#include "ConfigModule.cpp"

class ConfigManager
{
  private:
    static std::vector <ConfigModule> modules;
    static int hello;

    ConfigManager() = default;

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ConfigManager(ConfigManager&&) = delete;
    ConfigManager& operator=(ConfigManager&&) = delete;

  public:
    static ConfigManager& getInstance();

    void loadModule(std::string file) {
      ConfigModule * module = new ConfigModule(file);
      this->modules.push_back(*module);
    };

    void oui() {
      this->hello = 1;
      std::cout << this->hello << '\n';
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

ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}
