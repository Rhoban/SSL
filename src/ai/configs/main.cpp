#include "ConfigManager.cpp"

int main(int argc, char const *argv[])
{
  ConfigManager config;
  config.loadModule("../src/ai/configs/config.json");
  std::cout << config.get("../src/ai/configs/config.json", "pi") << std::endl;
  return 0;
}
