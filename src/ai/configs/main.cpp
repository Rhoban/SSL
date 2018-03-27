#include "ConfigManager.cpp"

int main(int argc, char const *argv[])
{
  srand(time(NULL));
  std::string file = "../src/ai/configs/config.json";
  ConfigManager config;
  config.loadModule(file);
  std::cout << config.get(file, "pi") << std::endl;
  config.set(file, "pi", "3.14159265");
  config.set(file, "random number", std::to_string(rand()));
  config.save(file);
  return 0;
}
