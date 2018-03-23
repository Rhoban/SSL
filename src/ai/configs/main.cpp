#include "ConfigManager.cpp"

int main(int argc, char const *argv[])
{
  ConfigManager config;
  config.loadModule("config.json");
  return 0;
}
