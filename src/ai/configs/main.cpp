#include "ConfigManager.cpp"

int main(int argc, char const *argv[]) {
  ConfigManager & config = ConfigManager::getInstance();

  config.oui();

  return 0;
}
