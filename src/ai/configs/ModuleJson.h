#ifndef __CONFIGS__MODULE__H__
#define __CONFIGS__MODULE__H__

#include <iostream>
#include <map>

class ModuleJson
{
private:
  template <T>
  std::map<std::string, T> values;
  std::string name;

public:
  ModuleJson();
  ModuleJson(std::string file);
  ~ModuleJson();

  void loadFromFile(std::string file);

  std::string getName();
  void setName(std::string newName);

  template <T>
  T get(std::string attribute);
  template <T>
  void set(std::string attribute, T value);
};

#endif