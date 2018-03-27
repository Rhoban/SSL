#include "ConfigModule.h"
#include <typeinfo>

ConfigModule::ConfigModule(std::string file)
{
  this->loadFromFile(file);
};

void ConfigModule::loadFromFile(std::string file)
{
  Json::Value json = rhoban_utils::file2Json(file);

  for (std::string s : json.getMemberNames())
  {
    //Json::arrayValue to handle
    std::string valueVariable = json.get(s, "unknown").asString();
    this->values.insert({s, valueVariable});
  }

  this->name = file;
};

std::string ConfigModule::getName()
{
  return this->name;
};

std::string ConfigModule::get(std::string attribute)
{
  return this->values.at(attribute);
};

void ConfigModule::set(std::string attribute, std::string value)
{
  if (this->values.find(attribute) != this->values.end())
  {
    this->values.at(attribute) = value;
  }
  else
  {
    this->values.insert({attribute, value});
  }
};

void ConfigModule::save()
{
  Json::Value root;
  for (std::pair<std::string, std::string> s : this->values)
  {
    root[s.first] = s.second;
  }
  std::ofstream file;
  file.open(this->name);
  Json::StyledWriter styledWriter;
  file << styledWriter.write(root);
  file.close();
}
