#include "ConfigModule.h"

ConfigModule::ConfigModule(std::string file)
{
  this->loadFromFile(file);
};

void ConfigModule::loadFromFile(std::string file)
{
  Json::Value json = rhoban_utils::file2Json(file);

  for (std::string s : json.getMemberNames())
  {
    std::string valueVariable = json[s].asString();
    this->values.insert(s, valueVariable);
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
  this->values.at(attribute) = value;
};

void ConfigModule::save()
{
}
