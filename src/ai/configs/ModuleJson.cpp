#include "ModuleJson.h"

ModuleJson::ModuleJson()
{
}

ModuleJson::ModuleJson(std::string file)
{
    loadFromFile(file);
}

ModuleJson::~ModuleJson()
{
}

void ModuleJson::loadFromFile(std::string file)
{
}

std::string ModuleJson::getName()
{
    return this->name;
}

void ModuleJson::setName(std::string newName)
{
    this->name = newName;
}

template <T>
T ModuleJson::get(std::string attribute)
{
    return values.at(attribute).second;
}

template <T>
void ModuleJson::set(std::string attribute, T value)
{
    values.at(attribute).second = value;
}