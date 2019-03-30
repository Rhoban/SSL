#include <fenv.h>
#include <tclap/CmdLine.h>
#include <com/ai_commander_real.h>
#include <com/ai_commander_simulation.h>
#include <QApplication>
#include "mainwindow.h"
#include "API.h"
#include "client_config.h"

#define TEAM_NAME "AMC"
#define ZONE_NAME "all"
#define CONFIG_PATH "./src/ai/config.json"

int main(int argc, char* argv[])
{
  // Command line parsing
  TCLAP::CmdLine cmd("Rhoban SSL AI", ' ', "0.0");
  TCLAP::SwitchArg simulation("s", "simulation", "Simulation mode", cmd, false);
  TCLAP::SwitchArg yellow("y", "yellow", "If set we are yellow otherwise we are blue.", cmd, false);

  TCLAP::ValueArg<std::string> team_name(
      "t",     // short argument name  (with one character)
      "team",  // long argument name
      "The referee team name. The default value is '" TEAM_NAME "'. "
      "The team name is used to detect from the referee the team color. "
      "If referee is not used, or there is no referee or the team name "
      "provided by the referee doesn't match the given team name, then, "
      "we use the default color provided by the yellow argument.",  // long Description of the argument
      false,                                                        // Flag is not required
      TEAM_NAME,                                                    // Default value
      "string",                                                     // short description of the expected value.
      cmd);

  TCLAP::ValueArg<std::string> zone_name("z",     // short argument name  (with one character)
                                         "zone",  // long argument name
                                         "Define A zone to watch. All vision event outside the zone are ignored."
                                         "It is used to work with another team in the same field."
                                         "Avalaible values are : 'positive', 'negative' and 'all'."
                                         "The default value is '" ZONE_NAME "'. ",
                                         false,      // Flag is not required
                                         ZONE_NAME,  // Default value
                                         "string",   // short description of the expected value.
                                         cmd);

  TCLAP::ValueArg<std::string> config_path("c",       // short argument name  (with one character)
                                           "config",  // long argument name
                                           "The config path to the json configuration of AI. The default value is "
                                           "'" CONFIG_PATH "'. ",
                                           false,        // Flag is not required
                                           CONFIG_PATH,  // Default value
                                           "string",     // short description of the expected value.
                                           cmd);

  TCLAP::ValueArg<std::string> addr("a",        // short argument name  (with one character)
                                    "address",  // long argument name
                                    "Vision client address",
                                    false,               // Flag is not required
                                    SSL_VISION_ADDRESS,  // Default value
                                    "string",            // short description of the expected value.
                                    cmd);

  TCLAP::ValueArg<std::string> port("p",     // short argument name  (with one character)
                                    "port",  // long argument name
                                    "Vision client port",
                                    false,            // Flag is not required
                                    SSL_VISION_PORT,  // Default value
                                    "string",         // short description of the expected value.
                                    cmd);

  TCLAP::ValueArg<std::string> sim_port("u",         // short argument name  (with one character)
                                        "sim_port",  // long argument name
                                        "Vision client simulator port",
                                        false,                       // Flag is not required
                                        SSL_SIMULATION_VISION_PORT,  // Default value
                                        "string",                    // short description of the expected value.
                                        cmd);

  cmd.parse(argc, argv);

  // Instantiating the commander
  rhoban_ssl::AICommander* commander;
  if (simulation.getValue())
  {
    commander = new rhoban_ssl::AICommanderSimulation(yellow.getValue());
  }
  else
  {
    commander = new rhoban_ssl::AICommanderReal(yellow.getValue());
  }

  rhoban_ssl::Vision::PartOfTheField part_of_the_field_used;
  if (zone_name.getValue() == "all")
  {
    part_of_the_field_used = rhoban_ssl::Vision::PartOfTheField::ALL_FIELD;
  }
  else if (zone_name.getValue() == "positive")
  {
    part_of_the_field_used = rhoban_ssl::Vision::PartOfTheField::POSIVE_HALF_FIELD;
  }
  else if (zone_name.getValue() == "negative")
  {
    part_of_the_field_used = rhoban_ssl::Vision::PartOfTheField::NEGATIVE_HALF_FIELD;
  }
  else
  {
    std::cerr << "Unknonw zone !" << std::endl;
    assert(false);
  }

  std::string theport;
  if (simulation.getValue())
  {
    theport = sim_port.getValue();
  }
  else
  {
    theport = port.getValue();
  }

  // Viewer API
  API api(team_name.getValue(), simulation.getValue(), yellow.getValue() ? rhoban_ssl::ai::Yellow : rhoban_ssl::ai::Blue,
          commander, config_path.getValue(), part_of_the_field_used, addr.getValue(), theport, theport);

  // Running Qt application
  QApplication a(argc, argv);
  MainWindow w(&api);
  w.show();

  int result = a.exec();

  delete commander;

  return result;
}
