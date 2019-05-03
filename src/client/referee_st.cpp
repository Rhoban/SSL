#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "referee_client_single_thread.h"
#include "client_config.h"

void stop(int s)
{
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

class RefereeTerminalPrinter : public rhoban_ssl::Task
{
  int counter_ = 0;

public:
  RefereeTerminalPrinter() : counter_(0)
  {
  }
  virtual bool runTask() override
  {
    counter_ += 1;
    std::stringstream ss;

    // ss << "\033[2J\033[1;1H";  // this clear the terminal
    ss << counter_ << std::endl;
    ss << "Refere infos (" << rhoban_ssl::RefereeMessages::singleton_.last_packets_.size() << " pending message)"
       << std::endl;
    if (rhoban_ssl::RefereeMessages::singleton_.last_packets_.size() > 0)
    {
      SSL_Referee* data = rhoban_ssl::RefereeMessages::singleton_.last_packets_.front();
      ss << "stage: " << data->stage() << std::endl;
      ss << "stage_time_left: " << data->stage_time_left() << std::endl;
      ss << "command: " << data->command() << std::endl;
      ss << "command_counter: " << data->command_counter() << std::endl;
      ss << "command_timestamp: " << data->command_timestamp() << std::endl;
      ss << "remaining time: " << data->stage_time_left() << std::endl;

      auto printTeamInfo = [&ss](const SSL_Referee_TeamInfo& teamInfo) {
        ss << "  name: " << teamInfo.name() << std::endl;
        ss << "  score: " << teamInfo.score() << std::endl;
        ss << "  red_cards: " << teamInfo.red_cards() << std::endl;
        ss << "  yellow_cards: " << teamInfo.yellow_cards() << std::endl;
        for (auto yellow_card_time : teamInfo.yellow_card_times())
        {
          ss << "    yellow_card_time: " << yellow_card_time << std::endl;
        }
        ss << "  timeouts: " << teamInfo.timeouts() << std::endl;
        ss << "  timeout_time: " << teamInfo.timeout_time() << std::endl;
        ss << "  goalie: " << teamInfo.goalie() << std::endl;
      };

      ss << "yellow infos:" << std::endl;
      printTeamInfo(data->yellow());
      ss << "blue infos:" << std::endl;
      printTeamInfo(data->blue());

      ss << "designated_position_x: " << data->designated_position().x() << std::endl;
      ss << "designated_position_y: " << data->designated_position().y() << std::endl;
    }
    else
    {
      ss << "*no data*" << std::endl;
    }
    rhoban_ssl::RefereeMessages::singleton_.last_packets_.clear();
    ss << "last packets size is :" << rhoban_ssl::RefereeMessages::singleton_.last_packets_.size() << std::endl;
    std::cout << ss.str();
    std::cout << std::flush;
    return true;
  }
};

int main()
{
  signal(SIGINT, stop);
  rhoban_ssl::ExecutionManager::getManager().addTask(
      new rhoban_ssl::RefereeClientSingleThread(SSL_REFEREE_ADDRESS, SSL_REFEREE_PORT));
  rhoban_ssl::ExecutionManager::getManager().addTask(new RefereeTerminalPrinter());
  rhoban_ssl::ExecutionManager::getManager().addTask(new rhoban_ssl::RefereeProtoBufReset(100));
  rhoban_ssl::ExecutionManager::getManager().run(0.1);
}
