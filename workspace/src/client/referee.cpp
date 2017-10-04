#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "RefereeClient.h"

static volatile bool running = true;

void stop(int s)
{
    running = false;
}

int main()
{
    signal(SIGINT, stop);
    RhobanSSL::RefereeClient client;

    while (running) {
        std::stringstream ss;

        ss << "Refere infos" << std::endl;
        if (client.hasData()) {
            auto data = client.getData();
            ss << "stage: " << data.stage() << std::endl;
            ss << "stage_time_left: " << data.stage_time_left() << std::endl;
            ss << "command: " << data.command() << std::endl;
            ss << "command_counter: " << data.command_counter() << std::endl;
            ss << "command_timestamp: " << data.command_timestamp() << std::endl;
            ss << "remaining time: " << data.stage_time_left() << std::endl;

            auto printTeamInfo = [&ss](SSL_Referee_TeamInfo teamInfo) {
                ss << "  name: " << teamInfo.name() << std::endl;
                ss << "  score: " << teamInfo.score() << std::endl;
                ss << "  red_cards: " << teamInfo.red_cards() << std::endl;
                ss << "  yellow_cards: " << teamInfo.yellow_cards() << std::endl;
                for (auto yellow_card_time : teamInfo.yellow_card_times()) {
                        ss << "    yellow_card_time: " << yellow_card_time << std::endl;
                }
                ss << "  timeouts: " << teamInfo.timeouts() << std::endl;
                ss << "  timeout_time: " << teamInfo.timeout_time() << std::endl;
                ss << "  goalie: " << teamInfo.goalie() << std::endl;
            };

            ss << "yellow infos:" << std::endl;
            printTeamInfo(data.yellow());
            ss << "blue infos:" << std::endl;
            printTeamInfo(data.blue());

            ss << "designated_position_x: " << data.designated_position().x() << std::endl;
            ss << "designated_position_y: " << data.designated_position().y() << std::endl;
        } else {
            ss << "*no data*" << std::endl;
        }

        (void)system("clear");
        std::cout << ss.str();
        std::cout << std::flush;
        usleep(30000);
    }
}
