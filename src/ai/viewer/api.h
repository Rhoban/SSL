/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <list>
#include "ai_packet.pb.h"


namespace rhoban_ssl {
namespace api {
class Api {
private:
    /**
    * @brief Constructor.
    */
    Api();
    /**
    * @brief The singleton of the class.
    */
    static Api api_singleton_;
    /**
     * @brief All packet to store and send.
     */
    std::list<AiPacket> packet_;

public:
    /**
    * @brief Get the unique instance of the class.
    */
    static Api getApi();
};
}
}
