#pragma once

#include <execution_manager.h>
#include <zmq.hpp>

namespace rhoban_ssl
{
class ViewerCommunication : public Task
{
  zmq::context_t context_;
  zmq::socket_t from_viewer_;
  zmq::socket_t to_viewer_;
  zmq::pollitem_t* zmq_poll_;

public:
  ViewerCommunication();
  bool runTask() override;
};
}
