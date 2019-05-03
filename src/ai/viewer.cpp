#include "viewer.h"
#include "debug.h"

rhoban_ssl::ViewerCommunication::ViewerCommunication()
  : context_(1), from_viewer_(context_, ZMQ_PULL), to_viewer_(context_, ZMQ_PUSH)
{
  from_viewer_.connect("tcp://127.0.0.1:5555");
  to_viewer_.bind("tcp://127.0.0.1:5556");
  zmq_poll_ = new zmq::pollitem_t[2];
  zmq_poll_[0].socket = static_cast<void*>(from_viewer_);
  zmq_poll_[0].events = ZMQ_POLLIN;
  zmq_poll_[1].socket = static_cast<void*>(to_viewer_);
  zmq_poll_[1].events = ZMQ_POLLOUT;
}

bool rhoban_ssl::ViewerCommunication::runTask()
{
  try
  {
    zmq::poll(zmq_poll_, 2, 0);
    zmq::message_t message;
    if (zmq_poll_[0].revents & ZMQ_POLLIN)
    {
      from_viewer_.recv(&message);
      DEBUG(message.data<char>());
    }
    if (zmq_poll_[1].revents & ZMQ_POLLOUT)
    {
      to_viewer_.send("world\n", 6, 0);
    }
  }
  catch (...)
  {
  }
  return true;
}
