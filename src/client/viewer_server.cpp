#include "viewer_server.h"
#include <assert.h>
#include <algorithm>

namespace rhoban_ssl
{
namespace viewer
{
ViewerDataGlobal ViewerDataGlobal::instance_;

ViewerDataGlobal::ViewerDataGlobal() : client_connected(false)
{
}

ViewerDataGlobal& ViewerDataGlobal::get()
{
  return ViewerDataGlobal::instance_;
}

void ViewerDataGlobal::parseAndStorePacketFromClient(char* packet_received)
{
  Json::Value root;
  Json::Reader reader;
  assert(reader.parse(packet_received, root));
  received_packets.push(root);
}

///////////////////////////////////////////////////////////////////////////////

std::atomic<bool> ViewerServer::running_(true);
std::vector<struct lws*> ViewerServer::clients_;
struct lws_context* ViewerServer::context_;
lws_protocols ViewerServer::protocols_[2];

uint ViewerServer::instance_counter_ = 0;

void ViewerServer::run()
{
  sigset_t set;
  sigemptyset(&set);
  assert(pthread_sigmask(SIG_SETMASK, &set, NULL) == 0);
  std::cout << "Thread viewer server STARTED" << std::endl;
  while (viewer::ViewerServer::running_)
  {
    if (clients_.size() > 0)
    {
      viewer::ViewerDataGlobal::get().client_connected = true;
      if (!ViewerDataGlobal::get().packets_to_send.empty())
        lws_callback_on_writable(clients_.at(0));
    }
    else
    {
      viewer::ViewerDataGlobal::get().client_connected = false;
    }
    lws_service(viewer::ViewerServer::context_, 10);
  }
  std::cout << "Thread viewer server CLOSED" << std::endl;
}

ViewerServer::ViewerServer() : thread_launched_(false)
{
  // prevent an invalid second instanciation of the task.
  instance_counter_++;
  assert(instance_counter_ < 2);

  // Set the protocols
  protocols_[0] = { "viewer_protocol",
                    rhoban_ssl::viewer::ViewerServer::callback_viewer,
                    sizeof(per_session_data_minimal),
                    6000,
                    0,
                    nullptr };
  protocols_[1] = { nullptr, nullptr, 0, 0, 0, nullptr };

  struct lws_context_creation_info info;
  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */

  // move to configuration
  info.port = 7882;
  info.protocols = protocols_;
  // info.vhost_name = "localhost";

  lws_set_log_level(2, nullptr);
  context_ = lws_create_context(&info);
}

ViewerServer::~ViewerServer()
{
  viewer::ViewerServer::running_ = false;
  lws_cancel_service(viewer::ViewerServer::context_);
  thread->join();
  delete thread;
  lws_context_destroy(context_);
}

bool ViewerServer::runTask()
{
  if (!thread_launched_)
  {
    thread = new std::thread([this]() { this->run(); });
    thread_launched_ = true;
  }
  return true;
}

int ViewerServer::callback_http_dummy(struct lws*, enum lws_callback_reasons, void*, void*, size_t)
{
  return 0;
}

int ViewerServer::callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void*, void* in, size_t)
{
  switch (reason)
  {
    case LWS_CALLBACK_ESTABLISHED:
      ViewerServer::clients_.push_back(wsi);
      return 0;
    case LWS_CALLBACK_RECEIVE:
      std::cout << (char*)in << std::endl;
      viewer::ViewerDataGlobal::get().parseAndStorePacketFromClient((char*)in);
      return 0;
    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
      Json::Value packet = rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.pop();
      Json::FastWriter writer = Json::FastWriter();
      std::string str_json = writer.write(packet);
      unsigned char packet_send[str_json.size() + LWS_PRE];
      std::copy(str_json.begin(), str_json.end(), packet_send + LWS_PRE);

      if (lws_send_pipe_choked(wsi) == 0)
      {
        lws_write(wsi, &packet_send[LWS_PRE], str_json.size(), LWS_WRITE_TEXT);
      }
    }
    break;
    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
      break;
    case LWS_CALLBACK_CLOSED:
      ViewerServer::clients_.erase(std::remove(ViewerServer::clients_.begin(), ViewerServer::clients_.end(), wsi),
                                   ViewerServer::clients_.end());
      break;
    default:
      break;
  }
  return 0;
}

}  // namespace viewer
}  // namespace rhoban_ssl
