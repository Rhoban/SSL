#include "viewer_client.h"
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

lws_protocols ViewerServerLauncher::ViewerServer::protocols_[3];
std::vector<struct lws*> ViewerServerLauncher::ViewerServer::clients_;
std::atomic<bool> ViewerServerLauncher::ViewerServer::running_(false);
struct lws_context* ViewerServerLauncher::ViewerServer::context_;

ViewerServerLauncher::ViewerServer::ViewerServer()
{
  // Set the protocols
  protocols_[0] = { "http", rhoban_ssl::viewer::ViewerServerLauncher::ViewerServer::callback_http_dummy, 0, 0 };
  protocols_[1] = { "viewer_protocol",
                    rhoban_ssl::viewer::ViewerServerLauncher::ViewerServer::callback_viewer,
                    sizeof(per_session_data_minimal),
                    6000,
                    0,
                    NULL };
  protocols_[2] = { NULL, NULL, 0, 0 };

  struct lws_context_creation_info info;
  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */

  // move to configuration
  info.port = 7882;
  info.protocols = protocols_;

  context_ = lws_create_context(&info);
}

ViewerServerLauncher::ViewerServer::~ViewerServer()
{
  lws_context_destroy(context_);
}

void ViewerServerLauncher::ViewerServer::run()
{
  while (running_)
  {
    if (clients_.size() > 0)
    {
      viewer::ViewerDataGlobal::get().client_connected = true;
      lws_callback_on_writable(clients_.at(0));
    }
    else
    {
      viewer::ViewerDataGlobal::get().client_connected = false;
    }
    lws_service(context_, 10);
  }
  std::cout << "ViewerServer close" << std::endl;
}

int ViewerServerLauncher::ViewerServer::callback_http_dummy(struct lws* wsi, enum lws_callback_reasons reason,
                                                            void* user, void* in, size_t len)
{
  return 0;
}

int ViewerServerLauncher::ViewerServer::callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void* user,
                                                        void* in, size_t len)
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

///////////////////////////////////////////////////////////////////////////////

uint ViewerServerLauncher::instance_counter_ = 0;
ViewerServerLauncher::ViewerServerLauncher()
{
  instance_counter_++;

  // prevent an invalid second instanciation of the task.
  assert(instance_counter_ < 2);
}

bool ViewerServerLauncher::runTask()
{
  if (!viewer_client_.running_)
  {
    viewer_client_.running_ = true;
    thread_ = new std::thread(&ViewerServerLauncher::ViewerServer::run);
  }
  return true;
}

ViewerServerLauncher::~ViewerServerLauncher()
{
  std::cout << "coiucou" << std::endl;
  viewer_client_.running_ = false;
  delete thread_;
}

}  // namespace viewer
}  // namespace rhoban_ssl
