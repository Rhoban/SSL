#include "viewer_client.h"
#include <assert.h>
#include <algorithm>

namespace rhoban_ssl
{
namespace viewer
{
ViewerDataGlobal ViewerDataGlobal::instance_;

ViewerDataGlobal::ViewerDataGlobal()
{
}

ViewerDataGlobal& ViewerDataGlobal::get()
{
  return ViewerDataGlobal::instance_;
}

void ViewerDataGlobal::addPacket(const Json::Value& packet)
{
  packets_to_send.push(packet);
}

void ViewerDataGlobal::parseAndStorePacketFromClient(char* packet_received)
{
  Json::Value root;
  Json::Reader reader;
  assert(reader.parse(packet_received, root));
  received_packets.push(root);
}

///////////////////////////////////////////////////////////////////////////////

int ViewerClient::callback_http_dummy(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in,
                                      size_t len)
{
  return 0;
}

std::vector<struct lws*> ViewerClient::clients_;

int ViewerClient::callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len)
{
  switch (reason)
  {
    case LWS_CALLBACK_ESTABLISHED:
      ViewerClient::clients_.push_back(wsi);
      // TODO : the viewer send a request to have informations at initialisation.
      return 0;
    case LWS_CALLBACK_RECEIVE:
      viewer::ViewerDataGlobal::get().parseAndStorePacketFromClient((char*)in);
      return 0;
    case LWS_CALLBACK_SERVER_WRITEABLE:
      return 0;
    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
      return 0;
    case LWS_CALLBACK_CLOSED:
      ViewerClient::clients_.erase(std::remove(ViewerClient::clients_.begin(), ViewerClient::clients_.end(), wsi),
                                   ViewerClient::clients_.end());
      return 0;
    default:
      return 0;
  }
}

ViewerClient::ViewerClient()
{
  // Set the protocols

  protocols_[0] = { "http", rhoban_ssl::viewer::ViewerClient::callback_http_dummy, 0, 0 };
  protocols_[1] = { "viewer_protocol",
                    rhoban_ssl::viewer::ViewerClient::callback_viewer,
                    sizeof(per_session_data_minimal),
                    3000,
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

bool ViewerClient::runTask()
{
  try
  {
    while (!rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.empty())
    {
      Json::Value packet = rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.front();
      if (clients_.size() > 0)
      {
        Json::FastWriter writer = Json::FastWriter();
        std::string str_json = writer.write(packet);
        unsigned char packet_send[str_json.size() + LWS_PRE];
        std::copy(str_json.begin(), str_json.end(), packet_send + LWS_PRE);
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
          lws_write(*it, &packet_send[LWS_PRE], str_json.size(), LWS_WRITE_TEXT);
        }
      }

      rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.pop();
    }

    lws_service(context_, 10);
  }
  catch (...)
  {
  }
  return true;
}

ViewerClient::~ViewerClient()
{
  lws_context_destroy(context_);
}

}  // namespace viewer
}  // namespace rhoban_ssl
