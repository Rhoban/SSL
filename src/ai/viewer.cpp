#include "viewer.h"
#include "debug.h"

namespace rhoban_ssl
{
int ViewerCommunication::callback_http_dummy(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in,
                                             size_t len)
{
  return 0;
}

std::vector<struct lws*> ViewerCommunication::clients_;

int ViewerCommunication::callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in,
                                         size_t len)
{
  unsigned char m_Test[20] = "Hello World";
  switch (reason)
  {
    case LWS_CALLBACK_ESTABLISHED:
      DEBUG("Connection - Initialized");
      ViewerCommunication::clients_.push_back(wsi);
      // Generate Game Packet with the first connection.
      rhoban_ssl::viewer::Api::getApi().generateGamePacket();
      return 0;
      break;
    case LWS_CALLBACK_RECEIVE:
      DEBUG("test");
      viewer::Api::getApi().addViewerPacket((char*)in);
      return 0;
      break;
    case LWS_CALLBACK_SERVER_WRITEABLE:
      lws_write(wsi, m_Test, 20, LWS_WRITE_TEXT);
      return 0;
      break;
    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
      DEBUG("Error - We receive the client");
      return 0;
      break;
    case LWS_CALLBACK_CLOSED:
      DEBUG("Connection closed");
      ViewerCommunication::clients_.erase(
          std::remove(ViewerCommunication::clients_.begin(), ViewerCommunication::clients_.end(), wsi),
          ViewerCommunication::clients_.end());
    default:
      return 0;
      break;
  }
}

ViewerCommunication::ViewerCommunication()
{
  // Set the protocols

  protocols_[0] = { "http", rhoban_ssl::ViewerCommunication::callback_http_dummy, 0, 0 };
  protocols_[1] = {
    "viewer_protocol", rhoban_ssl::ViewerCommunication::callback_viewer, sizeof(per_session_data_minimal), 3000, 0, NULL
  };
  protocols_[2] = { NULL, NULL, 0, 0 };

  struct lws_context_creation_info info;
  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */

  info.port = 7882;
  info.protocols = protocols_;

  context_ = lws_create_context(&info);
}

bool ViewerCommunication::runTask()
{
  try
  {
    while (!rhoban_ssl::viewer::Api::getApi().getQueue().empty())
    {
      Json::Value packet = rhoban_ssl::viewer::Api::getApi().getQueue().front();
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

      rhoban_ssl::viewer::Api::getApi().getQueue().pop();
    }

    lws_service(context_, 10);
  }
  catch (...)
  {
  }
  return true;
}

ViewerCommunication::~ViewerCommunication()
{
  lws_context_destroy(context_);
}

}  // namespace rhoban_ssl
