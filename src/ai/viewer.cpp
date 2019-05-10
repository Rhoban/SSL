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
      return 0;
      break;
    case LWS_CALLBACK_RECEIVE:
      std::cout << "we receive something" << std::endl;
      std::cout << len << std::endl;
      std::cout << (char*)in << std::endl;
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
    "viewer_protocol", rhoban_ssl::ViewerCommunication::callback_viewer, sizeof(per_session_data_minimal), 1024, 0, NULL
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
    // Here send.
    // rhoban_ssl::viewer::Api api = rhoban_ssl::viewer::Api::getApi();
    if (clients_.size() > 0)
    {
      unsigned char m_Test[30] = "Hello World of main";
      // TODO HERE
      // FOR EACH PACKET, SEND EACH THE USER.
      // After that supress the packet.
      for (auto it = clients_.begin(); it != clients_.end(); ++it)
      {
        lws_write(*it, m_Test, 30, LWS_WRITE_TEXT);
        // On accède à l'élément pointé via l'étoile
      }
    }
    // Use this to ask to send.
    // lws_callback_on_writable_all_protocol(context_, &protocols[1]);
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
