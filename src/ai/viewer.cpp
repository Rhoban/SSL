#include "viewer.h"
#include "debug.h"

int rhoban_ssl::ViewerCommunication::callback_http_dummy(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {return 0;}

int rhoban_ssl::ViewerCommunication::callback_viewer(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
    unsigned char m_Test[20] = "Hello World";
    switch (reason)
    {
    case LWS_CALLBACK_ESTABLISHED:
            std::cout << "we receive a client" << std::endl;
            return 0;
            break;
    case LWS_CALLBACK_RECEIVE:
        std::cout << "we receive something" << std::endl;
        std::cout << len << std::endl;
        std::cout << (char *) in << std::endl;
        return 0;
        break;
    case LWS_CALLBACK_SERVER_WRITEABLE:
        lws_write(wsi, m_Test, 20, LWS_WRITE_TEXT);
        return 0;
        break;
    //case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
      //  std::cout << "we receive client refused";
        //return 0;
        //break;
    default:
        return 0;
        break;
    }

}

rhoban_ssl::ViewerCommunication::ViewerCommunication()
{
      // Set the protocols

    protocols_[0] = { "http", rhoban_ssl::ViewerCommunication::callback_http_dummy, 0, 0 };
    protocols_[1] =   {
            "viewer_protocol",
            rhoban_ssl::ViewerCommunication::callback_viewer,
            sizeof(per_session_data_minimal),
            1024,
            0,
            NULL
    };
    protocols_[2] = {
            NULL, NULL, 0,0
    };

      struct lws_context_creation_info info;
      memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */

      info.port = 7880;
      info.protocols = protocols_;

      context_ = lws_create_context(&info);
}



bool rhoban_ssl::ViewerCommunication::runTask()
{
  try
  {
     // Use this to ask to send.
     //lws_callback_on_writable_all_protocol(context_, &protocols[1]);
     lws_service(context_, 10);
  }
  catch (...)
  {
  }
  return true;
}

rhoban_ssl::ViewerCommunication::~ViewerCommunication() {
    lws_context_destroy(context_);
}
