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

lws_protocols ViewerServer::protocols_[3];
std::vector<struct lws*> ViewerServer::clients_;

volatile bool ViewerServer::running_(true);
struct lws_context* ViewerServer::context_;

int ViewerServer::callback_http_dummy(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in,
                                      size_t len)
{
  return 0;
}

int ViewerServer::callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len)
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
      std::cout << "waiiiiiiiiiiiit " << std::endl;
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
  if (!thread_launched_)
  {
    thread_launched_ = true;
    // ViewerServer::running_ = true;

    sigemptyset(&set);
    assert(pthread_sigmask(SIG_BLOCK, &set, NULL) == 0);

    // thread_ = new std::thread(&ViewerServerLauncher::ViewerServer::run);
    // pthread_create(&thread, nullptr, &ViewerServerLauncher::ViewerServer::run, (void*)&set);

    std::cout << "tatatatata" << std::endl;
    std::cout << "tatatatata" << std::endl;
    std::cout << "tatatatata" << std::endl;
    std::cout << "tatatatata" << std::endl;
    std::cout << "tatatatata" << std::endl;
    std::cout << "tatatatata" << std::endl;
  }
  return true;
}

ViewerServerLauncher::~ViewerServerLauncher()
{
  std::cout << "Thread 1" << std::endl;
  // ViewerServer::running_ = false;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pthread_join(thread, nullptr);
  // thread_->join();
  std::cout << "Thread 2" << std::endl;
  // delete thread_;
  instance_counter_--;
}

ViewerServer::ViewerServer() : thread_launched_(false)
{
  // Set the protocols
  protocols_[0] = { "http", rhoban_ssl::viewer::ViewerServer::callback_http_dummy, 0, 0 };
  protocols_[1] = { "viewer_protocol",
                    rhoban_ssl::viewer::ViewerServer::callback_viewer,
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

ViewerServer::~ViewerServer()
{
  rhoban_ssl::viewer::ViewerServer::stop();
  lws_cancel_service(context_);
  std::cout << "wait join : " << running_ << std::endl;
  pthread_join(thread, nullptr);
  //  kill(thread, SIGTERM);
  lws_context_destroy(context_);
  std::cout << "destroy" << std::endl;
}

void ViewerServer::run()
{
  while (running_)
  {
    std::cout << "running" << std::endl;
    if (clients_.size() > 0)
    {
      viewer::ViewerDataGlobal::get().client_connected = true;
      if (!rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.empty())
        lws_callback_on_writable(clients_.at(0));
    }
    else
    {
      viewer::ViewerDataGlobal::get().client_connected = false;
    }
    std::cout << "running2" << std::endl;
    lws_service(context_, 10);
    std::cout << "running3" << std::endl;
  }
  std::cout << "Thread viewer server CLOSED" << std::endl;
}

void* ViewerServer::prun(void*)
{
  sigset_t set;
  sigemptyset(&set);
  assert(pthread_sigmask(SIG_SETMASK, &set, NULL) == 0);
  while (viewer::ViewerServer::running_)
  {
    // std::cout << "running" << std::endl;
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
    std::cout << "running " << viewer::ViewerServer::running_ << std::endl;
    // lws_cancel_service(((ViewerServer*)server)->context_);
    lws_service(viewer::ViewerServer::context_, 10);
    std::cout << "running3" << std::endl;
  }
  // lws_context_destroy(viewer::ViewerServer::context_);
  std::cout << "Thread viewer server CLOSED" << std::endl;
}

void ViewerServer::stop()
{
  viewer::ViewerServer::running_ = false;
  lws_cancel_service(viewer::ViewerServer::context_);
}

bool ViewerServer::runTask()
{
  if (!thread_launched_)
  {
    pthread_create(&thread, nullptr, &ViewerServer::prun, nullptr);
    // kill(thread, SIGINT);
    thread_launched_ = true;
  }
  return true;

  // thread_ = new std::thread(&ViewerServerLauncher::ViewerServer::run);

  // thread_ = new std::thread([this]() { this->run(); });
}

}  // namespace viewer
}  // namespace rhoban_ssl
