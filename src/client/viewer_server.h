#pragma once

#include "execution_manager.h"
#include <libwebsockets.h>
#include <iostream>
#include <json/json.h>
#include <vector>
#include <thread_queue.h>
#include <atomic>
/**
 * @brief The per_session_data__minimal struct
 *
 * One of these is created for each client connecting to us
 */
struct per_session_data_minimal
{
  struct per_session_data_minimal* pss_list;
  struct lws* wsi;
  int last;
};

namespace rhoban_ssl
{
namespace viewer
{
/**
 * @brief The ViewerDataGlobal class contains all the data exchanged between the monitor and the ia
 * in the form of json packets.
 */
class ViewerDataGlobal
{
private:
  /**
   * @brief Constructor.
   */
  ViewerDataGlobal();

  /**
   * @brief The singleton of the class.
   */
  static ViewerDataGlobal instance_;

public:
  /**
   * @brief Get the unique instance of the class.
   */
  static ViewerDataGlobal& get();

  /**
   * @brief Store all packets that will be send to clients.
   */
  ThreadQueue<Json::Value> packets_to_send;

  /**
   * @brief All packet received from the viewer.
   */
  ThreadQueue<Json::Value> received_packets;

  /**
   * @brief client_connected
   */
  std::atomic<bool> client_connected;

  /**
   * @brief Parse and add store a packet send by the viewer.
   * @param packet_received in char*.
   */
  void parseAndStorePacketFromClient(char* packet_received);
};

/**
 * @brief The task is use to lauch the server to communicate with the viewers.
 *
 * Permits to launch and close the thread of the viewer websocket server.
 *
 * A thread is launch when the task in created and when the execution manager removes it
 * the thread is automatically close.
 *
 * Currently only one client is supported.
 *
 * Plus, due to the use of libwebsockets all the communication process
 * must be handle in a single thread.
 * To ensure that the ViewerServer task can's be instanciate more than once.
 */
class ViewerServer : public Task
{
private:
  bool thread_launched_;

  std::thread* thread;

  /**
   * @brief thread loop
   *
   * This loop manage the websocket.
   */
  void run();

  /**
   * @brief running_
   */
  static std::atomic<bool> running_;

  /**
   * @brief This member is used to assure that this task is add only once in the execution manager.
   */
  static uint instance_counter_;

  /**
   * @brief The protocols structure.
   *
   * Use for register all protocols.
   * The tab is set to three because we have only three protocols registred.
   */
  static struct lws_protocols protocols_[3];

  /**
   * @brief The context structure
   * The struct lws_context represents the server.
   */
  static struct lws_context* context_;

  // Task interface
public:
  /**
   * @brief runTask
   *
   * Use to launch the thread of the server that handle de communication with the viewers.
   * Also, the task is alive if the thread running.
   * When the task is destroy, it stops the thread and destroys the websocket context.
   *
   * @return boolean to activate/desactivate the task
   */
  bool runTask();

public:
  /**
   * @brief Constructor.
   *
   * Create and init context, protocols and websocket.
   */
  ViewerServer();

  /**
   * @brief ~ViewerServer
   *
   * Destroys context and joins thread.
   */
  ~ViewerServer();

  /**
   * @brief All clients used.
   */
  static std::vector<struct lws*> clients_;

  /**
   * @brief A dummy callback because the libwebsocket doesn't start if the http is not handled.
   * @param wsi Opaque websocket instance pointer
   * @param reason The reason for the call
   * @param user Pointer to per-session user data allocated by library
   * @param in Pointer used for some callback reasons
   * @param len Length set for some callback reasons
   * @return is_success Boolean to see if the callback has success.
   */
  static int callback_http_dummy(struct lws*, enum lws_callback_reasons, void*, void*, size_t);

  /**
   *
   * @brief The callback for the viewer handler.
   * @param wsi Opaque websocket instance pointer
   * @param reason The reason for the call
   * @param user Pointer to per-session user data allocated by library
   * @param in Pointer used for some callback reasons
   * @param len Length set for some callback reasons
   * @return 0 Boolean to say the callback has success.
   */
  static int callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void*, void* in, size_t);
};

}  // namespace viewer
}  // namespace rhoban_ssl
