#pragma once

#include "execution_manager.h"
#include <libwebsockets.h>
#include <iostream>
#include <json/json.h>
#include <queue>
#include <vector>

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
  std::queue<Json::Value> packets_to_send;

  /**
   * @brief All packet received from the viewer.
   */
  std::queue<Json::Value> received_packets;

  /**
   * @brief client_connected
   */
  bool client_connected = false;

  /**
   * @brief Parse and add store a packet send by the viewer.
   * @param packet_received in char*.
   */
  void parseAndStorePacketFromClient(char* packet_received);
};

/**
 * @brief The class to communicate with the viewer.
 *
 * Currently only one viewer client is supporte.
 */
class ViewerClient : public Task
{
private:
  /** The callback for the viewer handler.
   *
   * @brief callback_viewer
   * @param wsi Opaque websocket instance pointer
   * @param reason The reason for the call
   * @param user Pointer to per-session user data allocated by library
   * @param in Pointer used for some callback reasons
   * @param len Length set for some callback reasons
   * @return is_success Boolean to see if the callback has success.
   */
  static int callback_http_dummy(struct lws* wsi, enum lws_callback_reasons, void*, void*, size_t);

  /** A dummy callback because the libwebsocket doesn't start if the http is not handled.
   *
   * @brief callback_viewer
   * @param wsi Opaque websocket instance pointer
   * @param reason The reason for the call
   * @param user Pointer to per-session user data allocated by library
   * @param in Pointer used for some callback reasons
   * @param len Length set for some callback reasons
   * @return 0 Boolean to say the callback has success.
   */
  static int callback_viewer(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len);

  /**
   * @brief The protocols structure.
   *
   * Use for register all protocols.
   * The tab is set to three because we have only three protocols registred.
   */
  struct lws_protocols protocols_[3];
  /**
   * @brief The context structure
   * The struct lws_context represents the server.
   */
  struct lws_context* context_;

  /**
   * @brief All clients used.
   */
  static std::vector<struct lws*> clients_;

public:
  /**
   * @brief Constructor.
   *
   * Create context, protocols and websocket.
   */
  ViewerClient();
  /**
   * @brief runTask
   *
   * Use here to launch the callback function if there is a upcoming connection.
   *
   * @return success Boolean to see if the function has success.
   */
  bool runTask() override;
  /**
   * @brief Destructor
   *
   * Destroy the context.
   */
  ~ViewerClient() override;
};
}  // namespace viewer
}  // namespace rhoban_ssl
