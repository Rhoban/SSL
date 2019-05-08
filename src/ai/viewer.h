#pragma once

#include <execution_manager.h>
#include <libwebsockets.h>
#include <iostream>

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
/**
 * @brief The class to communicate with the viewer.
 */
class ViewerCommunication : public Task
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
  static int callback_http_dummy(struct lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len);

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

public:
  /**
   * @brief Constructor.
   *
   * Create context, protocols and websocket.
   */
  ViewerCommunication();
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
  ~ViewerCommunication();
};
}  // namespace rhoban_ssl
