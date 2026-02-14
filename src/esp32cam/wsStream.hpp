#ifndef ESP32CAM_WSSTREAM_HPP
#define ESP32CAM_WSSTREAM_HPP

#include "esp32cam.h"
#include <ESPAsyncWebServer.h>
#include "esp_camera.h" // <--- Add this for camera_fb_t

namespace esp32cam {

/** WS stream options. */
struct WsStreamConfig {
  /**
   * @brief Minimum interval between frame captures in millis.
   *
   * Negative value causes @c WsStreamController::notifyAck to enter continuous mode.
   */
  int minInterval = 200;

  /**
   * @brief Maximum number of frames before disconnecting.
   *
   * Negative value means unlimited.
   */
  int maxFrames = -1;

  /** @brief Time limit of writing one frame in millis. */
  int frameTimeout = 10000;
};



class WsStreamController {
public:
  WsStreamController(AsyncWebSocket* server);


  /**
   * @brief Start the streaming state machine.
   */
  void start();

  /**
   * @brief Stop the streaming state machine.
   */
  void stop();

  /**
   * @brief Hook called when the client sends an "ACK" message.
   */
  void notifyAck();


private:
  static void taskEntry(void* pvParameters);
  bool sendUXGAInChunks(uint8_t* frameData, size_t frameSize, bool isHighRes);
  bool detectMotion(esp32cam::Frame* frame);

  AsyncWebSocket* m_ws;
  TaskHandle_t m_taskHandle = NULL;

  uint8_t* m_prevDownsampled = nullptr;
  volatile bool m_running = false;
  volatile bool m_abortChunks = false;
};

} // namespace esp32cam

#endif
