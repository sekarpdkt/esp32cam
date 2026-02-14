#include "wsStream.hpp"

const size_t IMG_CHUNK_SIZE = 12288;

// You'll need to store the previous frame's downsampled data in PSRAM
uint8_t* prevDownsampled = nullptr;
const int IMG_DWONSMPL_GRID_SIZE = 16; // 16x16 grid for sampling

extern uint8_t* imgChunkBuffer;

namespace esp32cam {

WsStreamController::WsStreamController(AsyncWebSocket* server)
  : m_ws(server) {}

void WsStreamController::start() {

  // If already running, we stop the old task and connection first
  if (m_running || m_taskHandle != NULL) {
    Serial.println("[WS] New connection detected. Cleaning up previous session...");

    m_running = false;
    m_abortChunks = true;

    // Wake up the task so it can exit and delete itself
    if (m_taskHandle != NULL) {
        xTaskNotifyGive(m_taskHandle);
        // Wait for it to die (safety loop)
        while(m_taskHandle != NULL) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    // Optional: Close all other existing WS connections to ensure
    // only the latest one is active
    m_ws->closeAll();
  }

  // 1. If a task is already running, kill it properly first
  if (m_taskHandle != NULL) {
    m_running = false;
    m_abortChunks = true;
    xTaskNotifyGive(m_taskHandle);

    // 2. WAIT for the old task to actually delete itself
    // This prevents the memory leak
    while(m_taskHandle != NULL) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  // Switch hardware sensor
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  m_running = true;
  m_abortChunks = true;

  xTaskCreatePinnedToCore(taskEntry, "WsStreamTask", 4096, this, 1, &m_taskHandle, 0);

}

void WsStreamController::stop() {
  m_running = false;
  if (m_taskHandle != NULL) {
    // Notify one last time so it can exit the loop and delete itself
    m_abortChunks=true;
    xTaskNotifyGive(m_taskHandle);
  }
}

void WsStreamController::notifyAck() {
  if (m_taskHandle != NULL && m_running) {
    Serial.printf("%u:Heap size: %u, PSRAM Free: %u - Received ack\n",millis(),ESP.getFreeHeap(),ESP.getFreePsram());
    // The "Hook": Unblocks the task to perform the next capture
    xTaskNotifyGive(m_taskHandle);
  }
}


bool WsStreamController::detectMotion(esp32cam::Frame* frame) {
    if (frame == nullptr) return false;

    // The library doesn't expose PIXFORMAT directly in Frame easily,
    // but we know we are capturing JPEGs.
    // Check width for Sentry Mode (e.g., CIF is 352)
    if (frame->getWidth() > 800) return false;

    const int GRID_SIZE = IMG_DWONSMPL_GRID_SIZE;
    bool motionDetected = false;
    int blocksChanged = 0;
    const int threshold = 15;

    if (m_prevDownsampled == nullptr) {
        m_prevDownsampled = (uint8_t*)ps_malloc(GRID_SIZE * GRID_SIZE);
        if (m_prevDownsampled) memset(m_prevDownsampled, 0, GRID_SIZE * GRID_SIZE);
        return false;
    }

    int blockW = frame->getWidth() / GRID_SIZE;
    int blockH = frame->getHeight() / GRID_SIZE;
    uint8_t* buf = frame->data();
    size_t len = frame->size();

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            int pixelIdx = ((y * blockH) * frame->getWidth()) + (x * blockW);

            // Access data safely using the Frame's buffer
            uint8_t currentValue = buf[pixelIdx % len];

            int diff = abs(currentValue - m_prevDownsampled[y * GRID_SIZE + x]);
            if (diff > threshold) {
                blocksChanged++;
            }
            m_prevDownsampled[y * GRID_SIZE + x] = currentValue;
        }
    }

    return (blocksChanged > (GRID_SIZE * GRID_SIZE / 25));
}

// Update signature in hpp and cpp
bool WsStreamController::sendUXGAInChunks(uint8_t* frameData, size_t frameSize, bool isHighRes) {
    m_abortChunks = false; //
    uint8_t total = (frameSize + IMG_CHUNK_SIZE - 1) / IMG_CHUNK_SIZE; //

    // Get the current real-world time
    uint64_t epochTime = (uint64_t)time(NULL);

    for (uint8_t i = 0; i < total; i++) {
        if (m_abortChunks || !m_running) return false; //

        size_t offset = i * IMG_CHUNK_SIZE; //
        size_t size = (frameSize - offset > IMG_CHUNK_SIZE) ? IMG_CHUNK_SIZE : (frameSize - offset); //

        // 1. Basic Chunk Info
        imgChunkBuffer[0] = i; //
        imgChunkBuffer[1] = total; //

        // 2. Millis (4 bytes)
        uint32_t ms = millis(); //
        memcpy(&imgChunkBuffer[2], &ms, 4); //

        // 3. Epoch Capture Time (8 bytes)
        memcpy(&imgChunkBuffer[6], &epochTime, 8); //

        // 4. Flags and Padding
        imgChunkBuffer[14] = isHighRes ? 1 : 0; //
        memset(&imgChunkBuffer[15], 0, 17); // Reserved for Salt

        // 5. Copy Data at Offset 32
        memcpy(&imgChunkBuffer[32], &frameData[offset], size); //

        m_ws->binaryAll(imgChunkBuffer, size + 32); //

        if(i < total - 1) { //
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdFALSE) break; //
        }
    }
    m_abortChunks = true; //
    return true; //
}
void WsStreamController::taskEntry(void* pvParameters) {
    auto* self = static_cast<WsStreamController*>(pvParameters);
    bool sent;
    bool isHighRes = false;
    int framesInHighRes = 0;
    const int COOLDOWN_FRAMES = 50; // Number of frames to stay in HD after motion
    int ignoreFrames = 10; // Ignore motion for the first 10 frames of any switch

    // 1. Critical Null Check: Ensure the server exists
    if (self == nullptr || self->m_ws == nullptr) {
        Serial.println("[WS] Error: Controller or Server is NULL");
        vTaskDelete(NULL);
        return;
    }
    while (self->m_running) {
      sent=false;
      if (ESP.getFreeHeap() < 20000) { // If less than 20KB free
          Serial.println("Low memory! Skipping frame...");
          vTaskDelay(200 / portTICK_PERIOD_MS);
          continue;
      }

          // 1. SAFETY: If the WebSocket buffer is still busy, DO NOT capture.
          // This prevents the DMA/TCP deadlock.
          // 1. CONGESTION CHECK: Check if the first client is overwhelmed
          if (self->m_ws->count() > 0) {
              // Get the first connected client
              AsyncWebSocketClient* client = self->m_ws->client(0);

              // If the client's outgoing queue is backed up, skip capture
              if (client != nullptr && client->queueLen() > 0) {
                  vTaskDelay(10 / portTICK_PERIOD_MS);
                  continue;
              }
          }

      //    vTaskDelay(200 / portTICK_PERIOD_MS);
      Serial.printf("%u:Heap size: %u - Capturing image\n",millis(),ESP.getFreeHeap());
      // 1. Capture Image (Using library's standard capture)
      auto frame = Camera.capture();
      //imgChunkBuffer

      if (frame != nullptr) {
            /**/
            if (ignoreFrames > 0) {
                ignoreFrames--;
                self->detectMotion(frame.get()); // Still call it to populate m_prevDownsampled
            }
            else if (!isHighRes) {
                // SENTRY MODE: Check for motion in Low-Res
                if (self->detectMotion(frame.get())) {
                    Serial.println("[Motion] Triggered! Switching to UXGA...");
                    isHighRes = true;
                    framesInHighRes = COOLDOWN_FRAMES;
                    // Switch hardware sensor
                    sensor_t * s = esp_camera_sensor_get();
                    s->set_framesize(s, FRAMESIZE_UXGA);

                    // Allow sensor to stabilize (avoid white-out frames)
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    frame.reset(); // Throw away the low-res frame
                    continue;      // Immediately capture new UXGA frame
                }
            } else {
                // RECORDING MODE: Stay in High-Res until cooldown expires
                framesInHighRes--;
                if (framesInHighRes <= 0) {
                    Serial.println("[Motion] Cooldown finished. Reverting to CIF...");
                    isHighRes = false;
                    ignoreFrames = 15; // Ignore flicker caused by the resolution jump
                    sensor_t * s = esp_camera_sensor_get();
                    s->set_framesize(s, FRAMESIZE_QVGA);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
          /**/


        // 2. Send Binary Frame
        if (self->m_ws->count() > 0) {
          Serial.printf("%u:Heap size: %u - Sending image\n",millis(),ESP.getFreeHeap());

          //self->m_ws->binaryAll(frame->data(), frame->size());
          sent = self->sendUXGAInChunks(frame->data(), frame->size(),isHighRes);
          Serial.printf("%u:Heap size: %u - Sent image\n",millis(),ESP.getFreeHeap());
        }

        // 3. Clean up frame immediately to free camera buffer
        frame.reset();

        // 4. WAIT FOR NOTIFICATION (The "Ack" Hook)
        // This blocks the task indefinitely until notifyAck() or stop() is called.
        if(sent) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("Ready for next image...");
      } else {
      Serial.printf("%u:Heap size: %u - Captureing failed\n",millis(),ESP.getFreeHeap());
      // If capture fails, wait a bit before retrying automatically
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
    }

    // --- CRITICAL CLEANUP ---
    Serial.println("[Task] Stopping and freeing memory");

    // Free the motion detection buffer to prevent memory leaks
    if (self->m_prevDownsampled != nullptr) {
        free(self->m_prevDownsampled);
        self->m_prevDownsampled = nullptr;
    }

    self->m_taskHandle = NULL;
    vTaskDelete(NULL);
}

} // namespace esp32cam
