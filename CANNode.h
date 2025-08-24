#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "esp_err.h"
#include <functional>

class CANException : public std::exception {
public:
    CANException(const char* msg, esp_err_t code) {
        snprintf(_what, sizeof(_what), "%s (%s)", msg, esp_err_to_name(code));
    }
    const char* what() const noexcept override { return _what; }
private:
    char _what[128];
};

using CANReceiveCallback = std::function<void(uint32_t, const uint8_t*, uint8_t)>;
using CANJSONCallback = std::function<void(const String&)>;

// Supported CAN bus speeds. Default is 1 Mbps.
enum CANSpeed {
    CAN_SPEED_1MBPS,
    CAN_SPEED_500KBPS,
    CAN_SPEED_250KBPS,
    CAN_SPEED_125KBPS
};

class CANNode {
public:
    CANNode(uint32_t address, gpio_num_t txPin, gpio_num_t rxPin);
    ~CANNode();

    void begin(CANSpeed speed = CAN_SPEED_1MBPS);
    void send(const uint8_t* data, uint8_t len);
    bool receive(uint32_t& identifier, uint8_t* data, uint8_t& len, uint32_t timeoutMs = 100);

    void setReceiveCallback(CANReceiveCallback cb);

    // JSON support
    void sendJSON(const String& json);
    void setJSONCallback(CANJSONCallback cb);

private:
    uint32_t _address;
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    bool _started;

    // low-level callback
    CANReceiveCallback _callback;
    TaskHandle_t _rxTaskHandle;

    // json support
    CANJSONCallback _jsonCallback;
    uint8_t _nextMsgId = 0;
    String _jsonBuffer;
    uint8_t _expectedChunks = 0;
    uint8_t _receivedChunks = 0;
    uint8_t _currentMsgId = 0;

    static void rxTask(void* arg);
    void handleFrame(const twai_message_t& msg);
    void startRxTaskIfNeeded();
};
