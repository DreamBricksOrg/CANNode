#include "CANNode.h"

CANNode::CANNode(uint32_t address, gpio_num_t txPin, gpio_num_t rxPin)
: _address(address), _txPin(txPin), _rxPin(rxPin), _started(false), _callback(nullptr), _rxTaskHandle(nullptr) {}

CANNode::~CANNode() {
    if (_rxTaskHandle) {
        vTaskDelete(_rxTaskHandle);
    }
    if (_started) {
        twai_stop();
        twai_driver_uninstall();
    }
}

void CANNode::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t res;
    res = twai_driver_install(&g_config, &t_config, &f_config);
    if (res != ESP_OK) throw CANException("Driver install failed", res);

    res = twai_start();
    if (res != ESP_OK) throw CANException("TWAI start failed", res);

    _started = true;
}

void CANNode::send(const uint8_t* data, uint8_t len) {
    if (len > 8) len = 8;
    twai_message_t msg{};
    msg.identifier = _address;
    msg.data_length_code = len;
    msg.flags = 0;
    memcpy(msg.data, data, len);

    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (res != ESP_OK) throw CANException("Transmit failed", res);
}

bool CANNode::receive(uint32_t& identifier, uint8_t* data, uint8_t& len, uint32_t timeoutMs) {
    twai_message_t rx_msg;
    esp_err_t res = twai_receive(&rx_msg, pdMS_TO_TICKS(timeoutMs));
    if (res == ESP_ERR_TIMEOUT) {
        return false;
    }
    if (res != ESP_OK) {
        throw CANException("Receive failed", res);
    }
    identifier = rx_msg.identifier;
    len = rx_msg.data_length_code;
    memcpy(data, rx_msg.data, len);
    return true;
}

/*
void CANNode::setReceiveCallback(CANReceiveCallback cb) {
    _callback = cb;

    // if a previous task exists, kill it
    if (_rxTaskHandle) {
        vTaskDelete(_rxTaskHandle);
        _rxTaskHandle = nullptr;
    }

    if (_callback) {
        // create a task to listen
        xTaskCreatePinnedToCore(rxTask, "CANNodeRxTask", 4096, this, 5, &_rxTaskHandle, tskNO_AFFINITY);
    }
}*/

void CANNode::startRxTaskIfNeeded() {
    if (_callback == nullptr && _jsonCallback == nullptr && _rxTaskHandle != nullptr) {
      vTaskDelete(_rxTaskHandle);
      _rxTaskHandle == nullptr;
    }
    else if (_rxTaskHandle == nullptr) {
        xTaskCreatePinnedToCore(rxTask, "CANNodeRxTask", 4096, this, 5, &_rxTaskHandle, tskNO_AFFINITY);
    }
}

void CANNode::setReceiveCallback(CANReceiveCallback cb) {
    _callback = cb;
    startRxTaskIfNeeded();
}

void CANNode::setJSONCallback(CANJSONCallback cb) {
    _jsonCallback = cb;
    startRxTaskIfNeeded();
}

void CANNode::rxTask(void* arg) {
    CANNode* self = static_cast<CANNode*>(arg);
    twai_message_t rx_msg;
    while (true) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            self->handleFrame(rx_msg);
        }
    }
}

void CANNode::handleFrame(const twai_message_t& msg) {
    // Always call raw callback
    if (_callback) {
        _callback(msg.identifier, msg.data, msg.data_length_code);
    }

    // Try decode as JSON frame if callback set
    if (_jsonCallback && msg.data_length_code >= 3) {
        uint8_t msgId = msg.data[0];
        uint8_t chunkIdx = msg.data[1];
        uint8_t chunkCount = msg.data[2];

        // Start of a new message
        if (chunkIdx == 0) {
            _jsonBuffer = "";
            _expectedChunks = chunkCount;
            _receivedChunks = 0;
            _currentMsgId = msgId;
        }

        if (msgId == _currentMsgId) {
            // Append payload bytes after header
            for (int i = 3; i < msg.data_length_code; i++) {
                _jsonBuffer += (char)msg.data[i];
            }
            _receivedChunks++;
        }

        if (_receivedChunks == _expectedChunks) {
            // Complete message
            _jsonCallback(_jsonBuffer);
            _jsonBuffer = "";
            _expectedChunks = 0;
            _receivedChunks = 0;
        }
    }
}

void CANNode::sendJSON(const String& json) {
    // split json into chunks
    const uint8_t HEADER = 3;
    const uint8_t MAX_PAYLOAD = 8 - HEADER;
    int totalLen = json.length();
    uint8_t chunks = (totalLen + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

    uint8_t msgId = _nextMsgId++;
    for (uint8_t idx = 0; idx < chunks; idx++) {
        twai_message_t msg{};
        msg.identifier = _address;
        msg.flags = 0;

        msg.data[0] = msgId;
        msg.data[1] = idx;
        msg.data[2] = chunks;

        int offset = idx * MAX_PAYLOAD;
        int remaining = totalLen - offset;
        int copyLen = remaining > MAX_PAYLOAD ? MAX_PAYLOAD : remaining;
        for (int i = 0; i < copyLen; i++) {
            msg.data[3 + i] = (uint8_t)json[offset + i];
        }

        msg.data_length_code = 3 + copyLen;

        esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(100));
        if (res != ESP_OK) throw CANException("Transmit JSON chunk failed", res);
    }
}
