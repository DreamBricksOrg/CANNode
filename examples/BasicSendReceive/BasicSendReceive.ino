#include <Arduino.h>
#include <CANNode.h>
#include <CANPayloadEncoder.h>

// Create a CANNode instance (adjust TX/RX pins as needed)
CANNode canNode(0x0A1, gpio_num_t(5), gpio_num_t(4));

// State variables
uint8_t time_left = 120;
bool runningState = false;
float fixedDecibel = 0.0f; // will be chosen randomly at startup

/**
 * Optional: callback when a frame is received.
 */
void onReceive(uint32_t id, const uint8_t* data, uint8_t len) {
  if (len >= 6) {
    CANPayload payload = CANPayloadEncoder::decode(data);
    Serial.printf("[RX] From 0x%03X -> running=%d, decibel=%.1f, time_left=%u, volume=%u\n",
                  id, payload.running, payload.decibel, payload.time_left, payload.volume);
  } else {
    Serial.printf("[RX] From 0x%03X with %u bytes\n", id, len);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  randomSeed(analogRead(0)); // seed random generator

  // Choose a fixed random decibel between 0.0 and 99.9
  fixedDecibel = (float)random(0, 1000) / 10.0f; // random int 0..999 then /10
  Serial.printf("[INIT] Fixed decibel chosen: %.1f\n", fixedDecibel);

  Serial.println("[INIT] Starting CANNode...");
  try {
    // Starts at 1 Mbps by default. Pass a CANSpeed to change it, e.g. CAN_SPEED_500KBPS
    canNode.begin();
    canNode.setReceiveCallback(onReceive);
    Serial.println("[INIT] CANNode started!");
  } catch (const CANException &e) {
    Serial.printf("[ERROR] %s\n", e.what());
    while (true) { delay(1000); }
  }
}

void loop() {
  // Toggle running each send
  runningState = !runningState;

  // Build payload
  CANPayload payload;
  payload.running   = runningState;
  payload.decibel   = fixedDecibel;          // constant decibel
  payload.time_left = time_left;
  payload.volume    = random(0, 1001);       // random 0..1000

  // Encode and send
  uint8_t data[8];
  CANPayloadEncoder::encode(payload, data);

  try {
    canNode.send(data, 8);
    Serial.printf("[TX] running=%d decibel=%.1f time_left=%u volume=%u\n",
                  payload.running, payload.decibel, payload.time_left, payload.volume);
  } catch (const CANException &e) {
    Serial.printf("[TX ERROR] %s\n", e.what());
  }

  // Update time_left
  if (time_left == 0) {
    time_left = 120;
  } else {
    time_left--;
  }

  delay(1000); // send every second
}
