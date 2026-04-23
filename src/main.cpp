#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>
#include <math.h>

constexpr gpio_num_t CAN_TXD_PIN = GPIO_NUM_22;
constexpr gpio_num_t CAN_RXD_PIN = GPIO_NUM_23;
constexpr bool CAN_NO_ACK_MODE = false;
constexpr uint32_t DEVICE_ID = 4;

constexpr uint32_t BASE_ID_HEARTBEAT        = 0x2052C80;
constexpr uint32_t BASE_ID_VELOCITY_SET     = 0x2050480;
constexpr uint32_t BASE_ID_STATUS_0         = 0x2051800;
constexpr uint32_t BASE_ID_STATUS_1         = 0x2051840;
constexpr uint32_t BASE_ID_STATUS_2         = 0x2051880;

constexpr int BUTTON_FORWARD_PIN = 32;
constexpr int BUTTON_REVERSE_PIN = 33;

constexpr float TARGET_VELOCITY_RPM = 4000.0f;
constexpr float STOP_VELOCITY_RPM   = 0.0f;
constexpr float RAMP_RPM_PER_SEC    = 2000.0f;

constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5;
constexpr uint32_t CONTROL_INTERVAL_MS   = 5;
constexpr uint32_t DEBOUNCE_MS           = 25;
constexpr uint32_t PRINT_INTERVAL_MS     = 200;

uint32_t lastHeartbeatMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;

float smoothedVelocityRpm = STOP_VELOCITY_RPM;
float lastRequestedVelocityRpm = 999999.0f;

bool forwardStablePressed = false;
bool reverseStablePressed = false;
bool forwardLastRawPressed = false;
bool reverseLastRawPressed = false;
uint32_t forwardLastChangeMs = 0;
uint32_t reverseLastChangeMs = 0;

// Telemetry from Spark MAX
float rxAppliedOutput = NAN;
float rxVelocityRpm = NAN;
float rxPositionRot = NAN;
uint16_t rxFaults = 0;
uint16_t rxStickyFaults = 0;
uint32_t rxStatus0Count = 0;
uint32_t rxStatus1Count = 0;
uint32_t rxStatus2Count = 0;

static inline uint32_t sparkFrameId(uint32_t baseId) {
  return baseId | (DEVICE_ID & 0x3F);
}

float bytesToFloatLE(const uint8_t *data) {
  uint32_t raw =
      ((uint32_t)data[3] << 24) |
      ((uint32_t)data[2] << 16) |
      ((uint32_t)data[1] << 8)  |
      ((uint32_t)data[0] << 0);

  float out;
  memcpy(&out, &raw, sizeof(out));
  return out;
}

bool updateDebouncedPressed(
  int pin,
  bool &lastRawPressed,
  bool &stablePressed,
  uint32_t &lastChangeMs,
  uint32_t nowMs
) {
  bool rawPressed = (digitalRead(pin) == LOW);

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastChangeMs = nowMs;
  }

  if ((nowMs - lastChangeMs) >= DEBOUNCE_MS) {
    stablePressed = rawPressed;
  }

  return stablePressed;
}

float getVelocityFromButtons(bool forwardPressed, bool reversePressed) {
  if (forwardPressed && !reversePressed) return TARGET_VELOCITY_RPM;
  if (reversePressed && !forwardPressed) return -TARGET_VELOCITY_RPM;
  return STOP_VELOCITY_RPM;
}

float applyRamp(float currentValue, float targetValue, float deltaTimeSec) {
  float maxStep = RAMP_RPM_PER_SEC * deltaTimeSec;

  if (targetValue > currentValue) {
    currentValue += maxStep;
    if (currentValue > targetValue) currentValue = targetValue;
  } else if (targetValue < currentValue) {
    currentValue -= maxStep;
    if (currentValue < targetValue) currentValue = targetValue;
  }

  return currentValue;
}

bool sendExtendedFrame(uint32_t extId, const uint8_t *data, uint8_t dataLen) {
  if (dataLen > 8) return false;

  twai_message_t msg{};
  msg.identifier = extId;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = dataLen;

  for (uint8_t i = 0; i < dataLen; i++) {
    msg.data[i] = data[i];
  }

  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

bool sendHeartbeat() {
  const uint8_t heartbeatData[8] = {255,255,255,255,255,255,255,255};
  return sendExtendedFrame(BASE_ID_HEARTBEAT, heartbeatData, 8);
}

bool sendVelocitySetpoint(float rpm) {
  uint8_t data[8] = {0,0,0,0,0,0,0,0};
  memcpy(data, &rpm, sizeof(rpm));
  return sendExtendedFrame(sparkFrameId(BASE_ID_VELOCITY_SET), data, 8);
}

bool sendPeriodicFramePeriod(uint32_t baseId, uint16_t periodMs) {
  uint8_t data[2];
  data[0] = (uint8_t)(periodMs & 0xFF);
  data[1] = (uint8_t)((periodMs >> 8) & 0xFF);
  return sendExtendedFrame(sparkFrameId(baseId), data, 2);
}

void pollCanRx() {
  twai_message_t msg{};

  while (twai_receive(&msg, 0) == ESP_OK) {
    if (!msg.extd) continue;

    if (msg.identifier == sparkFrameId(BASE_ID_STATUS_0) && msg.data_length_code >= 6) {
      int16_t appliedRaw = (int16_t)(((uint16_t)msg.data[1] << 8) | msg.data[0]);
      rxAppliedOutput = (float)appliedRaw / 32767.0f;
      rxFaults = (uint16_t)(((uint16_t)msg.data[3] << 8) | msg.data[2]);
      rxStickyFaults = (uint16_t)(((uint16_t)msg.data[5] << 8) | msg.data[4]);
      rxStatus0Count++;
    }
    else if (msg.identifier == sparkFrameId(BASE_ID_STATUS_1) && msg.data_length_code >= 4) {
      rxVelocityRpm = bytesToFloatLE(msg.data);
      rxStatus1Count++;
    }
    else if (msg.identifier == sparkFrameId(BASE_ID_STATUS_2) && msg.data_length_code >= 4) {
      rxPositionRot = bytesToFloatLE(msg.data);
      rxStatus2Count++;
    }
  }
}

bool initCan() {
  twai_mode_t mode = CAN_NO_ACK_MODE ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
  twai_general_config_t gConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TXD_PIN, CAN_RXD_PIN, mode);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_REVERSE_PIN, INPUT_PULLUP);

  uint32_t nowMs = millis();
  forwardLastRawPressed = (digitalRead(BUTTON_FORWARD_PIN) == LOW);
  reverseLastRawPressed = (digitalRead(BUTTON_REVERSE_PIN) == LOW);
  forwardStablePressed = forwardLastRawPressed;
  reverseStablePressed = reverseLastRawPressed;
  forwardLastChangeMs = nowMs;
  reverseLastChangeMs = nowMs;

  Serial.println("Booting SPARK MAX velocity diagnostic...");

  if (!initCan()) {
    Serial.println("CAN init failed");
    while (true) delay(1000);
  }

  sendHeartbeat();
  sendVelocitySetpoint(0.0f);

  // Explicitly request telemetry
  sendPeriodicFramePeriod(BASE_ID_STATUS_0, 10);
  sendPeriodicFramePeriod(BASE_ID_STATUS_1, 20);
  sendPeriodicFramePeriod(BASE_ID_STATUS_2, 20);

  Serial.println("Telemetry requested: status0=10ms status1=20ms status2=20ms");
}

void loop() {
  uint32_t nowMs = millis();

  pollCanRx();

  bool forwardPressed = updateDebouncedPressed(
    BUTTON_FORWARD_PIN, forwardLastRawPressed, forwardStablePressed, forwardLastChangeMs, nowMs
  );
  bool reversePressed = updateDebouncedPressed(
    BUTTON_REVERSE_PIN, reverseLastRawPressed, reverseStablePressed, reverseLastChangeMs, nowMs
  );

  float targetVelocityRpm = getVelocityFromButtons(forwardPressed, reversePressed);

  if (targetVelocityRpm != lastRequestedVelocityRpm) {
    if (targetVelocityRpm > 0.0f) Serial.println("Command: FORWARD");
    else if (targetVelocityRpm < 0.0f) Serial.println("Command: REVERSE");
    else Serial.println("Command: STOP");
    lastRequestedVelocityRpm = targetVelocityRpm;
  }

  if ((nowMs - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
    lastHeartbeatMs = nowMs;
  }

  if ((nowMs - lastControlMs) >= CONTROL_INTERVAL_MS) {
    float dt = (float)(nowMs - lastControlMs) / 1000.0f;
    smoothedVelocityRpm = applyRamp(smoothedVelocityRpm, targetVelocityRpm, dt);
    sendVelocitySetpoint(smoothedVelocityRpm);
    lastControlMs = nowMs;
  }

  if ((nowMs - lastPrintMs) >= PRINT_INTERVAL_MS) {
    Serial.print("cmd_rpm=");
    Serial.print(smoothedVelocityRpm, 1);
    Serial.print("  rx_rpm=");
    Serial.print(rxVelocityRpm, 1);
    Serial.print("  rx_pos=");
    Serial.print(rxPositionRot, 3);
    Serial.print("  applied=");
    Serial.print(rxAppliedOutput, 3);
    Serial.print("  faults=0x");
    Serial.print(rxFaults, HEX);
    Serial.print("  sticky=0x");
    Serial.print(rxStickyFaults, HEX);
    Serial.print("  counts[");
    Serial.print(rxStatus0Count);
    Serial.print(",");
    Serial.print(rxStatus1Count);
    Serial.print(",");
    Serial.print(rxStatus2Count);
    Serial.println("]");
    lastPrintMs = nowMs;
  }

  delay(5);
}