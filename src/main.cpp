#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>

constexpr gpio_num_t CAN_TXD_PIN = GPIO_NUM_22;
constexpr gpio_num_t CAN_RXD_PIN = GPIO_NUM_23;
constexpr bool CAN_NO_ACK_MODE = false;
constexpr uint32_t DEVICE_ID = 4;

constexpr uint32_t BASE_ID_HEARTBEAT = 0x2052C80;
constexpr uint32_t BASE_ID_DUTY_CYCLE_SET = 0x2050080;

constexpr int BUTTON_FORWARD_PIN = 32;
constexpr int BUTTON_REVERSE_PIN = 33;

constexpr float TARGET_DUTY_CYCLE = 1.00f;
constexpr float STOP_DUTY_CYCLE = 0.0f;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5;
constexpr uint32_t CONTROL_INTERVAL_MS = 5;
constexpr uint32_t DEBOUNCE_MS = 25;

uint32_t lastHeartbeatMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastDiagMs = 0;
uint32_t txOkCount = 0;
uint32_t txFailCount = 0;
float lastCommandedDuty = 999999.0f;

bool forwardStablePressed = false;
bool reverseStablePressed = false;
bool forwardLastRawPressed = false;
bool reverseLastRawPressed = false;
uint32_t forwardLastChangeMs = 0;
uint32_t reverseLastChangeMs = 0;

bool updateDebouncedPressed(int pin, bool &lastRawPressed, bool &stablePressed, uint32_t &lastChangeMs, uint32_t nowMs) {
  bool rawPressed = digitalRead(pin) == LOW;

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastChangeMs = nowMs;
  }

  if ((nowMs - lastChangeMs) >= DEBOUNCE_MS) {
    stablePressed = rawPressed;
  }

  return stablePressed;
}

float getDutyFromButtons(bool forwardPressed, bool reversePressed) {
  if (forwardPressed && !reversePressed) {
    return TARGET_DUTY_CYCLE;
  }
  if (reversePressed && !forwardPressed) {
    return -TARGET_DUTY_CYCLE;
  }
  return STOP_DUTY_CYCLE;
}

const char* twaiStateToText(twai_state_t state) {
  switch (state) {
    case TWAI_STATE_STOPPED:
      return "STOPPED";
    case TWAI_STATE_RUNNING:
      return "RUNNING";
    case TWAI_STATE_BUS_OFF:
      return "BUS_OFF";
    case TWAI_STATE_RECOVERING:
      return "RECOVERING";
    default:
      return "UNKNOWN";
  }
}

bool sendExtendedFrame(uint32_t extId, const uint8_t *data, uint8_t dataLen) {
  twai_message_t msg{};
  msg.identifier = extId;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) {
    msg.data[i] = data[i];
  }

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    txOkCount++;
    return true;
  }
  txFailCount++;
  return false;
}

bool sendHeartbeat() {
  const uint8_t heartbeatData[8] = {255, 255, 255, 255, 255, 255, 255, 255};
  return sendExtendedFrame(BASE_ID_HEARTBEAT, heartbeatData, 8);
}

bool sendDutyCycleSetpoint(float dutyCycle) {
  uint8_t controlData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  memcpy(controlData, &dutyCycle, sizeof(dutyCycle));
  uint32_t controlId = BASE_ID_DUTY_CYCLE_SET + DEVICE_ID;
  return sendExtendedFrame(controlId, controlData, 8);
}

void printCanStatus() {
  twai_status_info_t info{};
  if (twai_get_status_info(&info) == ESP_OK) {
    Serial.print("CAN state=");
    Serial.print(twaiStateToText(info.state));
    Serial.print(" tx_ok=");
    Serial.print(txOkCount);
    Serial.print(" tx_fail=");
    Serial.print(txFailCount);
    Serial.print(" tx_err=");
    Serial.print(info.tx_error_counter);
    Serial.print(" rx_err=");
    Serial.print(info.rx_error_counter);
    Serial.print(" tx_failed_cnt=");
    Serial.println(info.tx_failed_count);
  }
}

bool initCan() {
  twai_mode_t mode = CAN_NO_ACK_MODE ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
  twai_general_config_t gConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TXD_PIN, CAN_RXD_PIN, mode);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) {
    return false;
  }

  if (twai_start() != ESP_OK) {
    return false;
  }

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

  Serial.println("Booting SPARK MAX CAN drive...");
  Serial.print("ESP32->Transceiver TXD pin: ");
  Serial.println(static_cast<int>(CAN_TXD_PIN));
  Serial.print("ESP32<-Transceiver RXD pin: ");
  Serial.println(static_cast<int>(CAN_RXD_PIN));
  Serial.print("CAN mode: ");
  Serial.println(CAN_NO_ACK_MODE ? "NO_ACK (single-node test)" : "NORMAL (requires ACK)");
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  Serial.print("Duty target (-1..1): ");
  Serial.println(TARGET_DUTY_CYCLE, 3);
  Serial.print("Forward button pin: ");
  Serial.println(BUTTON_FORWARD_PIN);
  Serial.print("Reverse button pin: ");
  Serial.println(BUTTON_REVERSE_PIN);
  Serial.println("Control: hold FWD or REV, release for STOP");

  if (!initCan()) {
    Serial.println("CAN init failed");
    while (true) {
      delay(1000);
    }
  }

  sendHeartbeat();
  sendDutyCycleSetpoint(STOP_DUTY_CYCLE);
  Serial.println("SPARK MAX CAN stream started");
  printCanStatus();
}

void loop() {
  uint32_t nowMs = millis();

  bool forwardPressed = updateDebouncedPressed(
    BUTTON_FORWARD_PIN,
    forwardLastRawPressed,
    forwardStablePressed,
    forwardLastChangeMs,
    nowMs
  );
  bool reversePressed = updateDebouncedPressed(
    BUTTON_REVERSE_PIN,
    reverseLastRawPressed,
    reverseStablePressed,
    reverseLastChangeMs,
    nowMs
  );

  float activeSetpoint = getDutyFromButtons(forwardPressed, reversePressed);

  if (activeSetpoint != lastCommandedDuty) {
    if (activeSetpoint > 0.0f) {
      Serial.println("Command: FORWARD");
    } else if (activeSetpoint < 0.0f) {
      Serial.println("Command: REVERSE");
    } else {
      Serial.println("Phase: STOP");
    }
    lastCommandedDuty = activeSetpoint;
  }

  if ((nowMs - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
    lastHeartbeatMs = nowMs;
  }

  if ((nowMs - lastControlMs) >= CONTROL_INTERVAL_MS) {
    sendDutyCycleSetpoint(activeSetpoint);
    lastControlMs = nowMs;
  }

  if ((nowMs - lastDiagMs) >= 1000) {
    printCanStatus();
    lastDiagMs = nowMs;
  }

  delay(5);
}