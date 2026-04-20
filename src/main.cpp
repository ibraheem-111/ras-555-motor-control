#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>
#include <math.h>

constexpr gpio_num_t CAN_TXD_PIN = GPIO_NUM_22;
constexpr gpio_num_t CAN_RXD_PIN = GPIO_NUM_23;
constexpr bool CAN_NO_ACK_MODE = false;   // keep false for a real Spark MAX on the bus

constexpr uint32_t DEVICE_ID = 4;

// Spark MAX CAN IDs from the protocol definition
constexpr uint32_t BASE_ID_NON_RIO_HEARTBEAT = 0x2052C80; // broadcast heartbeat for non-RIO master
constexpr uint32_t BASE_ID_VELOCITY_SET      = 0x2050480; // closed-loop velocity setpoint

constexpr int BUTTON_FORWARD_PIN = 32;
constexpr int BUTTON_REVERSE_PIN = 33;

constexpr float TARGET_VELOCITY_RPM = 1000.0f;  // change this to your desired speed
constexpr float STOP_VELOCITY_RPM   = 0.0f;
constexpr float RAMP_RPM_PER_SEC     = 100.0f;  // lower = smoother, higher = snappier

constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5;
constexpr uint32_t CONTROL_INTERVAL_MS   = 5;
constexpr uint32_t DEBOUNCE_MS           = 25;

uint32_t lastHeartbeatMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastDiagMs = 0;

uint32_t txOkCount = 0;
uint32_t txFailCount = 0;

float smoothedVelocityRpm = STOP_VELOCITY_RPM;
float lastRequestedVelocityRpm = 999999.0f;

bool forwardStablePressed = false;
bool reverseStablePressed = false;
bool forwardLastRawPressed = false;
bool reverseLastRawPressed = false;
uint32_t forwardLastChangeMs = 0;
uint32_t reverseLastChangeMs = 0;

uint32_t canAlertsEnabled =
    TWAI_ALERT_TX_SUCCESS |
    TWAI_ALERT_TX_FAILED  |
    TWAI_ALERT_BUS_ERROR  |
    TWAI_ALERT_ARB_LOST   |
    TWAI_ALERT_ERR_PASS   |
    TWAI_ALERT_BUS_OFF    |
    TWAI_ALERT_ABOVE_ERR_WARN |
    TWAI_ALERT_BELOW_ERR_WARN;
// Optional: have the driver also print alerts to UART automatically
// | TWAI_ALERT_AND_LOG;

static inline uint32_t sparkFrameId(uint32_t baseId) {
  return baseId | (DEVICE_ID & 0x3F);
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
  if (forwardPressed && !reversePressed) {
    return TARGET_VELOCITY_RPM;
  }
  if (reversePressed && !forwardPressed) {
    return -TARGET_VELOCITY_RPM;
  }
  return STOP_VELOCITY_RPM;
}

float applyRamp(float currentValue, float targetValue, float deltaTimeSec) {
  float maxStep = RAMP_RPM_PER_SEC * deltaTimeSec;

  if (targetValue > currentValue) {
    currentValue += maxStep;
    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  } else if (targetValue < currentValue) {
    currentValue -= maxStep;
    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}

const char* twaiStateToText(twai_state_t state) {
  switch (state) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "UNKNOWN";
  }
}

bool sendExtendedFrame(uint32_t extId, const uint8_t *data, uint8_t dataLen) {
  if (dataLen > 8) {
    return false;
  }

  twai_message_t msg{};
  msg.identifier = extId;
  msg.extd = 1;
  msg.rtr = 0;
  msg.ss = 1;
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
  return sendExtendedFrame(BASE_ID_NON_RIO_HEARTBEAT, heartbeatData, 8);
}

bool sendVelocitySetpoint(float rpm) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  memcpy(data, &rpm, sizeof(rpm));

  // Bytes 4..7 left zero for a simple setpoint with no extra feedforward/slot fields.
  return sendExtendedFrame(sparkFrameId(BASE_ID_VELOCITY_SET), data, 8);
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
    Serial.println("twai_driver_install() failed");
    return false;
  }

  if (twai_reconfigure_alerts(canAlertsEnabled, nullptr) != ESP_OK) {
    Serial.println("twai_reconfigure_alerts() failed");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("twai_start() failed");
    return false;
  }

  return true;
}

void pollCanAlerts() {
  uint32_t alerts = 0;

  // Non-blocking: read all currently pending alerts
  while (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      Serial.println("CAN ALERT: TX_SUCCESS (frame transmitted successfully)");
    }
    if (alerts & TWAI_ALERT_TX_FAILED) {
      Serial.println("CAN ALERT: TX_FAILED");
    }
    if (alerts & TWAI_ALERT_BUS_ERROR) {
      Serial.println("CAN ALERT: BUS_ERROR (bit/stuff/CRC/form/ACK)");
    }
    if (alerts & TWAI_ALERT_ARB_LOST) {
      Serial.println("CAN ALERT: ARB_LOST");
    }
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
      Serial.println("CAN ALERT: ABOVE_ERR_WARN");
    }
    if (alerts & TWAI_ALERT_BELOW_ERR_WARN) {
      Serial.println("CAN ALERT: BELOW_ERR_WARN");
    }
    if (alerts & TWAI_ALERT_ERR_PASS) {
      Serial.println("CAN ALERT: ERR_PASS");
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("CAN ALERT: BUS_OFF");
    }

    printCanStatus();
  }
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

  Serial.println("Booting SPARK MAX CAN velocity controller...");
  Serial.print("ESP32->Transceiver TXD pin: ");
  Serial.println(static_cast<int>(CAN_TXD_PIN));
  Serial.print("ESP32<-Transceiver RXD pin: ");
  Serial.println(static_cast<int>(CAN_RXD_PIN));
  Serial.print("CAN mode: ");
  Serial.println(CAN_NO_ACK_MODE ? "NO_ACK (single-node test)" : "NORMAL (requires ACK)");
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  Serial.print("Target velocity (RPM): ");
  Serial.println(TARGET_VELOCITY_RPM, 1);
  Serial.print("Ramp rate (RPM/sec): ");
  Serial.println(RAMP_RPM_PER_SEC, 1);
  Serial.print("Forward button pin: ");
  Serial.println(BUTTON_FORWARD_PIN);
  Serial.print("Reverse button pin: ");
  Serial.println(BUTTON_REVERSE_PIN);

  if (!initCan()) {
    Serial.println("CAN init failed");
    while (true) {
      delay(1000);
    }
  }

  sendHeartbeat();
  sendVelocitySetpoint(STOP_VELOCITY_RPM);
  Serial.println("SPARK MAX velocity stream started");
  printCanStatus();
}

void loop() {
  uint32_t nowMs = millis();
  pollCanAlerts();

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

  float targetVelocityRpm = getVelocityFromButtons(forwardPressed, reversePressed);

  if (targetVelocityRpm != lastRequestedVelocityRpm) {
    if (targetVelocityRpm > 0.0f) {
      Serial.println("Command: FORWARD");
    } else if (targetVelocityRpm < 0.0f) {
      Serial.println("Command: REVERSE");
    } else {
      Serial.println("Command: STOP");
    }
    lastRequestedVelocityRpm = targetVelocityRpm;
  }

  if ((nowMs - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    bool ok = sendHeartbeat();
    if (!ok) {
      Serial.println("Heartbeat queue failed");
    }
    lastHeartbeatMs = nowMs;
  }

  if ((nowMs - lastControlMs) >= CONTROL_INTERVAL_MS) {
    float deltaTimeSec = static_cast<float>(nowMs - lastControlMs) / 1000.0f;
    smoothedVelocityRpm = applyRamp(smoothedVelocityRpm, targetVelocityRpm, deltaTimeSec);

    bool ok = sendVelocitySetpoint(smoothedVelocityRpm);
    if (!ok) {
      Serial.println("Velocity frame queue failed");
    }

    lastControlMs = nowMs;
  }

  if ((nowMs - lastDiagMs) >= 1000) {
    printCanStatus();
    lastDiagMs = nowMs;
  }

  delay(5);
}