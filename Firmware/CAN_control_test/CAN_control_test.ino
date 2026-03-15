/**
 * @file MS42DC_CAN_Control_ESP32S3_Native.ino
 * @brief
 *  - This is an Arduino example for controlling the Wheeltec MS42DC stepper motor
 *    using an ESP32-S3.
 *  - THIS VERSION USES THE OFFICIAL ESP32 TWAI DRIVER, NO EXTERNAL LIBRARIES NEEDED.
 * @hardware
 *  - ESP32-S3 Development Board
 *  - CAN Transceiver Module
 *  - Wheeltec MS42DC Motor
 * @wiring
 *  - ESP32-S3 GPIO 20 -> CAN Transceiver [TX]green
 *  - ESP32-S3 GPIO 21 -> CAN Transceiver [RX]white
 */

// Include the official ESP32 TWAI (CAN) driver header.
// This file is part of the ESP32 board package itself. NO LIBRARY INSTALLATION IS REQUIRED.
#include "driver/twai.h"

// --- Configuration ---
// The motor's CAN ID. Default is 1.
const int MOTOR_ID = 1;

// --- Function Prototypes ---
void sendCANMessage(int motorId, byte data[], int dlc);
void setSpeedMode(int motorId, bool isClockwise, int speed_rad_s);
void setPositionMode(int motorId, bool isClockwise, int angle_deg, int speed_rad_s);
void setAbsoluteAngleMode(int motorId, int angle_deg, int speed_rad_s);
void setTorqueMode(int motorId, bool isClockwise, int current_mA, int speed_rad_s);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("ESP32-S3 CAN Bus Control (using Native ESP32 TWAI Driver)");

  // --- Initialize CAN Bus using the native TWAI driver ---
  
  // 1. Configure general CAN settings
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);
  
  // 2. Configure timing for 1MHz baud rate
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  
  // 3. Configure a filter to accept all messages (optional, but good practice)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // 4. Install the TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    while (1);
  }
  Serial.println("TWAI driver installed");

  // 5. Start the TWAI driver
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    while (1);
  }
  Serial.println("TWAI driver started successfully at 1MHz.");
}

void loop() {
  Serial.println("\n--- Starting Robotic Arm Movement Sequence ---");

  Serial.println("Moving to absolute angle: 90 degrees.");
  setAbsoluteAngleMode(MOTOR_ID, 90, 15);
  delay(3000);

  Serial.println("Moving to absolute angle: 180 degrees.");
  setAbsoluteAngleMode(MOTOR_ID, 180, 20);
  delay(3000);

  Serial.println("Returning to home position (0 degrees).");
  setAbsoluteAngleMode(MOTOR_ID, 0, 15);
  delay(3000);

  Serial.println("Moving relative position: 360 degrees clockwise.");
  setPositionMode(MOTOR_ID, true, 360, 25);
  delay(4000);

  Serial.println("Moving relative position: 360 degrees counter-clockwise.");
  setPositionMode(MOTOR_ID, false, 360, 25);
  delay(4000);

  Serial.println("--- Sequence complete. Restarting in 5 seconds. ---");
  delay(5000);
}

/**
 * @brief Sends a generic CAN message using the native TWAI driver.
 */
void sendCANMessage(int motorId, byte data[], int dlc) {
  twai_message_t message;
  message.identifier = motorId;      // Set the CAN ID
  message.flags = TWAI_MSG_FLAG_NONE; // Standard frame
  message.data_length_code = dlc;    // Set the data length
  
  // Copy data into the message buffer
  memcpy(message.data, data, dlc);

  // Transmit the message
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.print("Sent CAN message to ID: 0x");
    Serial.print(motorId, HEX);
    Serial.print(" | DLC: ");
    Serial.print(dlc);
    Serial.print(" | Data: ");
    for (int i = 0; i < dlc; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to queue message for transmission");
  }
}

// --- The following high-level motor control functions remain unchanged ---

void setPositionMode(int motorId, bool isClockwise, int angle_deg, int speed_rad_s) {
  byte data[7];
  unsigned int angle_data = angle_deg * 10;
  unsigned int speed_data = speed_rad_s * 10;
  data[0] = 0x02;
  data[1] = isClockwise ? 0x01 : 0x00;
  data[2] = 0x20;
  data[3] = (angle_data >> 8) & 0xFF;
  data[4] = angle_data & 0xFF;
  data[5] = (speed_data >> 8) & 0xFF;
  data[6] = speed_data & 0xFF;
  sendCANMessage(motorId, data, 7);
}

void setAbsoluteAngleMode(int motorId, int angle_deg, int speed_rad_s) {
  byte data[7];
  unsigned int angle_data = angle_deg * 10;
  unsigned int speed_data = speed_rad_s * 10;
  data[0] = 0x04;
  data[1] = 0x00;
  data[2] = 0x20;
  data[3] = (angle_data >> 8) & 0xFF;
  data[4] = angle_data & 0xFF;
  data[5] = (speed_data >> 8) & 0xFF;
  data[6] = speed_data & 0xFF;
  sendCANMessage(motorId, data, 7);
}

void setSpeedMode(int motorId, bool isClockwise, int speed_rad_s) {
  byte data[7];
  unsigned int speed_data = speed_rad_s * 10;
  data[0] = 0x01;
  data[1] = isClockwise ? 0x01 : 0x00;
  data[2] = 0x20;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = (speed_data >> 8) & 0xFF;
  data[6] = speed_data & 0xFF;
  sendCANMessage(motorId, data, 7);
}

void setTorqueMode(int motorId, bool isClockwise, int current_mA, int speed_rad_s) {
  byte data[7];
  unsigned int current_data = current_mA;
  unsigned int speed_data = speed_rad_s * 10;
  data[0] = 0x03;
  data[1] = isClockwise ? 0x01 : 0x00;
  data[2] = 0x20;
  data[3] = (current_data >> 8) & 0xFF;
  data[4] = current_data & 0xFF;
  data[5] = (speed_data >> 8) & 0xFF;
  data[6] = speed_data & 0xFF;
  sendCANMessage(motorId, data, 7);
}
