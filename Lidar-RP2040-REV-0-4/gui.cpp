/**
 * @file gui.cpp
 * @brief This file contains the implementation for the GUI command processing.
 * @author The Lidar-RP2040-REV-0-4 Team
 * @version 1.0
 * @date 2025-09-07
 *
 * @details This file implements a serial communication protocol for interacting with a
 * graphical user interface (GUI). It uses a state machine to parse incoming
 * packets and executes commands to configure the device, retrieve data, and
 * control its operation. Now includes support for global parameter configuration.
 */

#include "gui.h"
#include "globals.h"
#include "storage.h"
#include "globals_config.h"  // NEW: Include globals configuration
#include "neopixel_integration.h"

/** @brief The start byte for a GUI packet. */
#define GUI_PACKET_START_BYTE 0x7E
/** @brief The maximum size of the payload in a GUI packet. */
#define GUI_MAX_PAYLOAD_SIZE 64
/** @brief The timeout in milliseconds for receiving a complete GUI packet. */
#define GUI_PACKET_TIMEOUT_MS 100
/** @brief The response code for a successful acknowledgment (ACK). */
#define RSP_ACK 0x06
/** @brief The response code for a negative acknowledgment (NAK). */
#define RSP_NAK 0x15
/** @brief No error. */
#define NAK_ERR_NONE 0x00
/** @brief Bad checksum error. */
#define NAK_ERR_BAD_CHECKSUM 0x01
/** @brief Unknown command error. */
#define NAK_ERR_UNKNOWN_CMD 0x02
/** @brief Invalid payload error. */
#define NAK_ERR_INVALID_PAYLOAD 0x03
/** @brief Execution failure error. */
#define NAK_ERR_EXECUTION_FAIL 0x04
/** @brief Timeout error. */
#define NAK_ERR_TIMEOUT 0x05

/**
 * @brief Defines the states for the GUI packet parser state machine.
 */
enum GuiParserState {
  STATE_WAIT_FOR_START,     ///< Waiting for the start byte of a packet.
  STATE_READ_CMD,           ///< Reading the command byte.
  STATE_READ_LEN,           ///< Reading the payload length byte.
  STATE_READ_PAYLOAD,       ///< Reading the payload data.
  STATE_READ_CHECKSUM       ///< Reading the checksum byte.
};

/**
 * @brief Represents a GUI packet.
 */
struct GuiPacket {
  uint8_t cmd;                           ///< The command byte.
  uint8_t len;                           ///< The length of the payload.
  uint8_t payload[GUI_MAX_PAYLOAD_SIZE]; ///< The payload data.
  uint8_t checksum;                      ///< The checksum of the packet.
};

/**
 * @brief Calculates the checksum for a GUI packet.
 * @param data A pointer to the data to be checksummed.
 * @param length The length of the data.
 * @return The calculated checksum.
 */
uint8_t calculateGuiChecksum(const uint8_t* data, uint8_t length) {
  uint8_t chk = 0;
  for (uint8_t i = 0; i < length; i++) {
    chk += data[i];
  }
  return chk;
}

/**
 * @brief Sends a response packet to the GUI.
 * @param cmd The command byte of the response.
 * @param payload A pointer to the payload data.
 * @param len The length of the payload.
 */
void sendResponsePacket(uint8_t cmd, const uint8_t* payload, uint8_t len) {
  uint8_t buffer[GUI_MAX_PAYLOAD_SIZE + 4];
  buffer[0] = GUI_PACKET_START_BYTE;
  buffer[1] = cmd;
  buffer[2] = len;
  if (payload && len > 0) {
    memcpy(&buffer[3], payload, len);
  }
  uint8_t checksum = calculateGuiChecksum(&buffer[1], len + 2);
  buffer[len + 3] = checksum;
  Serial.write(buffer, len + 4);
}

/**
 * @brief Sends an ACK (acknowledgment) response to the GUI.
 * @param original_cmd The original command that is being acknowledged.
 */
void sendAck(uint8_t original_cmd) {
  sendResponsePacket(RSP_ACK, &original_cmd, 1);
}

/**
 * @brief Sends a NAK (negative acknowledgment) response to the GUI.
 * @param error_code The error code to send.
 */
void sendNak(uint8_t error_code) {
  sendResponsePacket(RSP_NAK, &error_code, 1);
}

/**
 * @brief Executes a GUI command.
 * @param packet The GUI packet containing the command and payload.
 */
void executeGuiCommand(const GuiPacket& packet) {
  safeSerialPrintfln("Core 1: Executing GUI command: 0x%02X", packet.cmd);
  switch (packet.cmd) {
    case 'S': {
        uint8_t payload[9];
        uint32_t frames_received_local, error_flags_local;
        mutex_enter_blocking(&comm_mutex);
        payload[0] = core_comm.switch_code;
        frames_received_local = core_comm.frames_received;
        error_flags_local = core_comm.error_flags;
        mutex_exit(&comm_mutex);
        memcpy(&payload[1], &frames_received_local, 4);
        memcpy(&payload[5], &error_flags_local, 4);
        sendResponsePacket('S', payload, sizeof(payload));
        break;
    }
    case 'D': {
        sendResponsePacket('D', (uint8_t*)currentConfig.distance_thresholds, sizeof(currentConfig.distance_thresholds));
        break;
    }
    case 'd': {
        if (packet.len == 3) {
          uint8_t pos = packet.payload[0];
          uint16_t val = packet.payload[1] | (packet.payload[2] << 8);
          if (pos < 8 && val >= MIN_DISTANCE_CM && val <= MAX_DISTANCE_CM) {
            currentConfig.distance_thresholds[pos] = val;
            sendAck('d');
            triggerGuiSuccessGlow();
          } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        break;
    }
    case 'V': {
        sendResponsePacket('V', (uint8_t*)currentConfig.velocity_min_thresholds, sizeof(currentConfig.velocity_min_thresholds));
        break;
    }
    case 'v': {
        sendResponsePacket('v', (uint8_t*)currentConfig.velocity_max_thresholds, sizeof(currentConfig.velocity_max_thresholds));
        break;
    }
    case 'W': {
        if (saveConfiguration()) {
          sendAck('W');
          triggerGuiSuccessGlow();
        } else sendNak(NAK_ERR_EXECUTION_FAIL);
        break;
    }
    case 'w': {
        if (packet.len == 4) {
          char type = packet.payload[0];
          uint8_t pos = packet.payload[1];
          int16_t val = packet.payload[2] | (packet.payload[3] << 8);
          if (pos < 8 && (type == 'm' || type == 'x')) {
            if (type == 'm') currentConfig.velocity_min_thresholds[pos] = val;
            else currentConfig.velocity_max_thresholds[pos] = val;
            sendAck('w');
            triggerGuiSuccessGlow();
          } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        break;
    }
    case 'T': {
        sendResponsePacket('T', (uint8_t*)currentConfig.trigger_rules, sizeof(currentConfig.trigger_rules));
        break;
    }
    case 't': {
        if (packet.len == 5) {
          uint8_t pos = packet.payload[0];
          if (pos < 8) {
            memcpy(currentConfig.trigger_rules[pos], &packet.payload[1], 4);
            sendAck('t');
            triggerGuiSuccessGlow();
          } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        break;
    }
    case 'M': {
        uint8_t mode = currentConfig.use_velocity_trigger ? 2 : 1;
        sendResponsePacket('M', &mode, 1);
        break;
    }
    case 'm': {
        if (packet.len == 1) {
          uint8_t mode = packet.payload[0];
          if (mode == 1 || mode == 2) {
            currentConfig.use_velocity_trigger = (mode == 2);
            sendAck('m');
            triggerGuiSuccessGlow();
          } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        break;
    }
    case 'G': {
        uint8_t debug_flag = currentConfig.enable_debug ? 1 : 0;
        sendResponsePacket('G', &debug_flag, 1);
        break;
    }
    case 'g': {
        if (packet.len == 1) {
          uint8_t debug_val = packet.payload[0];
          if (debug_val == 0 || debug_val == 1) {
            currentConfig.enable_debug = (debug_val == 1);
            mutex_enter_blocking(&comm_mutex);
            core_comm.enable_debug = currentConfig.enable_debug;
            mutex_exit(&comm_mutex);
            sendAck('g');
            triggerGuiSuccessGlow();
          } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        } else sendNak(NAK_ERR_INVALID_PAYLOAD);
        break;
    }
    // NEW: Global configuration commands
    case 'L': {
        // Read globals response (safe parameters only)
        uint8_t payload[56]; // Size for safe global parameters (13 ints * 4 + 1 float * 4)
        uint8_t idx = 0;
        
        // Integers (4 bytes each, little-endian)
        memcpy(&payload[idx], &runtimeGlobals.config_mode_timeout_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.min_strength_threshold, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.max_recovery_attempts, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.recovery_attempt_delay_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.startup_delay_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.lidar_init_step_delay_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.lidar_final_delay_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.command_response_delay_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.debug_output_interval_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.status_check_interval_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.performance_report_interval_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.critical_error_report_interval_ms, 4); idx += 4;
        memcpy(&payload[idx], &runtimeGlobals.distance_deadband_threshold_cm, 4); idx += 4;
        
        // Float (4 bytes)
        memcpy(&payload[idx], &runtimeGlobals.velocity_deadband_threshold_cm_s, 4); idx += 4;
        
        sendResponsePacket('L', payload, idx);
        break;
    }
    case 'l': {
        // Write globals command (safe parameters only)
        if (packet.len >= 56) {
          uint8_t idx = 0;
          
          // Integers (4 bytes each, little-endian)
          memcpy(&runtimeGlobals.config_mode_timeout_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.min_strength_threshold, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.max_recovery_attempts, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.recovery_attempt_delay_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.startup_delay_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.lidar_init_step_delay_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.lidar_final_delay_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.command_response_delay_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.debug_output_interval_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.status_check_interval_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.performance_report_interval_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.critical_error_report_interval_ms, &packet.payload[idx], 4); idx += 4;
          memcpy(&runtimeGlobals.distance_deadband_threshold_cm, &packet.payload[idx], 4); idx += 4;
          
          // Float (4 bytes)
          memcpy(&runtimeGlobals.velocity_deadband_threshold_cm_s, &packet.payload[idx], 4); idx += 4;
          
          if (validateGlobalConfiguration(runtimeGlobals)) {
            sendAck('l');
            triggerGuiSuccessGlow();
          } else {
            sendNak(NAK_ERR_INVALID_PAYLOAD);
          }
        } else {
          sendNak(NAK_ERR_INVALID_PAYLOAD);
        }
        break;
    }
    case 'R': {
        safeSerialPrintln("Core 1: System reset requested via GUI");
        sendAck('R');
        triggerGuiSuccessGlow();
        delay(100);
        rp2040.restart();
        break;
    }
    case 'F': {
        safeSerialPrintln("Core 1: Factory reset requested via GUI");
        sendAck('F');
        triggerGuiSuccessGlow();
        factoryReset();
        break;
    }
    default:
      safeSerialPrintfln("Core 1: Unknown GUI command: 0x%02X", packet.cmd);
      sendNak(NAK_ERR_UNKNOWN_CMD);
      break;
  }
}

/**
 * @brief Processes incoming serial data from the GUI.
 *
 * @details This function implements a state machine to parse GUI packets from the serial
 * port. It reads the incoming bytes and transitions through different states to
 * identify the start of a packet, read the command and payload, and verify the
 * checksum. Once a valid packet is received, it calls `executeGuiCommand` to
 * process the command.
 */
void processGuiCommands() {
  static GuiParserState state = STATE_WAIT_FOR_START;
  static GuiPacket current_packet;
  static uint8_t payload_index = 0;
  static uint32_t packet_start_time = 0;

  if (state != STATE_WAIT_FOR_START && safeMillisElapsed(packet_start_time, millis()) > GUI_PACKET_TIMEOUT_MS) {
    safeSerialPrintln("Core 1: GUI packet timeout");
    state = STATE_WAIT_FOR_START;
    sendNak(NAK_ERR_TIMEOUT);
  }

  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    switch (state) {
      case STATE_WAIT_FOR_START:
        if (byte == GUI_PACKET_START_BYTE) {
          state = STATE_READ_CMD;
          packet_start_time = millis();
        }
        break;
      case STATE_READ_CMD:
        current_packet.cmd = byte;
        state = STATE_READ_LEN;
        break;
      case STATE_READ_LEN:
        if (byte <= GUI_MAX_PAYLOAD_SIZE) {
          current_packet.len = byte;
          payload_index = 0;
          if (current_packet.len == 0) state = STATE_READ_CHECKSUM;
          else state = STATE_READ_PAYLOAD;
        } else {
          safeSerialPrintln("Core 1: GUI packet invalid length");
          sendNak(NAK_ERR_INVALID_PAYLOAD);
          state = STATE_WAIT_FOR_START;
        }
        break;
      case STATE_READ_PAYLOAD:
        current_packet.payload[payload_index++] = byte;
        if (payload_index >= current_packet.len) {
          state = STATE_READ_CHECKSUM;
        }
        break;
      case STATE_READ_CHECKSUM:
        current_packet.checksum = byte;
        uint8_t buffer_to_check[GUI_MAX_PAYLOAD_SIZE + 2];
        buffer_to_check[0] = current_packet.cmd;
        buffer_to_check[1] = current_packet.len;
        memcpy(&buffer_to_check[2], current_packet.payload, current_packet.len);
        uint8_t calculated_checksum = calculateGuiChecksum(buffer_to_check, current_packet.len + 2);
        if (calculated_checksum == current_packet.checksum) {
          executeGuiCommand(current_packet);
        } else {
          safeSerialPrintfln("Core 1: GUI packet checksum failed. Got: %d, Expected: %d",
            current_packet.checksum, calculated_checksum);
          sendNak(NAK_ERR_BAD_CHECKSUM);
        }
        state = STATE_WAIT_FOR_START;
        break;
    }
  }
}