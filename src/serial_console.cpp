#include "serial_console.h"
#include "string.h"
#include "stdlib.h"

#define Serial SerialUSB
#define MAX_CMD_LENGTH 128

// Command buffer
static char cmd_buffer[MAX_CMD_LENGTH];
static uint8_t cmd_index = 0;

void handleHelpCommand() {
  Serial.println("\n=== VCU Serial Console ===");
  Serial.println("\nAvailable Commands:");
  Serial.println("  help              - Show this help message");
  Serial.println("  status            - Show system status");
  Serial.println("  inputs            - Read all L9966 inputs");
  Serial.println("  input <ch>        - Read specific input channel (1-15)");
  Serial.println("  monitor           - Continuously monitor all inputs (press any key to stop)");
  Serial.println("  outputs           - Show all L9026 output states");
  Serial.println("  output <dev> <ch> <state> - Set output (dev: 0-1, ch: 0-7, state: 0/1)");
  Serial.println("  relay <name> <state>      - Control relay by name (state: on/off)");
  Serial.println("  diagnostics       - Show L9026 diagnostic information");
  Serial.println("  test              - Run output test sequence");
  Serial.println("\nRelay Names:");
  Serial.println("  efi1, efi23, starter, acc, etcs, stcut");
  Serial.println("  fan3, fan12, ac, blower, injector, ls0");
  Serial.println();
}

void handleStatusCommand() {
  Serial.println("\n=== System Status ===");

  // L9966 Status
  Serial.println("\n--- L9966 Input Driver ---");
  L9966_GSR gsr = l9966.getGeneralStatus();
  Serial.printf("Configuration Reset: %s\n", gsr.configuration_reset ? "YES" : "NO");
  Serial.printf("Using Calibrated ADC: %s\n", gsr.using_calibrated_adc ? "YES" : "NO");
  Serial.printf("Voltage Supply Fault: %s\n", gsr.voltage_supply_fault ? "YES" : "NO");
  Serial.printf("Over Temperature: %s\n", gsr.over_temperature_fault ? "YES" : "NO");

  L9966_DeviceInfo info = l9966.getDeviceInfo();
  Serial.printf("Device Version: 0x%02X\n", info.device_version);
  Serial.printf("Hardware Revision: 0x%02X\n", info.hardware_revision);
  Serial.printf("Device ID: 0x%02X\n", info.device_id);

  // L9026 Status
  Serial.println("\n--- L9026 Output Drivers ---");
  for (uint8_t dev = 0; dev < outputs.getDeviceCount(); dev++) {
    L9026DeviceStatus status = outputs.getDeviceStatus(dev);
    Serial.printf("\nDevice %d:\n", dev);
    Serial.printf("  Operating Mode: ");
    switch (status.operating_mode) {
      case L9026_SLEEP_MODE: Serial.println("SLEEP"); break;
      case L9026_LIMP_HOME_MODE: Serial.println("LIMP HOME"); break;
      case L9026_IDLE_MODE: Serial.println("IDLE"); break;
      case L9026_ACTIVE_MODE: Serial.println("ACTIVE"); break;
    }
    Serial.printf("  Disable Status: %s\n", status.disable_status ? "DISABLED" : "ENABLED");
    Serial.printf("  Idle Status: %s\n", status.idle_status ? "IDLE" : "ACTIVE");
    Serial.printf("  Power-On Reset: %s\n", status.power_on_reset_condition_detected ? "DETECTED" : "OK");
    Serial.printf("  Overcurrent/Overtemp: %s\n", status.overcurrent_overtemperature_detected ? "FAULT" : "OK");
    Serial.printf("  Off-State Diagnostic Failure: %s\n", status.off_state_diagnostic_failure_detected ? "FAULT" : "OK");
    Serial.printf("  VDDIO Undervoltage: %s\n", status.vddio_undervoltage_detected ? "FAULT" : "OK");
    Serial.printf("  VBATT Undervoltage: %s\n", status.vbatt_undervoltage_detected ? "FAULT" : "OK");
  }
  Serial.println();
}

void printInputStatus(uint8_t channel) {
  L9966_DigitalInputStatus digital_status = l9966.getDigitalInputStatus();
  bool digital_state = (digital_status.channel_states >> (channel - 1)) & 0x01;

  switch (channel) {
    case INPUT_CRUISE_CONTROL:
      {
        uint16_t resistance = readInputResistance(channel);
        Serial.printf("CH%d (Cruise Control): Digital=%s, Resistance=%u\n",
                      channel, digital_state ? "HIGH" : "LOW", resistance);
      }
      break;

    case INPUT_ACCEL_PEDAL_1:
    case INPUT_ACCEL_PEDAL_2:
      {
        uint16_t voltage = readInputVoltage(channel);
        float volts = (voltage / 4095.0f) * 5.0f;
        Serial.printf("CH%d (Accel Pedal %d): Digital=%s, Voltage=%.3fV (raw=%u)\n",
                      channel, channel - 1, digital_state ? "HIGH" : "LOW", volts, voltage);
      }
      break;

    case INPUT_CLUTCH_SWITCH:
      Serial.printf("CH%d (Clutch Switch): %s\n",
                    channel, digital_state ? "PRESSED" : "RELEASED");
      break;

    case INPUT_AC_PRESSURE_SWITCH:
      Serial.printf("CH%d (AC Pressure Switch): %s\n",
                    channel, digital_state ? "HIGH" : "LOW");
      break;

    case INPUT_DLC_DIAGNOSTIC:
      Serial.printf("CH%d (DLC Diagnostic): %s\n",
                    channel, digital_state ? "NORMAL" : "DIAGNOSTIC MODE");
      break;

    case INPUT_ENGINE_COOLANT_TEMP:
      {
        uint16_t resistance = readInputResistance(channel);
        Serial.printf("CH%d (Coolant Temp): Digital=%s, Resistance=%u\n",
                      channel, digital_state ? "HIGH" : "LOW", resistance);
      }
      break;

    case INPUT_OIL_TEMP:
      {
        uint16_t resistance = readInputResistance(channel);
        Serial.printf("CH%d (Oil Temp): Digital=%s, Resistance=%u\n",
                      channel, digital_state ? "HIGH" : "LOW", resistance);
      }
      break;

    case INPUT_BRAKE_SWITCH_NC:
      Serial.printf("CH%d (Brake Switch NC): %s\n",
                    channel, digital_state ? "CLOSED" : "OPEN");
      break;

    case INPUT_BRAKE_SWITCH_NO:
      Serial.printf("CH%d (Brake Switch NO): %s\n",
                    channel, digital_state ? "CLOSED" : "OPEN");
      break;

    default:
      {
        uint16_t voltage = readInputVoltage(channel);
        float volts = (voltage / 4095.0f) * 5.0f;
        Serial.printf("CH%d: Digital=%s, Voltage=%.3fV (raw=%u)\n",
                      channel, digital_state ? "HIGH" : "LOW", volts, voltage);
      }
      break;
  }
}

void handleReadAllInputsCommand() {
  Serial.println("\n=== L9966 Input Status ===\n");

  // Read digital input status once
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();

  // Print all configured inputs
  for (uint8_t ch = 1; ch <= 11; ch++) {
    if (ch == 6) continue;  // Skip channel 6 (misconfigured)
    printInputStatus(ch);
  }

  // Also print the combined accelerator pedal reading
  Serial.println("\n--- Accelerator Pedal (Combined) ---");
  AccelPedalReading pedal = getAccelPedalPosition();
  Serial.printf("Pedal Position: %.1f%%\n", pedal.position_percent);
  Serial.printf("Valid: %s\n", pedal.valid ? "YES" : "NO");
  Serial.printf("Pedal 1: %.3fV (raw=%u)\n", pedal.pedal1_voltage, pedal.pedal1_raw);
  Serial.printf("Pedal 2: %.3fV (raw=%u)\n", pedal.pedal2_voltage, pedal.pedal2_raw);

  // Print combined brake state
  Serial.println("\n--- Brake Switch (Combined) ---");
  bool brake_pressed = getBrakeSwitchState();
  Serial.printf("Brake: %s\n", brake_pressed ? "PRESSED" : "RELEASED");

  Serial.println();
}

void handleReadInputCommand(const char* args) {
  if (args == nullptr || strlen(args) == 0) {
    Serial.println("ERROR: Missing channel number. Usage: input <channel>");
    return;
  }

  int channel = atoi(args);
  if (channel < 1 || channel > 15) {
    Serial.println("ERROR: Channel must be between 1 and 15");
    return;
  }

  if (channel == 6) {
    Serial.println("ERROR: Channel 6 is not configured (IMO line misconfigured)");
    return;
  }

  Serial.println();
  printInputStatus(channel);
  Serial.println();
}

void handleMonitorInputsCommand() {
  Serial.println("\n=== Monitoring Inputs (press any key to stop) ===\n");
  Serial.println("Digital Inputs:");
  Serial.println("  Clutch | AC_Press | DLC_Diag | Brake_NC | Brake_NO | Brake");
  Serial.println("Analog Inputs:");
  Serial.println("  Accel_1 | Accel_2 | Pedal% | Coolant | Oil");
  Serial.println("---");

  while (!Serial.available()) {
    // Read digital inputs
    L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
    bool clutch = (status.channel_states >> (INPUT_CLUTCH_SWITCH - 1)) & 0x01;
    bool ac_press = (status.channel_states >> (INPUT_AC_PRESSURE_SWITCH - 1)) & 0x01;
    bool dlc_diag = (status.channel_states >> (INPUT_DLC_DIAGNOSTIC - 1)) & 0x01;
    bool brake_nc = (status.channel_states >> (INPUT_BRAKE_SWITCH_NC - 1)) & 0x01;
    bool brake_no = (status.channel_states >> (INPUT_BRAKE_SWITCH_NO - 1)) & 0x01;
    bool brake = getBrakeSwitchState();

    // Read analog inputs
    AccelPedalReading pedal = getAccelPedalPosition();
    uint16_t coolant = readInputResistance(INPUT_ENGINE_COOLANT_TEMP);
    uint16_t oil = readInputResistance(INPUT_OIL_TEMP);

    // Print digital status
    Serial.printf("  %5s | %8s | %8s | %8s | %8s | %5s\n",
                  clutch ? "PRESS" : "REL",
                  ac_press ? "HIGH" : "LOW",
                  dlc_diag ? "NORM" : "DIAG",
                  brake_nc ? "CLOS" : "OPEN",
                  brake_no ? "CLOS" : "OPEN",
                  brake ? "PRESS" : "REL");

    // Print analog values
    Serial.printf("  %5.2fV | %5.2fV | %5.1f%% | %7u | %5u\n",
                  pedal.pedal1_voltage,
                  pedal.pedal2_voltage,
                  pedal.position_percent,
                  coolant,
                  oil);

    delay(200);  // Update 5 times per second
  }

  // Clear the character that stopped monitoring
  while (Serial.available()) Serial.read();

  Serial.println("\nMonitoring stopped.\n");
}

void printRelayStatus(const char* name, uint8_t device, OutputChannel channel) {
  // Note: We don't have direct output state reading from L9026
  // This would require tracking state in software or reading back PWM_SPI register
  Serial.printf("  %-12s (Dev %d, CH %d)\n", name, device, channel);
}

void handleReadAllOutputsCommand() {
  Serial.println("\n=== L9026 Output Status ===");
  Serial.println("\nNote: Output states shown are device/channel mappings.");
  Serial.println("Actual on/off state tracking requires software state management.\n");

  Serial.println("Device 0:");
  printRelayStatus("EFI Main 1", RELAY_EFI_MAIN_1_DEVICE, RELAY_EFI_MAIN_1_CHANNEL);
  printRelayStatus("EFI Main 23", RELAY_EFI_MAIN_23_DEVICE, RELAY_EFI_MAIN_23_CHANNEL);
  printRelayStatus("Starter", RELAY_STARTER_DEVICE, RELAY_STARTER_CHANNEL);
  printRelayStatus("Acc Cutoff", RELAY_ACC_CUTOFF_DEVICE, RELAY_ACC_CUTOFF_CHANNEL);
  printRelayStatus("ETCS", RELAY_ETCS_DEVICE, RELAY_ETCS_CHANNEL);
  printRelayStatus("ST Cut", RELAY_ST_CUT_DEVICE, RELAY_ST_CUT_CHANNEL);

  Serial.println("\nDevice 1:");
  printRelayStatus("Fan 3", RELAY_FAN3_DEVICE, RELAY_FAN3_CHANNEL);
  printRelayStatus("Fan 12", RELAY_FAN12_DEVICE, RELAY_FAN12_CHANNEL);
  printRelayStatus("AC Relay", RELAY_AC_DEVICE, RELAY_AC_CHANNEL);
  printRelayStatus("Blower", RELAY_BLOWER_DEVICE, RELAY_BLOWER_CHANNEL);
  printRelayStatus("Injector", RELAY_INJECTOR_DEVICE, RELAY_INJECTOR_CHANNEL);
  printRelayStatus("LS0", RELAY_LS0_DEVICE, RELAY_LS0_CHANNEL);
  Serial.println();
}

void handleSetOutputCommand(const char* args) {
  if (args == nullptr || strlen(args) == 0) {
    Serial.println("ERROR: Missing arguments. Usage: output <device> <channel> <state>");
    Serial.println("  device: 0-1, channel: 0-7, state: 0/1");
    return;
  }

  // Parse device, channel, and state
  char args_copy[64];
  strncpy(args_copy, args, sizeof(args_copy) - 1);
  args_copy[sizeof(args_copy) - 1] = '\0';

  char* token = strtok(args_copy, " ");
  if (token == nullptr) {
    Serial.println("ERROR: Missing device number");
    return;
  }
  int device = atoi(token);

  token = strtok(nullptr, " ");
  if (token == nullptr) {
    Serial.println("ERROR: Missing channel number");
    return;
  }
  int channel = atoi(token);

  token = strtok(nullptr, " ");
  if (token == nullptr) {
    Serial.println("ERROR: Missing state (0/1)");
    return;
  }
  int state = atoi(token);

  // Validate inputs
  if (device < 0 || device >= outputs.getDeviceCount()) {
    Serial.printf("ERROR: Device must be 0-%d\n", outputs.getDeviceCount() - 1);
    return;
  }

  if (channel < 0 || channel > 7) {
    Serial.println("ERROR: Channel must be 0-7");
    return;
  }

  if (state < 0 || state > 1) {
    Serial.println("ERROR: State must be 0 or 1");
    return;
  }

  // Set output
  outputs.setOutput(device, static_cast<OutputChannel>(channel), state);
  outputs.updateOutputs();

  Serial.printf("Set Device %d Channel %d to %s\n", device, channel, state ? "ON" : "OFF");
}

void handleRelayCommand(const char* args) {
  if (args == nullptr || strlen(args) == 0) {
    Serial.println("ERROR: Missing arguments. Usage: relay <name> <state>");
    Serial.println("  Available relays: efi1, efi23, starter, acc, etcs, stcut,");
    Serial.println("                    fan3, fan12, ac, blower, injector, ls0");
    return;
  }

  // Parse relay name and state
  char args_copy[64];
  strncpy(args_copy, args, sizeof(args_copy) - 1);
  args_copy[sizeof(args_copy) - 1] = '\0';

  char* name = strtok(args_copy, " ");
  char* state_str = strtok(nullptr, " ");

  if (name == nullptr || state_str == nullptr) {
    Serial.println("ERROR: Missing relay name or state");
    return;
  }

  // Parse state
  bool state = false;
  if (strcmp(state_str, "on") == 0 || strcmp(state_str, "1") == 0) {
    state = true;
  } else if (strcmp(state_str, "off") == 0 || strcmp(state_str, "0") == 0) {
    state = false;
  } else {
    Serial.println("ERROR: State must be 'on', 'off', '1', or '0'");
    return;
  }

  // Set relay based on name
  if (strcmp(name, "efi1") == 0) {
    setEFIMainRelay1(state);
    Serial.printf("EFI Main Relay 1: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "efi23") == 0) {
    setEFIMainRelay23(state);
    Serial.printf("EFI Main Relay 2/3: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "starter") == 0) {
    setStarterRelay(state);
    Serial.printf("Starter Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "acc") == 0) {
    setAccessoryCutoff(state);
    Serial.printf("Accessory Cutoff: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "etcs") == 0) {
    setETCSRelay(state);
    Serial.printf("ETCS Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "stcut") == 0) {
    setSTCutRelay(state);
    Serial.printf("ST Cut Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "fan3") == 0) {
    setFan3Relay(state);
    Serial.printf("Fan 3 Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "fan12") == 0) {
    setFan12Relay(state);
    Serial.printf("Fan 1/2 Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "ac") == 0) {
    setACRelay(state);
    Serial.printf("AC Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "blower") == 0) {
    setBlowerRelay(state);
    Serial.printf("Blower Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "injector") == 0) {
    setInjectorRelay(state);
    Serial.printf("Injector Relay: %s\n", state ? "ON" : "OFF");
  } else if (strcmp(name, "ls0") == 0) {
    setLS0Output(state);
    Serial.printf("LS0 Output: %s\n", state ? "ON" : "OFF");
  } else {
    Serial.printf("ERROR: Unknown relay name '%s'\n", name);
    Serial.println("  Available relays: efi1, efi23, starter, acc, etcs, stcut,");
    Serial.println("                    fan3, fan12, ac, blower, injector, ls0");
  }
}

void handleDiagnosticsCommand() {
  Serial.println("\n=== L9026 Diagnostics ===");

  // Read diagnostics from all devices
  outputs.readAllDeviceOffModeDiagnostics();

  for (uint8_t dev = 0; dev < outputs.getDeviceCount(); dev++) {
    Serial.printf("\nDevice %d:\n", dev);

    for (uint8_t ch = 0; ch <= 7; ch++) {
      L9026ChannelDiagnostics diag = outputs.getChannelDiagnostics(dev, static_cast<OutputChannel>(ch));

      Serial.printf("  CH%d: ", ch);
      bool has_fault = false;

      if (diag.overcurrent_overtemperature_detected) {
        Serial.print("OVERCURRENT/OVERTEMP ");
        has_fault = true;
      }
      if (diag.off_state_open_load_detected) {
        Serial.print("OPEN_LOAD(OFF) ");
        has_fault = true;
      }
      if (diag.on_state_open_load_detected) {
        Serial.print("OPEN_LOAD(ON) ");
        has_fault = true;
      }
      if (diag.shorted_load_detected) {
        Serial.print("SHORT ");
        has_fault = true;
      }

      if (!has_fault) {
        Serial.print("OK");
      }

      Serial.println();
    }
  }

  Serial.println();
}

void handleTestSequenceCommand() {
  Serial.println("\n=== Running Output Test Sequence ===");
  Serial.println("Testing each relay for 1 second...\n");

  // Test each relay in sequence
  struct RelayTest {
    const char* name;
    void (*set_func)(bool);
  };

  RelayTest relays[] = {
    {"EFI Main 1", setEFIMainRelay1},
    {"EFI Main 2/3", setEFIMainRelay23},
    {"Starter", setStarterRelay},
    {"Acc Cutoff", setAccessoryCutoff},
    {"ETCS", setETCSRelay},
    {"ST Cut", setSTCutRelay},
    {"Fan 3", setFan3Relay},
    {"Fan 1/2", setFan12Relay},
    {"AC", setACRelay},
    {"Blower", setBlowerRelay},
    {"Injector", setInjectorRelay},
    {"LS0", setLS0Output}
  };

  for (const auto& relay : relays) {
    Serial.printf("Testing %s... ", relay.name);
    relay.set_func(true);
    delay(1000);
    relay.set_func(false);
    Serial.println("done");
  }

  Serial.println("\nTest sequence complete!\n");
}

void parseCommand(const char* command) {
  // Make a copy to parse
  char cmd_copy[MAX_CMD_LENGTH];
  strncpy(cmd_copy, command, sizeof(cmd_copy) - 1);
  cmd_copy[sizeof(cmd_copy) - 1] = '\0';

  // Get the command name (first token)
  char* cmd_name = strtok(cmd_copy, " ");
  if (cmd_name == nullptr) {
    return;  // Empty command
  }

  // Get the arguments (rest of the string)
  const char* args = command + strlen(cmd_name);
  while (*args == ' ') args++;  // Skip leading spaces
  if (*args == '\0') args = nullptr;  // No arguments

  // Handle commands
  if (strcmp(cmd_name, "help") == 0) {
    handleHelpCommand();
  } else if (strcmp(cmd_name, "status") == 0) {
    handleStatusCommand();
  } else if (strcmp(cmd_name, "inputs") == 0) {
    handleReadAllInputsCommand();
  } else if (strcmp(cmd_name, "input") == 0) {
    handleReadInputCommand(args);
  } else if (strcmp(cmd_name, "monitor") == 0) {
    handleMonitorInputsCommand();
  } else if (strcmp(cmd_name, "outputs") == 0) {
    handleReadAllOutputsCommand();
  } else if (strcmp(cmd_name, "output") == 0) {
    handleSetOutputCommand(args);
  } else if (strcmp(cmd_name, "relay") == 0) {
    handleRelayCommand(args);
  } else if (strcmp(cmd_name, "diagnostics") == 0 || strcmp(cmd_name, "diag") == 0) {
    handleDiagnosticsCommand();
  } else if (strcmp(cmd_name, "test") == 0) {
    handleTestSequenceCommand();
  } else {
    Serial.printf("Unknown command: '%s'. Type 'help' for available commands.\n", cmd_name);
  }
}

void taskSerialConsole(void *pvParameters) {
  (void) pvParameters;

  // Wait a bit for system to stabilize
  vTaskDelay(pdMS_TO_TICKS(2000));

  Serial.println("\n\n=== VCU Serial Console Ready ===");
  Serial.println("Type 'help' for available commands");
  Serial.print("> ");

  for (;;) {
    // Check for incoming serial data
    while (Serial.available() > 0) {
      char c = Serial.read();

      // Echo character
      Serial.print(c);

      // Handle special characters
      if (c == '\n' || c == '\r') {
        // End of command
        Serial.println();
        cmd_buffer[cmd_index] = '\0';

        if (cmd_index > 0) {
          // Parse and execute command
          parseCommand(cmd_buffer);
        }

        // Reset buffer and print prompt
        cmd_index = 0;
        Serial.print("> ");
      } else if (c == '\b' || c == 127) {
        // Backspace
        if (cmd_index > 0) {
          cmd_index--;
          Serial.print(" \b");  // Erase character on terminal
        }
      } else if (cmd_index < MAX_CMD_LENGTH - 1) {
        // Add character to buffer
        cmd_buffer[cmd_index++] = c;
      }
    }

    // Small delay to prevent hogging CPU
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
