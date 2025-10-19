#include <Arduino.h>
#include "STM32_PWM.h"
#include "SPI.h"
#include "common.h"
#include "main.h"

#define Serial SerialUSB

#define IO_COPI PA7
#define IO_CIPO PA6
#define IO_SCK PB3
#define INPUT_CS PA9
#define OUTPUT_CS PA10
#define OUTPUT_STANDBY PB14
#define INPUT_INTERRUPT PB1
#define INPUT_RESET PB2
#define SSHUT_ENABLE PB0

#define BRZ_CAN_TERMINATION PB7
#define HV_CAN_TERMINATION PB12
#define M3_CAN_TERMINATION PB15

SPIClass spi(IO_COPI, IO_CIPO, IO_SCK, NC);
L9966 l9966(&spi, INPUT_CS, INPUT_INTERRUPT, INPUT_RESET, false, takeSPI, releaseSPI);

L9026Device outputDevices[] = {
  L9026Device(0, SSHUT_ENABLE, NC, OUTPUT_STANDBY),
  L9026Device(1, NC, NC, OUTPUT_STANDBY),
};

L9026 outputs(&spi, OUTPUT_CS, outputDevices, 2, takeSPI, releaseSPI);

// Relay control function implementations
void setEFIMainRelay1(bool state) {
  outputs.setOutput(RELAY_EFI_MAIN_1_DEVICE, RELAY_EFI_MAIN_1_CHANNEL, state);
}

void setEFIMainRelay23(bool state) {
  outputs.setOutput(RELAY_EFI_MAIN_23_DEVICE, RELAY_EFI_MAIN_23_CHANNEL, state);
}

void setStarterRelay(bool state) {
  outputs.setOutput(RELAY_STARTER_DEVICE, RELAY_STARTER_CHANNEL, state);
}

void setAccessoryCutoff(bool state) {
  outputs.setOutput(RELAY_ACC_CUTOFF_DEVICE, RELAY_ACC_CUTOFF_CHANNEL, state);
}

void setETCSRelay(bool state) {
  outputs.setOutput(RELAY_ETCS_DEVICE, RELAY_ETCS_CHANNEL, state);
}

void setSTCutRelay(bool state) {
  outputs.setOutput(RELAY_ST_CUT_DEVICE, RELAY_ST_CUT_CHANNEL, state);
}

void setFan3Relay(bool state) {
  outputs.setOutput(RELAY_FAN3_DEVICE, RELAY_FAN3_CHANNEL, state);
}

void setFan12Relay(bool state) {
  outputs.setOutput(RELAY_FAN12_DEVICE, RELAY_FAN12_CHANNEL, state);
}

void setACRelay(bool state) {
  outputs.setOutput(RELAY_AC_DEVICE, RELAY_AC_CHANNEL, state);
}

void setBlowerRelay(bool state) {
  outputs.setOutput(RELAY_BLOWER_DEVICE, RELAY_BLOWER_CHANNEL, state);
}

void setInjectorRelay(bool state) {
  outputs.setOutput(RELAY_INJECTOR_DEVICE, RELAY_INJECTOR_CHANNEL, state);
}

void setLS0Output(bool state) {
  outputs.setOutput(RELAY_LS0_DEVICE, RELAY_LS0_CHANNEL, state);
}

// Convenience functions
void setAllEFIRelays(bool state) {
  setEFIMainRelay1(state);
  setEFIMainRelay23(state);
}

void setAllFanRelays(bool state) {
  setFan3Relay(state);
  setFan12Relay(state);
}

// Input reading function implementations
uint16_t readInputVoltage(uint8_t channel) {
  l9966.startSingleConversion(static_cast<L9966_ADCChannel>(channel), true, L9966_PullupDivider::NO_PULLUP_5V);
  delay(10);  // Wait for conversion to complete
  L9966_ADCResult result = l9966.getSingleConversionResult();
  return result.value & 0x0FFF;  // 12-bit voltage value
}

uint16_t readInputResistance(uint8_t channel) {
  l9966.startSingleConversion(static_cast<L9966_ADCChannel>(channel), false, L9966_PullupDivider::NO_PULLUP_5V);
  delay(10);  // Wait for conversion to complete
  L9966_ADCResult result = l9966.getSingleConversionResult();
  return result.value & 0x7FFF;  // 15-bit resistance value
}

// Digital input functions
bool getCruiseControlState() {
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  return (status.channel_states >> (INPUT_CRUISE_CONTROL - 1)) & 0x01;
}

bool getClutchSwitchState() {
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  return (status.channel_states >> (INPUT_CLUTCH_SWITCH - 1)) & 0x01;
}

bool getACPressureSwitchState() {
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  return (status.channel_states >> (INPUT_AC_PRESSURE_SWITCH - 1)) & 0x01;
}

bool getDLCDiagnosticState() {
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  return (status.channel_states >> (INPUT_DLC_DIAGNOSTIC - 1)) & 0x01;
}

bool getBrakeSwitchState() {
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  bool nc_state = (status.channel_states >> (INPUT_BRAKE_SWITCH_NC - 1)) & 0x01;
  bool no_state = (status.channel_states >> (INPUT_BRAKE_SWITCH_NO - 1)) & 0x01;
  // Brake is pressed if NC is open (0) OR NO is closed (1)
  return (!nc_state) || no_state;
}

// Input event system (hardware interrupt-based)
static InputEventCallback input_callbacks[16] = {nullptr};  // Callbacks for channels 0-15
static uint16_t last_input_state = 0;

void registerInputCallback(uint8_t channel, InputEventCallback callback) {
  if (channel >= 1 && channel <= 15) {
    input_callbacks[channel] = callback;
  }
}

// Master interrupt handler - called from L9966 ISR when any watched channel changes
void onInputInterrupt(uint16_t wake_sources) {
  // Read current state of all inputs
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  uint16_t current_state = status.channel_states;

  // Detect which channels actually changed
  uint16_t changes = current_state ^ last_input_state;

  if (changes) {
    // Process each changed channel
    for (uint8_t ch = 1; ch <= 15; ch++) {
      uint16_t mask = 1 << (ch - 1);
      if (changes & mask) {
        // Channel changed - call callback if registered
        bool new_state = (current_state & mask) != 0;
        if (input_callbacks[ch]) {
          input_callbacks[ch](ch, new_state);
        }
      }
    }
    last_input_state = current_state;
  }
}

void initInputMonitoring() {
  // Read initial state for change detection
  L9966_DigitalInputStatus status = l9966.getDigitalInputStatus();
  last_input_state = status.channel_states;

  // Set up interrupt callback
  l9966.setInterruptCallback(onInputInterrupt);

  // Enable hardware interrupts for digital input channels
  // Bits set for channels we want to monitor: 1, 4, 5, 7, 10, 11
  uint16_t interrupt_mask = 0;
  interrupt_mask |= (1 << (INPUT_CRUISE_CONTROL - 1));     // Channel 1
  interrupt_mask |= (1 << (INPUT_CLUTCH_SWITCH - 1));      // Channel 4
  interrupt_mask |= (1 << (INPUT_AC_PRESSURE_SWITCH - 1)); // Channel 5
  interrupt_mask |= (1 << (INPUT_DLC_DIAGNOSTIC - 1));     // Channel 7
  interrupt_mask |= (1 << (INPUT_BRAKE_SWITCH_NC - 1));    // Channel 10
  interrupt_mask |= (1 << (INPUT_BRAKE_SWITCH_NO - 1));    // Channel 11

  l9966.enableInterrupts(interrupt_mask);
}

// Analog input functions
AccelPedalReading getAccelPedalPosition() {
  AccelPedalReading reading;

  // Read both pedal sensors
  reading.pedal1_raw = readInputVoltage(INPUT_ACCEL_PEDAL_1);
  reading.pedal2_raw = readInputVoltage(INPUT_ACCEL_PEDAL_2);

  // Convert to voltage (assuming 5V reference, 12-bit ADC)
  reading.pedal1_voltage = (reading.pedal1_raw / 4095.0f) * 5.0f;
  reading.pedal2_voltage = (reading.pedal2_raw / 4095.0f) * 5.0f;

  // Typical pedal sensors: VPA1 = 0.5-4.5V, VPA2 = 0.25-2.25V (half of VPA1)
  // Calculate percentage from VPA1 (0.5V = 0%, 4.5V = 100%)
  float pedal1_percent = ((reading.pedal1_voltage - 0.5f) / 4.0f) * 100.0f;
  float pedal2_percent = ((reading.pedal2_voltage - 0.25f) / 2.0f) * 100.0f;

  // Clamp to 0-100%
  pedal1_percent = constrain(pedal1_percent, 0.0f, 100.0f);
  pedal2_percent = constrain(pedal2_percent, 0.0f, 100.0f);

  // Validate: sensors should agree within 10%
  float difference = abs(pedal1_percent - pedal2_percent);
  reading.valid = (difference < 10.0f);

  // Use average if valid, otherwise use 0
  reading.position_percent = reading.valid ? ((pedal1_percent + pedal2_percent) / 2.0f) : 0.0f;

  return reading;
}

float getEngineCoolantTemp() {
  // Read resistance value
  uint16_t raw_resistance = readInputResistance(INPUT_ENGINE_COOLANT_TEMP);

  // TODO: Convert resistance to temperature using NTC thermistor lookup table
  // This is a placeholder - you'll need the actual thermistor curve for your sensor
  // For now, return raw value
  return raw_resistance;
}

float getOilTemp() {
  // Read resistance value
  uint16_t raw_resistance = readInputResistance(INPUT_OIL_TEMP);

  // TODO: Convert resistance to temperature using NTC thermistor lookup table
  // This is a placeholder - you'll need the actual thermistor curve for your sensor
  // For now, return raw value
  return raw_resistance;
}

// FreeRTOS Tasks
void taskAccelPedalMonitor(void *pvParameters) {
  (void) pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // Run every 20ms (50Hz)

  for (;;) {
    // Read accelerator pedal position
    AccelPedalReading pedal = getAccelPedalPosition();

    // Check if reading is valid
    if (pedal.valid) {
      // Store or use the pedal position
      // You can add logic here to update throttle, trigger events, etc.

      // Example: Log significant changes (optional)
      static float last_position = 0;
      if (abs(pedal.position_percent - last_position) > 5.0f) {
        Serial.print("Accel Pedal: ");
        Serial.print(pedal.position_percent, 1);
        Serial.println("%");
        last_position = pedal.position_percent;
      }
    } else {
      // Pedal sensors disagree - fault condition
      Serial.println("WARNING: Accelerator pedal sensor mismatch!");
      // Handle fault (e.g., set pedal to 0%, trigger limp mode)
    }

    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Example: Callback functions for input events
void onClutchChanged(uint8_t channel, bool state) {
  Serial.print("Clutch switch: ");
  Serial.println(state ? "PRESSED" : "RELEASED");

  // Example: Prevent starter engagement if clutch not pressed
  if (!state) {
    setStarterRelay(false);
  }
}

void onBrakeChanged(uint8_t channel, bool state) {
  Serial.print("Brake switch changed (channel ");
  Serial.print(channel);
  Serial.print("): ");
  Serial.println(state ? "HIGH" : "LOW");
}

void onDiagnosticModeChanged(uint8_t channel, bool state) {
  if (!state) {  // Grounded = diagnostic mode
    Serial.println("DIAGNOSTIC MODE ACTIVATED");
    // Enter diagnostic mode logic here
  } else {
    Serial.println("Normal mode");
  }
}

void setup() {
  Serial.begin(115200);
  spi.begin();

  delay(5000);
  Serial.println("Starting...");

  Serial.println("Setup L9026 output drivers");
  // setup L9026 output drivers
  if (!outputs.begin()) {
    Serial.println("Failed to initialize L9026 output drivers");
    while (1);
  }

  Serial.println("L9026 output drivers initialized - resetting devices");
  outputs.softReset();

  // configure L9026 output drivers
  /*
  Driver 0:
    SSHUT_ENABLE controls both IN0 and IN1 -> Channel 2 / 3 as low side outputs (tied together for SSHUT though probably not necessary for next iteration)
    HS0 -> Source 0
    HS1 -> Source 1
    Channel 4 -> Low side output for ST Starter Relay (STA)
    Channel 5 -> Low side output for Accessory Current Cutoff Request (ACC)
    Channel 6 -> Low side output for ETCS relay (MCR)
    Channel 7 -> Low side output for ST CUT Relay NC (STAR)
  Driver 1:
    IN0 is not connected
    IN1 is not connected
    Channel 0 -> High side output (HS2)
    Channel 1 -> High side output (HS3)
    Channel 2 -> Low side output for Fan #3 Relay (FAN1)
    Channel 3 -> Low side output for Fan #1,2 Relay (FAN2)
    Channel 4 -> Low side output for AC Compressor / Heater Relay (AC)
    Channel 5 -> Low side output for Blower Motor Relay (HB)
    Channel 6 -> Low side output for Injector Relay (IREL)
    Channel 7 -> Low side output (LS0)
  */

  Serial.println("Configuring L9026 output drivers");

  // Device 0 Configuration
  // Channels 0-1 are HIGH_SIDE by default (HS0, HS1)
  // Configure channels 2-7 as LOW_SIDE outputs
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_2, OutputType::LOW_SIDE);  // EFI Main Relay #1 (SSHUT)
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_3, OutputType::LOW_SIDE);  // EFI Main Relay #2,#3 (SSHUT)
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_4, OutputType::LOW_SIDE);  // ST Starter Relay (STA)
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_5, OutputType::LOW_SIDE);  // Accessory Current Cutoff Request (ACCR)
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_6, OutputType::LOW_SIDE);  // ETCS Relay (MCR)
  outputs.configureOutputSide(0, OutputChannel::CHANNEL_7, OutputType::LOW_SIDE);  // ST Cut Relay NC (STAR)

  // Device 1 Configuration
  // Channels 0-1 are HIGH_SIDE by default (HS2, HS3)
  // Configure channels 2-7 as LOW_SIDE outputs
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_2, OutputType::LOW_SIDE);  // Fan #3 Relay (FAN1)
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_3, OutputType::LOW_SIDE);  // Fan #1,#2 Relay (FAN2)
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_4, OutputType::LOW_SIDE);  // AC Compressor/Heater Relay (AC)
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_5, OutputType::LOW_SIDE);  // Blower Motor Relay (HB)
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_6, OutputType::LOW_SIDE);  // Injector Relay (IREL)
  outputs.configureOutputSide(1, OutputChannel::CHANNEL_7, OutputType::LOW_SIDE);  // Low side output (LS0)

  // Map SSHUT_ENABLE to control EFI Main Relays on Device 0
  Serial.println("Mapping inputs to output channels");
  outputs.mapInput(0, L9026PWMInput::IN0, OutputChannel::CHANNEL_2);  // EFI Main Relay #1
  outputs.mapInput(0, L9026PWMInput::IN1, OutputChannel::CHANNEL_3);  // EFI Main Relay #2,#3

  // Enable active mode for both devices
  Serial.println("Setting active mode for all devices");
  outputs.setActiveMode(0, true);
  outputs.setActiveMode(1, true);

  // Update configuration
  outputs.updateConfiguration();

  // read status of all devices
  Serial.println("Reading status of all devices");
  outputs.readAllDeviceStatus();

  // debug print device status for each device
  for (uint i = 0; i < outputs.getDeviceCount(); i++) {
    auto status = outputs.getDeviceStatus(i);
    Serial.printf("Device %d Disable Status: %d\n", i, status.disable_status);
    Serial.printf("Device %d Idle Status: %d\n", i, status.idle_status);
    Serial.printf("Device %d IN1 Status: %d\n", i, status.in1_status);
    Serial.printf("Device %d IN0 Status: %d\n", i, status.in0_status);
    Serial.printf("Device %d Power On Reset Condition Detected: %d\n", i, status.power_on_reset_condition_detected);
    Serial.printf("Device %d Overcurrent/Overtemperature Detected: %d\n", i, status.overcurrent_overtemperature_detected);
    Serial.printf("Device %d Off State Diagnostic Failure Detected: %d\n", i, status.off_state_diagnostic_failure_detected);
    Serial.printf("Device %d Operating Mode: %d\n", i, status.operating_mode);
  }

  // run off mode diagnostics
  Serial.println("Running off mode diagnostics");
  outputs.readAllDeviceOffModeDiagnostics();

  // debug print diagnostics for each device
  for (uint8_t i = 0; i < outputs.getDeviceCount(); i++) {
    for (uint8_t j = 2; j < 8; j++) {
      auto diagnostics = outputs.getChannelDiagnostics(i, (OutputChannel)j);
      Serial.printf("Device %d Channel %d Off State Open Load Detected: %d\n", i, j, diagnostics.off_state_open_load_detected);
      Serial.printf("Device %d Channel %d Shorted Load Detected: %d\n", i, j, diagnostics.shorted_load_detected);
    }
  }

  // setup L9966 input driver
  l9966.begin();

  // Configure input channels
  Serial.println("Configuring L9966 input channels...");

  // IO 1: Cruise Control Switch (analog resistor array)
  // Resistance measurement with 250µA current source
  L9966_CurrentSourceConfig io1_config = {
    .control_channel = 0,  // Force to 0 (always on)
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_250uA_PU_100uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLDOWN,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(1, io1_config);

  // IO 2: VPA1 Accelerator Pedal Position (voltage input 0-5V)
  L9966_CurrentSourceConfig io2_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_7_5uA_PU_1uA_VVAR_600uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::HIZ,  // High impedance for voltage measurement
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(2, io2_config);

  // IO 3: VPA2 Accelerator Pedal Position Signal 2 (voltage input 0-5V)
  L9966_CurrentSourceConfig io3_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_7_5uA_PU_1uA_VVAR_600uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::HIZ,  // High impedance for voltage measurement
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(3, io3_config);

  // IO 4: Clutch Switch (digital input with pull-up)
  L9966_CurrentSourceConfig io4_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(4, io4_config);

  // IO 5: AC Pressure Switch (digital input with pull-down)
  L9966_CurrentSourceConfig io5_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLDOWN,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(5, io5_config);

  // IO 6: Skip (misconfigured IMO line)

  // IO 7: DLC Diagnostic Mode (detects ground, needs pull-up)
  L9966_CurrentSourceConfig io7_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(7, io7_config);

  // IO 8: Engine Coolant Temperature Sensor (NTC thermistor)
  // Resistance measurement with pull-up to 5V reference
  L9966_CurrentSourceConfig io8_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_250uA_PU_100uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(8, io8_config);

  // IO 9: Oil Temperature Sensor (NTC thermistor)
  // Resistance measurement with pull-up to 5V reference
  L9966_CurrentSourceConfig io9_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_250uA_PU_100uA_PD,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(9, io9_config);

  // IO 10: Stop Light Switch NC (normally closed, pull-up)
  L9966_CurrentSourceConfig io10_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLUP_5V_REF,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(10, io10_config);

  // IO 11: Stop Light Switch NO (normally open, pull-down)
  L9966_CurrentSourceConfig io11_config = {
    .control_channel = 0,
    .threshold = L9966_ComparatorThreshold::UTH1,
    .current_value = L9966_CurrentValue::I_20uA,
    .dewetting_current = L9966_DewettingCurrent::DWT_USE_CV,
    .pull_mode = L9966_PullMode::PULLDOWN,
    .invert_control = false
  };
  l9966.setCurrentSourceConfig(11, io11_config);

  Serial.println("L9966 input channels configured");

  // Register input event callbacks
  Serial.println("Registering input event callbacks...");
  registerInputCallback(INPUT_CLUTCH_SWITCH, onClutchChanged);
  registerInputCallback(INPUT_BRAKE_SWITCH_NC, onBrakeChanged);
  registerInputCallback(INPUT_BRAKE_SWITCH_NO, onBrakeChanged);
  registerInputCallback(INPUT_DLC_DIAGNOSTIC, onDiagnosticModeChanged);

  // Initialize hardware interrupt-based input monitoring
  Serial.println("Initializing hardware interrupts for input monitoring...");
  initInputMonitoring();

  // Create FreeRTOS tasks
  Serial.println("Creating FreeRTOS tasks...");

  // Accelerator pedal monitor task - Priority 3 (safety critical)
  xTaskCreate(
    taskAccelPedalMonitor,  // Task function
    "AccelPedal",           // Task name
    2048,                   // Stack size (bytes)
    NULL,                   // Parameters
    3,                      // Priority (highest for safety-critical sensor)
    NULL                    // Task handle
  );

  Serial.println("FreeRTOS task created!");
  Serial.println("Starting scheduler...");

  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach here
  Serial.println("ERROR: Scheduler failed to start!");
  while(1);
}

void loop() {
  // FreeRTOS scheduler is running, this loop() should never execute
  // If we reach here, something went wrong with the scheduler
  Serial.println("ERROR: loop() should not be executing!");
  delay(1000);
}