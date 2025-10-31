#pragma once

#include "Arduino.h"
#include "l9026.h"
#include "l9966.h"
#include "hal/can/can.h"

// Relay output definitions
// Device 0 outputs
#define RELAY_EFI_MAIN_1_DEVICE     0
#define RELAY_EFI_MAIN_1_CHANNEL    OutputChannel::CHANNEL_2

#define RELAY_EFI_MAIN_23_DEVICE    0
#define RELAY_EFI_MAIN_23_CHANNEL   OutputChannel::CHANNEL_3

#define RELAY_STARTER_DEVICE        0
#define RELAY_STARTER_CHANNEL       OutputChannel::CHANNEL_4

#define RELAY_ACC_CUTOFF_DEVICE     0
#define RELAY_ACC_CUTOFF_CHANNEL    OutputChannel::CHANNEL_5

#define RELAY_ETCS_DEVICE           0
#define RELAY_ETCS_CHANNEL          OutputChannel::CHANNEL_6

#define RELAY_ST_CUT_DEVICE         0
#define RELAY_ST_CUT_CHANNEL        OutputChannel::CHANNEL_7

// Device 1 outputs
#define RELAY_FAN3_DEVICE           1
#define RELAY_FAN3_CHANNEL          OutputChannel::CHANNEL_2

#define RELAY_FAN12_DEVICE          1
#define RELAY_FAN12_CHANNEL         OutputChannel::CHANNEL_3

#define RELAY_AC_DEVICE             1
#define RELAY_AC_CHANNEL            OutputChannel::CHANNEL_4

#define RELAY_BLOWER_DEVICE         1
#define RELAY_BLOWER_CHANNEL        OutputChannel::CHANNEL_5

#define RELAY_INJECTOR_DEVICE       1
#define RELAY_INJECTOR_CHANNEL      OutputChannel::CHANNEL_6

#define RELAY_LS0_DEVICE            1
#define RELAY_LS0_CHANNEL           OutputChannel::CHANNEL_7

// Input channel definitions
#define INPUT_CRUISE_CONTROL        1
#define INPUT_ACCEL_PEDAL_1         2
#define INPUT_ACCEL_PEDAL_2         3
#define INPUT_CLUTCH_SWITCH         4
#define INPUT_AC_PRESSURE_SWITCH    5
// INPUT 6 is skipped (IMO line misconfigured)
#define INPUT_DLC_DIAGNOSTIC        7
#define INPUT_ENGINE_COOLANT_TEMP   8
#define INPUT_OIL_TEMP              9
#define INPUT_BRAKE_SWITCH_NC       10
#define INPUT_BRAKE_SWITCH_NO       11

// External references
extern L9026 outputs;
extern L9966 l9966;

// Input event system (hardware interrupt-based)
typedef void (*InputEventCallback)(uint8_t channel, bool state);

void initInputMonitoring();  // Initialize input monitoring with hardware interrupts
void registerInputCallback(uint8_t channel, InputEventCallback callback);

// FreeRTOS tasks
void taskAccelPedalMonitor(void *pvParameters);
void taskM3CanMonitor(void *pvParameters);
void taskHVCanMonitor(void *pvParameters);
void taskBRZCanMonitor(void *pvParameters);

// Relay control functions
void setEFIMainRelay1(bool state);
void setEFIMainRelay23(bool state);
void setStarterRelay(bool state);
void setAccessoryCutoff(bool state);
void setETCSRelay(bool state);
void setSTCutRelay(bool state);
void setFan3Relay(bool state);
void setFan12Relay(bool state);
void setACRelay(bool state);
void setBlowerRelay(bool state);
void setInjectorRelay(bool state);
void setLS0Output(bool state);

// Convenience functions for common operations
void setAllEFIRelays(bool state);  // Controls all EFI main relays
void setAllFanRelays(bool state);  // Controls both fan relays

// Input reading functions
struct AccelPedalReading {
  uint16_t pedal1_raw;
  uint16_t pedal2_raw;
  float pedal1_voltage;
  float pedal2_voltage;
  bool valid;  // True if both sensors are in agreement
  float position_percent;  // 0-100% pedal position
};

// Digital inputs
bool getCruiseControlState();
bool getClutchSwitchState();
bool getACPressureSwitchState();
bool getDLCDiagnosticState();
bool getBrakeSwitchState();  // Combined from NC and NO for redundancy

// Analog inputs
AccelPedalReading getAccelPedalPosition();  // Reads and validates both pedal sensors
float getEngineCoolantTemp();  // Returns temperature in Celsius
float getOilTemp();  // Returns temperature in Celsius

// Raw ADC reading functions
uint16_t readInputVoltage(uint8_t channel);  // Returns raw ADC value for voltage mode
uint16_t readInputResistance(uint8_t channel);  // Returns raw ADC value for resistance mode

