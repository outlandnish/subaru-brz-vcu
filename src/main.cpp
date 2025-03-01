#include <Arduino.h>
#include "STM32_PWM.h"
#include "STM32_CAN.h"
#include "SPI.h"
#include "drivers/l9966.h"
#include "l9026.h"

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
// L9966 l9966(&spi, INPUT_CS, INPUT_INTERRUPT, INPUT_RESET, false);

L9026Device outputDevices[] = {
  {SSHUT_ENABLE, NC, OUTPUT_STANDBY, { 0, 0 }, { 0, 0 }, 0, NULL }, 
  {NC, NC, NC, { 0, 0 }, { 0, 0 }, 0, NULL }
};

L9026 outputs(&spi, OUTPUT_CS, outputDevices, 2, takeSPI, releaseSPI);

void setup() {
  Serial.begin(115200);
  spi.begin();

  delay(2000);
  Serial.println("Starting...");
  
  // setup L9966 input driver
  // l9966.begin();

  // setup L9026 output drivers
  // if (!outputs.begin()) {
  //   Serial.println("Failed to initialize L9026 output drivers");
  //   while (1);
  // }
  
  // Serial.println("L9026 output drivers initialized - resetting devices");
  // outputs.softReset();

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

  // Serial.println("Configuring L9026 output drivers");

  // // configure output side for all channels (channels 0 and 1 can't be changed)
  // for (uint i = 0; i < 2; i++) {
  //   for (uint j = 2; j < 8; j++) {
  //     outputs.configureOutputSide(i, (OutputChannel)j, OutputType::LOW_SIDE);
  //   }
  // }

  // // map input 0 to channel 2 and input 1 to channel 3 for driver 0
  // Serial.println("Mapping inputs to output channels");
  // outputs.mapInput(0, L9026PWMInput::IN0, OutputChannel::CHANNEL_2);
  // outputs.mapInput(0, L9026PWMInput::IN1, OutputChannel::CHANNEL_3);

  // // put device into active mode
  // Serial.println("Setting active mode for all devices");
  // outputs.setActiveMode(0, true);

  // // update configuration
  // outputs.updateConfiguration();
  
  // // read status of all devices
  // Serial.println("Reading status of all devices");
  // outputs.readAllDeviceStatus();
  
  // // debug print device status for each device
  // for (uint i = 0; i < 2; i++) {
  //   auto status = outputs.getDeviceStatus(i);
  //   Serial.printf("Device %d Disable Status: %d\n", i, status.disable_status);
  //   Serial.printf("Device %d Idle Status: %d\n", i, status.idle_status);
  //   Serial.printf("Device %d IN1 Status: %d\n", i, status.in1_status);
  //   Serial.printf("Device %d IN0 Status: %d\n", i, status.in0_status);
  //   Serial.printf("Device %d Power On Reset Condition Detected: %d\n", i, status.power_on_reset_condition_detected);
  //   Serial.printf("Device %d Overcurrent/Overtemperature Detected: %d\n", i, status.overcurrent_overtemperature_detected);
  //   Serial.printf("Device %d Off State Diagnostic Failure Detected: %d\n", i, status.off_state_diagnostic_failure_detected);
  //   Serial.printf("Device %d Operating Mode: %d\n", i, status.operating_mode);
  // }

  // // run off mode diagnostics
  // Serial.println("Running off mode diagnostics");
  // outputs.readAllDeviceOffModeDiagnostics();
  
  // // debug print diagnostics for each device
  // for (uint8_t i = 0; i < 2; i++) {
  //   for (uint8_t j = 2; j < 8; j++) {
  //     auto diagnostics = outputs.getChannelDiagnostics(i, (OutputChannel)j);
  //     Serial.printf("Device %d Channel %d Off State Open Load Detected: %d\n", i, j, diagnostics.off_state_open_load_detected);
  //     Serial.printf("Device %d Channel %d Shorted Load Detected: %d\n", i, j, diagnostics.shorted_load_detected);
  //   }
  // }
  
  // setup CAN buses
  
}

void loop() {
  // digitalWrite(INPUT_RESET, HIGH);
  // delay(1000);
  // digitalWrite(INPUT_RESET, LOW);
  // delay(1000);
  // Serial.println("Hello World");
}