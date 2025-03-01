#include "l9966.h"

void L9966::begin() {
  // attach input interrupt
  pinMode(interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt), []() {
    // do something
  }, FALLING);

  // reset device
  pinMode(reset, OUTPUT);
  digitalWrite(reset, LOW);
  delay(10);
  digitalWrite(reset, HIGH);
  delay(10);
  digitalWrite(reset, LOW);

  // delay 160 microseconds for default blanking time
  delayMicroseconds(160);

  if (whoami()) {
    SerialUSB.println("L9966 detected");

    // check general status
    auto gsr = getGeneralStatus();
    SerialUSB.print("Configuration reset: ");
    SerialUSB.println(gsr.configuration_reset);
    SerialUSB.print("Using calibrated ADC: ");
    SerialUSB.println(gsr.using_calibrated_adc);
    SerialUSB.print("Calibration fault: ");
    SerialUSB.println(gsr.calibration_fault);
    SerialUSB.print("Trim fault: ");
    SerialUSB.println(gsr.trim_fault);
    SerialUSB.print("Over temperature fault mask enabled: ");
    SerialUSB.println(gsr.over_temperature_fault_mask_enabed);
    SerialUSB.print("Entered wakeup event: ");
    SerialUSB.println(gsr.entered_wakeup_event);
    SerialUSB.print("Voltage supply fault: ");
    SerialUSB.println(gsr.voltage_supply_fault);
    SerialUSB.print("Over temperature fault: ");
    SerialUSB.println(gsr.over_temperature_fault);
  } else {
    SerialUSB.println("L9966 not detected");
  }
}

void L9966::packFrame(uint8_t address, uint16_t data, bool write, bool burst_mode, uint32_t &frame) {
  assert_param((write && data != NULL) || !write);
  frame = 0;
  
  // bit 31 = 1, bit 30 = CTRL_CFG, bit 29 = R/W, bit 28 = CLOCK_MONITORING (burst mode), bit 27 - 20 = address, bit 19 - 17: X, bit 16: odd parity bit for instruction
  // bit 15: odd parity bit for data, bit 14 - 0: data / ignored if a read
  frame |= 1 << 31;
  frame |= hardware_address_high << 30;
  frame |= write << 29;
  frame |= burst_mode << 28;
  frame |= address << 20;
  
  // calculate odd parity of bits 31 - 16 -> store in bit 16
  frame |= __builtin_parity(frame >> 16) << 16;

  // add data if write
  if (write) {
    frame |= data;
    frame |= __builtin_parity(frame) << 15;
  }
}

uint16_t L9966::transfer(uint8_t address, uint16_t data, bool write, bool burst_mode) {
  // take SPI interface
  takeSPI();

  // send command
  uint32_t tx, rx;
  packFrame(address, data, write, burst_mode, tx);

  // debug print frame
  Serial.print("L9966 tx frame: ");
  Serial.println(tx, HEX);

  // receive response
  spi->transfer(&tx, &rx, sizeof(uint32_t));

  // debug print rx frame
  Serial.print("L9966 rx frame: ");
  Serial.println(rx, HEX);

  // check rx for TRANS_F on bit 24
  if (rx & (1 << 24)) {
    Serial.println("L9966 transfer fault");
  }

  // check register address in blind echo (bits 23 - 16)
  if (((rx >> 16) & 0xFF) != address) {
    Serial.printf("L9966 address mismatch: %02X != %02X\n", (rx >> 16) & 0xFF, address);
  }

  // check odd parity of bits 15 - 0 (bit 16)
  if (__builtin_parity(rx) != ((rx >> 16) & 1)) {
    Serial.println("L9966 parity mismatch");
  }

  // return bits 15 - 0 of rx;
  auto response = rx & 0xFFFF;

  // release SPI interface
  releaseSPI();

  return response;
}


void L9966::takeSPI() {
  // acquire mutex
  xSemaphoreTake(spi_lock, portMAX_DELAY);

  // take SPI interface
  spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  delay(1);
}

void L9966::releaseSPI() {
  // release SPI interface
  delay(1);
  spi->endTransaction();
  digitalWrite(cs, HIGH);
  
  // release mutex
  xSemaphoreGive(spi_lock);
}

bool L9966::whoami() {
  auto response = transfer(L9966_DEV_ID_REG, NULL, false, false);

  SerialUSB.print("L9966 WHOAMI: ");
  SerialUSB.println(response, HEX);

  // print response
  return response == 0x5A;
}

L9966_GSR L9966::getGeneralStatus() {
  auto response = transfer(L9966_GEN_STATUS_REG, NULL, false, false);
  L9966_GSR gsr;

  // bits 14-13: configuration reset if equal to b10
  // bit 6: using calibrated ADC
  // bit 5: calibration fault
  // bit 4: trim fault
  // bit 3: over temperature fault mask enabled
  // bit 2: entered wakeup event
  // bit 1: voltage supply fault
  // bit 0: over temperature fault
  gsr.configuration_reset = (response & (1 << 14)) && !(response & (1 << 13));
  gsr.using_calibrated_adc = response & (1 << 6);
  gsr.calibration_fault = response & (1 << 5);
  gsr.trim_fault = response & (1 << 4);
  gsr.over_temperature_fault_mask_enabed = response & (1 << 3);
  gsr.entered_wakeup_event = response & (1 << 2);
  gsr.voltage_supply_fault = response & (1 << 1);
  gsr.over_temperature_fault = response & (1 << 0);

  return gsr;
}