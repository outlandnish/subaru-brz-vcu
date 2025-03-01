#include "l9026.h"

bool L9026::begin() {
  // setup chip select first
  pinMode(cs, OUTPUT);

  digitalWrite(cs, HIGH);
  delay(1);

  for (uint8_t i = 0; i < device_count; i++) {
    // setup in0, in1 if specified
    if (devices[i].in0 != NC) {
      pinMode(devices[i].in0, OUTPUT);
      digitalWrite(devices[i].in0, LOW);
    }
    
    if (devices[i].in1 != NC) {
      pinMode(devices[i].in1, OUTPUT);
      digitalWrite(devices[i].in1, LOW);
    }

    // setup idle if specified and pull high
    if (devices[i].idle != NC) {
      pinMode(devices[i].idle, OUTPUT);
      digitalWrite(devices[i].idle, HIGH);
    }

    // update channel map with channel 0 and 1 configured as high side (these can't be changed)
    devices[i].output_side[OutputChannel::CHANNEL_0] = OutputType::HIGH_SIDE;
    devices[i].output_side[OutputChannel::CHANNEL_1] = OutputType::HIGH_SIDE;

    // setup channel diagnostics
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++)
      devices[i].diagnostics[(OutputChannel)j] = L9026ChannelDiagnostics();
  } 

  // size of each transfer
  buffer_size = device_count * sizeof(uint16_t);
  tx_buffer = new uint8_t[buffer_size];
  rx_buffer = new uint8_t[buffer_size];

  // get chip id for each device
  return getChipID();
}

void L9026::end() {
  delete[] tx_buffer;
  delete[] rx_buffer;
}

void L9026::packFrame(TransactionData &transaction, uint8_t *buffer) {
  // debug print transaction
  // Serial.printf("Address: %02X\n", transaction.address);
  // Serial.printf("Data: %02X\n", transaction.data);
  // Serial.printf("Write: %d\n", transaction.write);
  // Serial.printf("Frame Counter: %d\n", transaction.frame_counter);
  // Serial.printf("Error: %d\n", transaction.error);

  uint16_t frame;
  frame = 0;

  // bit 15 is r/w flag
  frame |= (transaction.write << 15);

  // limit bits 14 - 10 to address
  frame |= (transaction.address & 0x1F) << 10;

  // mem copy data into bits 2 - 9
  if (transaction.write)
    frame |= (transaction.data & 0xFF) << 2;

  // bit 1 is odd parity (calculated from bits 2 - 15)
  frame |= __builtin_parity(frame) << 1;

  // bit 0 is frame counter
  frame |= transaction.frame_counter;

  // swap frame bytes
  frame = __builtin_bswap16(frame);

  // copy frame into buffer
  memcpy(buffer, &frame, sizeof(uint16_t));

  // debug print out bits 15 - 2 and parity bit
  Serial.printf("Packed Frame: %02X %02X Parity: %d\n", buffer[0], buffer[1], frame & 1);
}

void L9026::unpackFrame(TransactionData &transaction, uint8_t *buffer) {
  uint16_t frame;
  memcpy(&frame, buffer, sizeof(uint16_t));

  // Convert from big endian to little endian
  frame = __builtin_bswap16(frame);

  // Serial.printf("Frame: %04X\n", frame);
  // extract address from bits 14 - 10
  transaction.address = (frame >> 10) & 0x1F;

  // extract data from bits 2 - 9
  transaction.data = (frame >> 2) & 0xFF;

  // extract error flag from bit 15
  transaction.last_frame_error = (frame >> 15) & 1;

  // extract frame counter from bit 0
  transaction.frame_counter = frame & 1;

  // debug print transaction
  // Serial.printf("Address: %02X\n", transaction.address);
  // Serial.printf("Data: %02X\n", transaction.data);
  // Serial.printf("Error: %d\n", transaction.error);
  // Serial.printf("Frame Counter: %d\n", transaction.frame_counter);
}

void L9026::buildTransferBuffer(uint8_t *buffer) {
  assert_param(buffer != NULL);

  for (uint8_t i = 0; i < device_count; i++) {
    // reset memory
    memset(&devices[i].tx_buffer, 0, sizeof(uint16_t));
    packFrame(devices[i].transaction, devices[i].tx_buffer);
    memcpy(&buffer[DEVICE_BUFFER_OFFSET(device_count, i)], &devices[i].tx_buffer, sizeof(uint16_t));

    // debug print bytes of tx frame
    // Serial.printf("Device %d TX Frame: %02X %02X\n", i, devices[i].tx_buffer[0], devices[i].tx_buffer[1]);
  }

  // debug print buffer
  // Serial.printf("Packed buffer: %02X %02X %02X %02X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
}

void L9026::unpackTransferBuffer(uint8_t *buffer) {
  assert_param(buffer != NULL);

  // debug print buffer
  Serial.printf("Unpack buffer: %02X %02X %02X %02X\n", buffer[0], buffer[1], buffer[2], buffer[3]);

  for (uint8_t i = 0; i < device_count; i++) {
    // reset memory
    memset(&devices[i].rx_buffer, 0, sizeof(uint16_t));

    // last device should be at MSB, first device at LSB
    memcpy(&devices[i].rx_buffer, &buffer[DEVICE_BUFFER_OFFSET(device_count, i)], sizeof(uint16_t));
    unpackFrame(devices[i].transaction, devices[i].rx_buffer);

    if (devices[i].transaction.last_frame_error) {
      Serial.printf("Previous frame error on device %d\n", i);
    }

    // debug print bytes of rx frame
    // Serial.printf("Device %d RX Frame: %02X %02X\n", i, devices[i].rx_buffer[0], devices[i].rx_buffer[1]);
  }
}

void L9026::transfer(bool get_response) {
  // clear buffers
  memset(tx_buffer, 0, buffer_size);
  memset(rx_buffer, 0, buffer_size);

  // take SPI interface
  take_spi();

  // start SPI transaction
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // we need to duplicate the transfer to read data
  if (get_response) {
    // build transfer buffer
    buildTransferBuffer(tx_buffer);

    digitalWrite(cs, LOW);
    delay(1);

    Serial.printf("Frame counters: %d %d\n", devices[0].transaction.frame_counter, devices[1].transaction.frame_counter);
    spi->transfer(tx_buffer, rx_buffer, buffer_size);
    digitalWrite(cs, HIGH);
    delay(1);

    unpackTransferBuffer(rx_buffer);

    // flip frame counters
    for (uint8_t i = 0; i < device_count; i++) {
      devices[i].transaction.frame_counter = devices[i].transaction.frame_counter ? 0 : 1;
    }
  }

  // build transfer buffer
  buildTransferBuffer(tx_buffer);

  digitalWrite(cs, LOW);
  delay(1);

  Serial.printf("Frame counters: %d %d\n", devices[0].transaction.frame_counter, devices[1].transaction.frame_counter);
  spi->transfer(tx_buffer, rx_buffer, buffer_size);
  digitalWrite(cs, HIGH);
  
  spi->endTransaction();
  delay(1);

  // release SPI interface
  release_spi(); 

  // unpack transfer buffer
  unpackTransferBuffer(rx_buffer);

  // flip frame counters
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].transaction.frame_counter = devices[i].transaction.frame_counter ? 0 : 1;
  }
}

void L9026::prepareRead(uint8_t device_id, uint8_t address) {
  devices[device_id].transaction.address = address;
  devices[device_id].transaction.write = false;
  devices[device_id].transaction.data = 0;
  devices[device_id].transaction.last_frame_error = 0;

  // debug print transaction
  // Serial.printf("Read Transaction: Device %d Address: %02X Frame Counter: %d\n", device_id, devices[device_id].transaction.address, devices[device_id].transaction.frame_counter);
}

void L9026::prepareWrite(uint8_t device_id, uint8_t address, uint8_t data) {
  devices[device_id].transaction.address = address;
  devices[device_id].transaction.write = true;
  devices[device_id].transaction.data = data;
  devices[device_id].transaction.last_frame_error = 0;

  // debug print transaction
  // Serial.printf("Write Transaction: Device %d Address: %02X Frame Counter: %d\n", device_id, devices[device_id].transaction.address, devices[device_id].transaction.frame_counter);
  // Serial.printf("Write Transaction - Device %d Data: %02X\n", device_id, devices[device_id].transaction.data);
}

bool L9026::getChipID() {
  Serial.println("Getting Chip ID");
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, CHIP_ID_REG);
  }

  transfer(true);

  // set device id
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].id = devices[i].transaction.data;
    Serial.printf("Device %d ID: %02X\n", i, devices[i].id);
    if (devices[i].id == 0)
      return false;
  }

  return true;
}

// Channel [7:2], MSB refers to channel 7. Bit values: 0: (default) low side 1: high side
void L9026::configureOutputSide(uint8_t device_id, OutputChannel channel, OutputType type) {
  assert_param(device_id < device_count);
  assert_param(channel >= 2 && channel <= MAX_OUTPUT_CHANNEL);

  // bit offset in the mask is 2
  devices[device_id].registers[CFG0_REG] |= (type << channel);

  // store in map
  devices[device_id].output_side[channel] = type;
}

void L9026::configureDisableResetPins(uint8_t device_id, bool disable_enabled, bool reset_enabled) {
  assert_param(device_id < device_count);

  if (disable_enabled) {
    devices[device_id].registers[CFG0_REG] |= 1 << 1;
  }
  else {
    devices[device_id].registers[CFG0_REG] &= ~(1 << 1);
  }

  if (reset_enabled) {
    devices[device_id].registers[CFG0_REG] |= 1;
  }
  else {
    devices[device_id].registers[CFG0_REG] &= ~1;
  }
}

// todo: check if you can configure multiple PWM outputs simultaneously
void L9026::configureLEDPWM(uint8_t device_id, L9026LEDPWMConfiguration *config) {
  assert_param(device_id < device_count);

  // set frequency in the CFG_2 register
  devices[device_id].registers[CFG1_REG] |= config->frequency;

  // clear last output channel from the MAP_PWM register if it exists
  if (devices[device_id].led_pwm_config != NULL) {
    devices[device_id].registers[MAP_PWM_REG] &= ~(1 << devices[device_id].led_pwm_config->channel);
  }

  // set new output channel in the MAP_PWM register
  devices[device_id].registers[MAP_PWM_REG] |= (1 << config->channel);

  // set new output channel in the PWM_SEL register (0 = PWM_GEN, 1 = PWM_LED)
  devices[device_id].registers[PWM_SEL_REG] |= (1 << config->channel);

  // store new configuration to unset it next time
  devices[device_id].led_pwm_config = config;
}

// todo: check if you can configure multiple PWM outputs simultaneously
void L9026::configureGeneralPWM(uint8_t device_id, L9026GeneralPWMConfiguration *config) {
  assert_param(device_id < device_count);

  // set frequency and adjustment in the CFG_2 register (adjustment at bit offset 2)
  devices[device_id].registers[CFG2_REG] |= config->frequency;
  devices[device_id].registers[CFG2_REG] |= (config->frequency_adjustment << 2);

  // clear last output channel from the MAP_PWM register if it exists
  if (devices[device_id].general_pwm_config != NULL) {
    devices[device_id].registers[MAP_PWM_REG] &= ~(1 << devices[device_id].general_pwm_config->channel);
  }
  
  // set new output channel in the MAP_PWM register
  devices[device_id].registers[MAP_PWM_REG] |= (1 << config->channel);

  // set new output channel in the PWM_SEL register (0 = PWM_GEN, 1 = PWM_LED)
  devices[device_id].registers[PWM_SEL_REG] &= ~(1 << config->channel);

  // store new configuration to unset it next time
  devices[device_id].general_pwm_config = config;
}

// Channel [7:0], MSB refers to channel 7. Bit values: 0: (default) no BIM active 1: BIM active
void L9026::configureBulbInrushMode(uint8_t device_id, OutputChannel channel, bool enable) {
  assert_param(device_id < device_count);

  // bit offset in the mask is 0
  devices[device_id].registers[BIM_REG] |= (enable << channel);
}

void L9026::mapInput(uint8_t device_id, L9026PWMInput input, OutputChannel channel) {
  assert_param(channel >= 0 && channel <= MAX_OUTPUT_CHANNEL);

  switch (input) {
    case L9026PWMInput::IN0:
      // clear last output channel from the MAP_IN0 register if it exists
      devices[device_id].registers[MAP_IN0_REG] = 0;
      devices[device_id].registers[MAP_IN0_REG] |= (1 << channel);
      break;
    case L9026PWMInput::IN1:
      // clear last output channel from the MAP_IN1 register if it exists
      devices[device_id].registers[MAP_IN1_REG] = 0;
      devices[device_id].registers[MAP_IN1_REG] |= (1 << channel);
      break;
  }
}

void L9026::setGeneralPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle) {
  assert_param(device_id < device_count);

  devices[device_id].registers[PWM_GEN_DC_REG] = duty_cycle;
}

void L9026::setLEDPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle) {
  assert_param(device_id < device_count);

  devices[device_id].registers[PWM_LED_DC_REG] = duty_cycle;
}

void L9026::setOutput(uint8_t device_id, OutputChannel channel, bool enable) {
  assert_param(device_id < device_count);
  assert_param(channel >= 0 && channel <= MAX_OUTPUT_CHANNEL);

  // bit offset in the mask is 0
  devices[device_id].registers[PWM_SPI_REG] |= (enable << channel);
}

void L9026::setActiveMode(uint8_t device_id, bool enable) {
  assert_param(device_id < device_count);

  if (enable) {
    devices[device_id].registers[CFG1_REG] |= ENABLE_ACTIVE_MODE;
  }
  else {
    devices[device_id].registers[CFG1_REG] &= ~ENABLE_ACTIVE_MODE;
  }
}

void L9026::updateConfiguration() {
  // update all configuration registers for all devices
  for (auto i = CFG0_REG; i < DIAG_OFF_EN_REG; i++) {
    if (i != RESERVED_REG) {
      for (uint8_t j = 0; j < device_count; j++) {
        prepareWrite(j, i, devices[j].registers[i]);
      }
      transfer();
      delay(1);
    }
  }
}

void L9026::updateOutputs() {
  // updates outputs + duty cycle registers
  for (uint8_t i = 0; i < device_count; i++) {
    prepareWrite(i, PWM_SPI_REG, devices[i].registers[PWM_SPI_REG]);
    transfer();
    delay(1);

    prepareWrite(i, PWM_GEN_DC_REG, devices[i].registers[PWM_GEN_DC_REG]);
    transfer();
    delay(1);

    prepareWrite(i, PWM_LED_DC_REG, devices[i].registers[PWM_LED_DC_REG]);
    delay(1);
  }
}

void L9026::softReset() {
  for (uint8_t i = 0; i < device_count; i++) {
    prepareWrite(i, CFG1_REG, RESET);
  }

  transfer();
  delay(1);

  // reset frame counters
  // for (uint8_t i = 0; i < device_count; i++) {
  //   devices[i].transaction.frame_counter = 0;
  // }

  // reset status + diagnostics
  for (uint8_t i = 0; i < device_count; i++) {
    devices[i].status = L9026DeviceStatus();
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      devices[i].diagnostics[(OutputChannel)j] = L9026ChannelDiagnostics();
    }
  }
}

void L9026::toggleLimpHomeMode(bool inLimpHomeMode) {
  // todo: decide what to do here
  // limp mode is where idle state is low and either in0 or in1 is high
  // IN0 controls Channel 2 and IN1 controls Channel 3 in limp home mode (overriding any other configuration)  
}

void L9026::toggleSleepMode(bool inSleepMode) {
  if (inSleepMode) {
    // set all devices to sleep mode
    for (uint8_t i = 0; i < device_count; i++) {
      digitalWrite(devices[i].idle, LOW);
      digitalWrite(devices[i].in0, LOW);
      digitalWrite(devices[i].in1, LOW);
    }
  }
  else {
    // set idle high
    for (uint8_t i = 0; i < device_count; i++) {
      digitalWrite(devices[i].idle, HIGH);
    }
  }
}

void L9026::readAllDeviceStatus() {
  // read STA_0
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, STA_0_REG);
  }
  transfer(true);
  
  for (uint8_t i = 0; i < device_count; i++) {
    Serial.printf("Device %d Status: %02X\n", i, devices[i].transaction.data);
    // bit field 6 is disable status
    devices[i].status.disable_status = (devices[i].transaction.data >> 6) & 1;
    
    // bit field 4 is idle status
    devices[i].status.idle_status = (devices[i].transaction.data >> 4) & 1;

    // bit field 3 is IN1 status
    devices[i].status.in1_status = (devices[i].transaction.data >> 3) & 1;

    // bit field 2 is IN0 status
    devices[i].status.in0_status = (devices[i].transaction.data >> 2) & 1;

    // bit field 1 is overcurrent/overtemperature detected
    devices[i].status.overcurrent_overtemperature_detected = (devices[i].transaction.data >> 1) & 1;

    // bit field 0 is off state diagnostic failure detected
    devices[i].status.off_state_diagnostic_failure_detected = devices[i].transaction.data & 1;
  }

  // read STA_1
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, STA_1_REG);
  }
  transfer(true);

  for (uint8_t i = 0; i < device_count; i++) {
    // bit field 4 is power on reset condition detected
    devices[i].status.power_on_reset_condition_detected = (devices[i].transaction.data >> 4) & 1;

    // bit field 3 is VDDIO undervoltage detected
    devices[i].status.vddio_undervoltage_detected = (devices[i].transaction.data >> 3) & 1;

    // bit field 2 is VBATT undervoltage detected
    devices[i].status.vbatt_undervoltage_detected = (devices[i].transaction.data >> 2) & 1;

    // bit field 1-0 is operating mode
    devices[i].status.operating_mode = (L9026OperatingMode)(devices[i].transaction.data & 0x3);
  }
}

L9026DeviceStatus L9026::getDeviceStatus(uint8_t device_id) {
  return devices[device_id].status;
}

void L9026::readAllDeviceOffModeDiagnostics() {
  // write off mode diagnostics enable
  for (uint8_t i = 0; i < device_count; i++) {
    // enable diagnostics on all channels
    prepareWrite(i, DIAG_OFF_EN_REG, 0xFF);
  }
  transfer();

  // wait at least 1.6ms
  delayMicroseconds(1600);

  // read off mode open load detection
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, DIAG_OPL_OFF_REG);
  }
  transfer(true);

  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no open load detected 1: open load detected
  for (uint8_t i = 0; i < device_count; i++) {
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      
      
      // get device channel diagnostics reference
      devices[i].diagnostics[(OutputChannel)j].off_state_open_load_detected = (devices[i].transaction.data >> j) & 1;
    }
  }

  // read off mode short detection
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, DIAG_SHG_REG);
  }
  transfer(true);

  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no short detected 1: short detected
  for (uint8_t i = 0; i < device_count; i++) {
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      // get device channel diagnostics reference
      devices[i].diagnostics[(OutputChannel)j].shorted_load_detected = (devices[i].transaction.data >> j) & 1;
    }
  }
}

// note: blocking. this can take up to a minimum of 210ms x 8 HS channels = 1.68s to complete (run in a separate thread)
// todo: make this non-blocking
void L9026::readAllDeviceOnModeDiagnostics() {
  // read overcurrent/overtemperature detection
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, DIAG_OVC_OVT_REG);
  }
  transfer(true);

  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no overcurrent/overtemperature detected 1: overcurrent/overtemperature detected
  for (uint8_t i = 0; i < device_count; i++) {
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      // get device channel diagnostics reference
      devices[i].diagnostics[(OutputChannel)j].overcurrent_overtemperature_detected = (devices[i].transaction.data >> j) & 1;
    }
  }

  // clear overcurrent/overtemperature error register
  for (uint8_t i = 0; i < device_count; i++) {
    // clear all channels
    prepareWrite(i, DIAG_OVC_OVT_RLW_REG, 0xFF);
  }
  transfer();

  // write on mode diagnostics enable for any channel configured as high side (channel 0 and 1 are always high side)
  // The channel must not be configured for LED or general PWM
  // The channel cannot have and overcurrent/overtemperature error
  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no diagnostics 1: diagnostics enabled
  // only one channel can be diagnosed at a time - queue up diagnostics for each channel for each device for simultaneous requests
  std::map<uint8_t, std::queue<OutputChannel>> device_mapped_diagnostics_queue;
  for (uint8_t i = 0; i < device_count; i++) {
    for (uint8_t j = 2; j <= MAX_OUTPUT_CHANNEL; j++) {
      bool configured_as_high_side = devices[i].output_side[(OutputChannel)j] == OutputType::HIGH_SIDE;
      bool not_configured_for_led_pwm = devices[i].led_pwm_config == NULL || devices[i].led_pwm_config->channel != (OutputChannel)j;
      bool not_configured_for_general_pwm = devices[i].general_pwm_config == NULL || devices[i].general_pwm_config->channel != (OutputChannel)j;
      bool no_overcurrent_overtemperature_error = !devices[i].diagnostics[(OutputChannel)j].overcurrent_overtemperature_detected;
      
      if (configured_as_high_side && not_configured_for_led_pwm && not_configured_for_general_pwm && no_overcurrent_overtemperature_error) {
        device_mapped_diagnostics_queue[i].push((OutputChannel)j);
      }
    }
  }

  // iterate through each device and queue up diagnostics that can be simultaneously requested on the daisy chain  
  // while all devices have queues with diagnostics to request
  while (true) {
    bool all_queues_empty = true;
    for (uint8_t i = 0; i < device_count; i++) {
      if (!device_mapped_diagnostics_queue[i].empty()) {
        all_queues_empty = false;
        break;
      }
    }

    if (all_queues_empty)
      break;

    for (uint8_t i = 0; i < device_count; i++) {
      if (!device_mapped_diagnostics_queue[i].empty()) {
        // get next channel to diagnose
        OutputChannel channel = device_mapped_diagnostics_queue[i].front();
        device_mapped_diagnostics_queue[i].pop();

        // enable diagnostics for channel
        prepareWrite(i, DIAG_OPL_ON_REG, 1 << channel);
      }
      else {
        // dummy write to keep daisy chain in sync
        prepareWrite(i, DIAG_OPL_ON_REG, 0);
      }
    }

    transfer();
    // wait 210ms for diagnostics cycle to complete
    delay(210);
  }

  // read on mode open load detection
  for (uint8_t i = 0; i < device_count; i++) {
    prepareRead(i, DIAG_OPL_ON_REG);
  }
  transfer(true);


  // Channel [7:0], MSB refers to channel 7. Bit values: 0: no open load detected 1: open load detected
  for (uint8_t i = 0; i < device_count; i++) {
    for (uint8_t j = 0; j <= MAX_OUTPUT_CHANNEL; j++) {
      // get device channel diagnostics reference
      devices[i].diagnostics[(OutputChannel)j].on_state_open_load_detected = (devices[i].transaction.data >> j) & 1;
    }
  }
}

L9026ChannelDiagnostics L9026::getChannelDiagnostics(uint8_t device_id, OutputChannel channel) {
  return devices[device_id].diagnostics[channel];
}