#include "SPI.h"
#include "functional"
#include "map"
#include "queue"

#define CHIP_ID_REG 0x00

// general configuration registers
#define CFG0_REG 0x01
#define CFG1_REG 0x02
#define CFG2_REG 0x03

// bulb inrush mode configuration register
#define BIM_REG 0x04

// reserved
#define RESERVED_REG 0x05

// power output control register
#define PWM_SPI_REG 0x06

// IN0 input mapping register
#define MAP_IN0_REG 0x07

// IN1 input mapping register
#define MAP_IN1_REG 0x08

// Internal PWM generator mapping register
#define MAP_PWM_REG 0x09

// Internal PWM generator selection register
#define PWM_SEL_REG 0x0A

// PWM Gen duty cycle setting
#define PWM_GEN_DC_REG 0x0B

// PWM LED duty cycle setting
#define PWM_LED_DC_REG 0x0C

// OFF Diagnostic enable register
#define DIAG_OFF_EN_REG 0x0D

// Open Load in ON Diagnostic enable register
#define DIAG_OPL_ON_EN_REG 0x0E

// Overcurrent/overtemperature error Clear on write register
#define DIAG_OVC_OVT_RLW_REG 0x0F

// Status register 0
#define STA_0_REG 0x10

// Status register 1
#define STA_1_REG 0x11

// Overcurrent/overtemperature detection Register
#define DIAG_OVC_OVT_REG 0x12

// Driver Open load detection in off state Register
#define DIAG_OPL_OFF_REG 0x13

// Driver Open load detection in on state Register
#define DIAG_OPL_ON_REG 0x14

// Driver shorted load detection in OFF state Register
#define DIAG_SHG_REG 0x15

#define DEVICE_BUFFER_OFFSET(count, i) ((count - 1 - i) * sizeof(uint16_t))

#define DISABLE_PWM_OUTPUT 0x00
#define MAX_OUTPUT_CHANNEL 7

#define RESET 0x40
#define ENABLE_ACTIVE_MODE 0x20

enum L9026PWMInput {
  IN0 = 0,
  IN1 = 1
};

struct TransactionData {
  uint8_t address;
  uint8_t data;
  bool write;
  bool frame_counter;
  bool last_frame_error;
};

enum PWMFrequency {
  PWM_FREQUENCY_122_5_HZ = 0,
  PWM_FREQUENCY_245_1_HZ = 1,
  PWM_FREQUENCY_490_2_HZ = 2,
  PWM_FREQUENCY_980_4_HZ = 3,
};

enum PWMFrequencyAdjustment {
  NO_ADJUSTMENT = 0,
  NEG_15_PERCENT_ON_SELECTED_FREQUENCY = 1,
  POS_15_PERCENT_ON_SELECTED_FREQUENCY = 2,
  NO_ADJUSTMENT_2 = 3
};

enum OutputChannel : uint8_t {
  CHANNEL_0 = 0,
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
  CHANNEL_4,
  CHANNEL_5,
  CHANNEL_6,
  CHANNEL_7
};

struct L9026LEDPWMConfiguration {
  OutputChannel channel;
  bool active;
  PWMFrequency frequency;
};

struct L9026GeneralPWMConfiguration {
  OutputChannel channel;
  PWMFrequencyAdjustment frequency_adjustment;
  PWMFrequency frequency;
};

enum OutputType {
  LOW_SIDE = 0,
  HIGH_SIDE = 1
};

enum L9026OperatingMode : uint8_t {
  L9026_SLEEP_MODE = 0,
  L9026_LIMP_HOME_MODE,
  L9026_IDLE_MODE,
  L9026_ACTIVE_MODE
};

struct L9026DeviceStatus {
  bool disable_status;
  bool idle_status;
  bool in1_status;
  bool in0_status;
  bool overcurrent_overtemperature_detected;
  bool off_state_diagnostic_failure_detected;
  bool power_on_reset_condition_detected;
  bool vddio_undervoltage_detected;
  bool vbatt_undervoltage_detected;
  L9026OperatingMode operating_mode;
};

struct L9026ChannelDiagnostics {
  bool overcurrent_overtemperature_detected;
  bool off_state_open_load_detected;
  bool on_state_open_load_detected;
  bool shorted_load_detected;
};

struct L9026Device {
  uint32_t in0;
  uint32_t in1;
  uint32_t idle;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];
  uint8_t id;

  L9026LEDPWMConfiguration *led_pwm_config;
  L9026GeneralPWMConfiguration *general_pwm_config;
  uint8_t registers[16];
  TransactionData transaction;
  bool disable_enabled;
  bool reset_enabled;

  L9026DeviceStatus status;
  std::map<OutputChannel, OutputType> output_side;
  std::map<OutputChannel, L9026ChannelDiagnostics> diagnostics;
};

class L9026 {
  SPIClass *spi;
  uint32_t cs;
  uint32_t idle;
  L9026Device *devices;
  uint8_t device_count;
  size_t buffer_size;
  uint8_t *tx_buffer;
  uint8_t *rx_buffer;

  std::function<void(void)> take_spi;
  std::function<void(void)> release_spi;

  public:
    L9026(SPIClass *spi, uint32_t cs, L9026Device devices[], uint8_t device_count, std::function<void(void)> take_spi, std::function<void(void)> release_spi) : spi(spi), cs(cs), devices(devices), device_count(device_count), take_spi(take_spi), release_spi(release_spi) {};
    bool begin();
    void end();
    bool getChipID();

    // CFG_0
    void configureOutputSide(uint8_t device_id, OutputChannel channel, OutputType type);
    void configureDisableResetPins(uint8_t device_id, bool disable_enabled, bool reset_enabled);

    // CFG_1, MAP_PWM, PWM_SEL
    void configureLEDPWM(uint8_t device_id, L9026LEDPWMConfiguration *config);

    // CFG_2, MAP_PWM, PWM_SEL
    void configureGeneralPWM(uint8_t device_id, L9026GeneralPWMConfiguration *config);

    // BIM
    void configureBulbInrushMode(uint8_t device_id, OutputChannel channel, bool enable);

    // PWM_SPI
    void setOutput(uint8_t device_id, OutputChannel channel, bool enable);

    // MAP_IN0, MAP_IN1
    void mapInput(uint8_t device_id, L9026PWMInput input, OutputChannel channel);

    // PWM_GEN_DC
    void setGeneralPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle);

    // PWM_LED_DC
    void setLEDPWMDutyCycle(uint8_t device_id, uint8_t duty_cycle);

    // CFG_1
    void setActiveMode(uint8_t device_id, bool enable);

    // updates the registers for all devices
    void updateConfiguration();

    // update just output / duty cycle registers
    void updateOutputs();

    // resets all devices
    void softReset();

    // get device status
    void readAllDeviceStatus();
    L9026DeviceStatus getDeviceStatus(uint8_t device_id);

    // read off mode diagnostics
    void readAllDeviceOffModeDiagnostics();

    // read on mode diagnostics
    void readAllDeviceOnModeDiagnostics();

    // get channel diagnostics
    L9026ChannelDiagnostics getChannelDiagnostics(uint8_t device_id, OutputChannel channel);

    void prepareRead(uint8_t device_id, uint8_t address);
    void prepareWrite(uint8_t device_id, uint8_t address, uint8_t data);
    void packFrame(TransactionData &transaction, uint8_t *buffer);
    void unpackFrame(TransactionData &transaction, uint8_t *buffer);
    void buildTransferBuffer(uint8_t *buffer);
    void unpackTransferBuffer(uint8_t *buffer);
    void toggleLimpHomeMode(bool enable);
    void toggleSleepMode(bool enable);

    uint8_t* getTxBuffer() { return tx_buffer; }
    uint8_t* getRxBuffer() { return rx_buffer; }

    L9026Device* getDevices() { return devices; }

    void transfer(bool get_response = false);
};