#pragma once
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "Arduino.h"
#include "PeripheralPins.h"

#define M3_CAN_RX PA8
#define M3_CAN_TX PA15
#define M3_CAN_TERMINATION_ENABLE PB15

#define HV_CAN_RX PB5
#define HV_CAN_TX PB13
#define HV_CAN_TERMINATION_ENABLE PB12

#define BRZ_CAN_RX PB8
#define BRZ_CAN_TX PB9
#define BRZ_CAN_TERMINATION_ENABLE PB7

#define IPC_CAN_RX PB12
#define IPC_CAN_TX PB13

// Callback function type for RX interrupts
typedef void (*CANRxCallback)(uint32_t id, uint8_t* data, uint8_t len);

class CANBus {
  static CANRxCallback         callback1;
  static CANRxCallback         callback2;
  static CANRxCallback         callback3;

  public:
    CANBus(uint32_t rx_pin, uint32_t tx_pin, uint32_t termination_pin = 255);
    bool begin(uint32_t baudrate);
    void end();
    bool sendMessage(uint32_t id, uint8_t* data, uint8_t len);
    bool receiveMessage(uint32_t &id, uint8_t* data, uint8_t &len);
    void setTermination(bool enabled);
    
    // Filter configuration
    bool setFilter(uint32_t filter_id, uint32_t filter_mask, uint32_t filter_bank = 0);
    bool setFilterRange(uint32_t id_low, uint32_t id_high, uint32_t filter_bank = 0);
    bool disableFilter(uint32_t filter_bank = 0);
    
    // Interrupt configuration
    bool enableRxInterrupt(CANRxCallback callback);
    void disableRxInterrupt();
    
    // Static interrupt handlers
    static void handleRxInterrupt(CAN_HandleTypeDef* hcan);

    // Static CAN handles
    static CAN_HandleTypeDef     hcan1;
    static CAN_HandleTypeDef     hcan2;
    static CAN_HandleTypeDef     hcan3;
    
  private:
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint8_t termination_pin;
    CAN_HandleTypeDef* getCAN();
    uint8_t getFilterBank();
};