#include "can.h"

// Define static members
CAN_HandleTypeDef CANBus::hcan1 = {0};
CAN_HandleTypeDef CANBus::hcan2 = {0};
CAN_HandleTypeDef CANBus::hcan3 = {0};

CANRxCallback CANBus::callback1 = nullptr;
CANRxCallback CANBus::callback2 = nullptr;
CANRxCallback CANBus::callback3 = nullptr;

CANBus::CANBus(uint32_t rx_pin, uint32_t tx_pin, uint32_t termination_pin)
  : rx_pin(rx_pin), tx_pin(tx_pin), termination_pin(termination_pin) {
}

bool CANBus::begin(uint32_t baudrate) {
  CAN_HandleTypeDef* hcan = nullptr;
  HAL_StatusTypeDef status;
  
  // Convert Arduino pins to PinName
  PinName rx_pinname = digitalPinToPinName(rx_pin);
  PinName tx_pinname = digitalPinToPinName(tx_pin);
  
  // Find CAN peripheral and alternate function for RX pin
  void *can_rx = pinmap_find_peripheral(rx_pinname, PinMap_CAN_RD);
  void *can_tx = pinmap_find_peripheral(tx_pinname, PinMap_CAN_TD);
  
  if (can_rx == NP || can_tx == NP) {
    Serial.printf("Invalid CAN pins: RX=%d, TX=%d\n", rx_pin, tx_pin);
    return false; // Invalid pins
  }
  
  // Verify both pins map to the same CAN peripheral
  if (can_rx != can_tx) {
    Serial.printf("Mismatched CAN peripherals for RX and TX pins: RX=%d, TX=%d\n", rx_pin, tx_pin);
    return false; // RX and TX must be on same CAN peripheral
  }
  
  // Determine which CAN handle to use
  if (can_rx == CAN1) {
    hcan = &hcan1;
    hcan->Instance = CAN1;
    __HAL_RCC_CAN1_CLK_ENABLE();
  } 
#ifdef CAN2
  else if (can_rx == CAN2) {
    hcan = &hcan2;
    hcan->Instance = CAN2;
    __HAL_RCC_CAN2_CLK_ENABLE();
  }
#endif
#ifdef CAN3
  else if (can_rx == CAN3) {
    hcan = &hcan3;
    hcan->Instance = CAN3;
    __HAL_RCC_CAN3_CLK_ENABLE();
  }
#endif
  else {
    return false;
  }

  // Get alternate function for pins
  uint32_t rx_af = pinmap_find_function(rx_pinname, PinMap_CAN_RD);
  uint32_t tx_af = pinmap_find_function(tx_pinname, PinMap_CAN_TD);

  // Configure RX pin
  pin_function(rx_pinname, rx_af);
  
  // Configure TX pin  
  pin_function(tx_pinname, tx_af);

  // Configure termination resistor if available
  if (termination_pin != 255) {
    pinMode(termination_pin, OUTPUT);
    digitalWrite(termination_pin, LOW); // Disabled by default
  }

  // Configure CAN parameters
  // Assuming 45MHz CAN clock for STM32F4
  uint32_t prescaler;
  uint32_t bs1, bs2;
  
  switch(baudrate) {
    case 125000:  prescaler = 18; bs1 = CAN_BS1_13TQ; bs2 = CAN_BS2_2TQ; break;
    case 250000:  prescaler = 9;  bs1 = CAN_BS1_13TQ; bs2 = CAN_BS2_2TQ; break;
    case 500000:  prescaler = 6;  bs1 = CAN_BS1_11TQ; bs2 = CAN_BS2_3TQ; break;
    case 1000000: prescaler = 3;  bs1 = CAN_BS1_11TQ; bs2 = CAN_BS2_3TQ; break;
    default: return false;
  }

  hcan->Init.Prescaler = prescaler;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = bs1;
  hcan->Init.TimeSeg2 = bs2;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = ENABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = ENABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(hcan) != HAL_OK) {
    return false;
  }

  // Configure default filter to accept all messages
  CAN_FilterTypeDef filter = {0};
  uint8_t filter_bank = getFilterBank();
  
  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  status = HAL_CAN_ConfigFilter(hcan, &filter);
  if (status != HAL_OK) {
    Serial.printf("CAN Filter Config Error: %d\n", status);
    return false;
  }

  status = HAL_CAN_Start(hcan);
  if (status != HAL_OK) {
    Serial.printf("CAN Start Error: %d\n", status);
    return false;
  }

  return true;
}

void CANBus::end() {
  CAN_HandleTypeDef* hcan = getCAN();
  if (hcan) {
    disableRxInterrupt();
    HAL_CAN_Stop(hcan);
    HAL_CAN_DeInit(hcan);
  }
  
  // Disable termination resistor
  if (termination_pin != 255) {
    digitalWrite(termination_pin, LOW);
  }
}

bool CANBus::sendMessage(uint32_t id, uint8_t* data, uint8_t len) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return false;

  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;

  txHeader.StdId = id;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;

  auto error = HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox);

  if (error != HAL_OK) {
    Serial.printf("CAN Send Error: %d\n", error);
    return false;
  }

  return true;
}

bool CANBus::receiveMessage(uint32_t &id, uint8_t* data, uint8_t &len) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return false;

  CAN_RxHeaderTypeDef rxHeader;

  if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0) {
    return false;
  }

  auto error = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data);
  if (error != HAL_OK) {
    Serial.printf("CAN Receive Error: %d\n", error);
    return false;
  }

  id = rxHeader.StdId;
  len = rxHeader.DLC;

  return true;
}

void CANBus::setTermination(bool enabled) {
  if (termination_pin == 255) return;
  digitalWrite(termination_pin, enabled ? HIGH : LOW);
}

bool CANBus::setFilter(uint32_t filter_id, uint32_t filter_mask, uint32_t filter_bank) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (filter_id << 5) >> 16;
  filter.FilterIdLow = (filter_id << 5) & 0xFFFF;
  filter.FilterMaskIdHigh = (filter_mask << 5) >> 16;
  filter.FilterMaskIdLow = (filter_mask << 5) & 0xFFFF;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::setFilterRange(uint32_t id_low, uint32_t id_high, uint32_t filter_bank) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (id_low << 5) >> 16;
  filter.FilterIdLow = (id_low << 5) & 0xFFFF;
  filter.FilterMaskIdHigh = (id_high << 5) >> 16;
  filter.FilterMaskIdLow = (id_high << 5) & 0xFFFF;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::disableFilter(uint32_t filter_bank) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterActivation = DISABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::enableRxInterrupt(CANRxCallback callback) {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan || !callback) return false;

  // Store callback for this CAN instance
  if (hcan->Instance == CAN1) {
    callback1 = callback;
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
#ifdef CAN2
  else if (hcan->Instance == CAN2) {
    callback2 = callback;
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (hcan->Instance == CAN3) {
    callback3 = callback;
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
  }
#endif
  else {
    return false;
  }

  // Enable FIFO 0 message pending interrupt
  return (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
}

void CANBus::disableRxInterrupt() {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return;

  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  if (hcan->Instance == CAN1) {
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    callback1 = nullptr;
  }
#ifdef CAN2
  else if (hcan->Instance == CAN2) {
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    callback2 = nullptr;
  }
#endif
#ifdef CAN3
  else if (hcan->Instance == CAN3) {
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
    callback3 = nullptr;
  }
#endif
}

void CANBus::handleRxInterrupt(CAN_HandleTypeDef* hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t data[8];
  
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) == HAL_OK) {
    CANRxCallback callback = nullptr;
    
    if (hcan->Instance == CAN1) callback = callback1;
#ifdef CAN2
    else if (hcan->Instance == CAN2) callback = callback2;
#endif
#ifdef CAN3
    else if (hcan->Instance == CAN3) callback = callback3;
#endif
    
    if (callback) {
      callback(rxHeader.StdId, data, rxHeader.DLC);
    }
  }
}

CAN_HandleTypeDef* CANBus::getCAN() {
  PinName rx_pinname = digitalPinToPinName(rx_pin);
  void *can_peripheral = pinmap_find_peripheral(rx_pinname, PinMap_CAN_RD);
  
  if (can_peripheral == CAN1) return &hcan1;
#ifdef CAN2
  if (can_peripheral == CAN2) return &hcan2;
#endif
#ifdef CAN3
  if (can_peripheral == CAN3) return &hcan3;
#endif
  
  return nullptr;
}

uint8_t CANBus::getFilterBank() {
  CAN_HandleTypeDef* hcan = getCAN();
  if (!hcan) return 0;
  
  if (hcan->Instance == CAN1) return 0;
#ifdef CAN2
  if (hcan->Instance == CAN2) return 14;
#endif
#ifdef CAN3
  if (hcan->Instance == CAN3) return 21;
#endif
  
  return 0;
}

// Interrupt handlers - these need to be defined outside the class
extern "C" {
  void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan1);
  }

#ifdef CAN2
  void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan2);
  }
#endif

#ifdef CAN3
  void CAN3_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan3);
  }
#endif

  // HAL callback
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CANBus::handleRxInterrupt(hcan);
  }
}
