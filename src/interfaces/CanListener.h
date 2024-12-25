#include <vector>
#include <stdint.h>

class CanListener {
  std::vector<uint16_t> ids;
  virtual void onCanMessage(uint16_t id, uint8_t* data, uint8_t len) = 0;

  // add a RTOS backed queue to store messages
};