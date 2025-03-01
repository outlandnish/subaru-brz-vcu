#include <Arduino.h>
#include <unity.h>
#include <SPI.h>
#include <l9026.h>

#define Serial SerialUSB

#define IO_COPI PA7
#define IO_CIPO PA6
#define IO_SCK PB3
#define INPUT_CS PA9
#define OUTPUT_CS PA10
#define INPUT_INTERRUPT PB1
#define INPUT_RESET PB2
#define SSHUT_ENABLE PB0

SPIClass spi(IO_COPI, IO_CIPO, IO_SCK, NC);

L9026Device outputDevices[] = {
  {SSHUT_ENABLE, NC, 0, 0, false, false, 0},
  {NC, NC, 0, 0, false, false, 0}
};

void takeSPI() {}
void releaseSPI() {}

L9026 outputs(&spi, OUTPUT_CS, outputDevices, 2, takeSPI, releaseSPI);

void test_array_equal(uint8_t *expected, uint8_t *actual, size_t size) {
  for (size_t i = 0; i < size; i++) {
    TEST_ASSERT_EQUAL(expected[i], actual[i]);
  }
}

void setUp(void) {
  Serial.begin(115200);

  // set stuff up here
  spi.begin();

  pinMode(OUTPUT_CS, OUTPUT);
  digitalWrite(OUTPUT_CS, HIGH);

  outputs.begin();
}

void tearDown(void) {
  outputs.end();
}

void test_l9026_pack_frame_0() {
  TransactionData data {0x01, 0x02, true, false, false};
  uint8_t *frame = new uint8_t[2];
  uint8_t expected[] = { 0x84, 0x0A };

  outputs.packFrame(data, frame);
  test_array_equal(expected, frame, 2);

  delete frame;
}

void test_l9026_pack_frame_1() {
  TransactionData data {0x10, 0x05, true, true, false};
  uint8_t *frame = new uint8_t[2];
  uint8_t expected[] = { 0xC0, 0x15 };

  outputs.packFrame(data, frame);
  test_array_equal(expected, frame, 2);

  data.write = false;
  expected[0] = 0x40;
  expected[1] = 0x03;

  outputs.packFrame(data, frame);
  test_array_equal(expected, frame, 2);

  delete frame;
}

void test_l9026_unpack_frame_0() {
  TransactionData data;
  uint8_t frame[] = { 0x04, 0x08 };

  outputs.unpackFrame(data, frame);

  TEST_ASSERT_EQUAL(0x01, data.address);
  TEST_ASSERT_EQUAL(0x02, data.data);
  TEST_ASSERT_EQUAL(false, data.last_frame_error);
  TEST_ASSERT_EQUAL(false, data.frame_counter);
}

void test_l9026_unpack_frame_1() {
  TransactionData data;
  uint8_t frame[] = { 0x84, 0x0A };

  outputs.unpackFrame(data, frame);

  TEST_ASSERT_EQUAL(0x01, data.address);
  TEST_ASSERT_EQUAL(0x02, data.data);
  TEST_ASSERT_EQUAL(true, data.last_frame_error);
  TEST_ASSERT_EQUAL(false, data.frame_counter);
}

void test_l9026_unpack_frame_2() {
  TransactionData data;
  uint8_t frame[] = { 0x84, 0x09 };

  outputs.unpackFrame(data, frame);

  TEST_ASSERT_EQUAL(0x01, data.address);
  TEST_ASSERT_EQUAL(0x02, data.data);
  TEST_ASSERT_EQUAL(true, data.last_frame_error);
  TEST_ASSERT_EQUAL(true, data.frame_counter);
}

void test_l9026_build_transfer_buffer() {
  outputs.prepareRead(0, 0x01);
  outputs.prepareWrite(1, 0x02, 0x03);

  uint8_t buffer[2 * sizeof(uint16_t)];
  outputs.buildTransferBuffer(buffer);

  // 0x0402 should be the second uint16_t in Big Endian in the buffer
  TEST_ASSERT_EQUAL(0x04, ((uint8_t *)buffer)[2]);
  TEST_ASSERT_EQUAL(0x02, ((uint8_t *)buffer)[3]);

  // 0x880C should be the first uint16_t in Big Endian in the buffer
  TEST_ASSERT_EQUAL(0x88, ((uint8_t *)buffer)[0]);
  TEST_ASSERT_EQUAL(0x0C, ((uint8_t *)buffer)[1]);
}

void test_l9026_unpack_transfer_buffer() {
  uint8_t buffer[] = { 0x88, 0x0C, 0x04, 0x02 };
  outputs.unpackTransferBuffer(buffer);

  TEST_ASSERT_EQUAL(0x01, outputDevices[0].transaction.address);
  TEST_ASSERT_EQUAL(0x02, outputDevices[1].transaction.address);
  TEST_ASSERT_EQUAL(0x03, outputDevices[1].transaction.data);
}

void test_l9026_prepare_read() {
  outputs.prepareRead(0, 0x01);
  outputs.prepareRead(1, 0x02);

  TEST_ASSERT_EQUAL(0x01, outputDevices[0].transaction.address);
  TEST_ASSERT_EQUAL(0x02, outputDevices[1].transaction.address);
}

void test_l9026_prepare_read_and_build_transfer_buffer() {
  outputs.prepareRead(0, 0x01);
  outputs.prepareWrite(1, 0x02, 0x03);

  uint8_t buffer[2 * sizeof(uint16_t)];
  outputs.buildTransferBuffer(buffer);
  auto devices = outputs.getDevices();

  // ensure device[0] has correct buffer
  TEST_ASSERT_EQUAL(0x04, devices[0].tx_buffer[0]);
  TEST_ASSERT_EQUAL(0x02, devices[0].tx_buffer[1]);

  // ensure device[1] has correct buffer
  TEST_ASSERT_EQUAL(0x88, devices[1].tx_buffer[0]);
  TEST_ASSERT_EQUAL(0x0C, devices[1].tx_buffer[1]);

  // 0x0402 should be the second uint16_t in Big Endian in the buffer
  TEST_ASSERT_EQUAL(0x04, ((uint8_t *)buffer)[2]);
  TEST_ASSERT_EQUAL(0x02, ((uint8_t *)buffer)[3]);

  // 0x880C should be the first uint16_t in Big Endian in the buffer
  TEST_ASSERT_EQUAL(0x88, ((uint8_t *)buffer)[0]);
  TEST_ASSERT_EQUAL(0x0C, ((uint8_t *)buffer)[1]);
}

void test_l9026_get_chip_id() {
  outputs.getChipID();

  TEST_ASSERT_NOT_EQUAL(0, outputDevices[0].id);
  TEST_ASSERT_NOT_EQUAL(0, outputDevices[1].id);

  // TEST_ASSERT_EQUAL(0x02, outputDevices[0].rx_buffer);
  // TEST_ASSERT_EQUAL(0x03, outputDevices[1].rx_buffer);
}

void setup()
{
  delay(2000); // service delay
  UNITY_BEGIN();

  RUN_TEST(test_l9026_pack_frame_0);
  RUN_TEST(test_l9026_pack_frame_1);
  RUN_TEST(test_l9026_unpack_frame_0);
  RUN_TEST(test_l9026_unpack_frame_1);
  RUN_TEST(test_l9026_unpack_frame_2);
  RUN_TEST(test_l9026_build_transfer_buffer);
  RUN_TEST(test_l9026_unpack_transfer_buffer);
  RUN_TEST(test_l9026_prepare_read);
  RUN_TEST(test_l9026_prepare_read_and_build_transfer_buffer);
  RUN_TEST(test_l9026_get_chip_id);

  UNITY_END(); // stop unit testing
}

void loop()
{
}