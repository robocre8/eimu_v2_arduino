#include "eimuV2.h"

EIMU_V2::EIMU_V2(int slave_addr)
{
  slaveAddr = slave_addr;
}

uint8_t computeChecksum(uint8_t *packet, uint8_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += packet[i]; 
  }
  return sum & 0xFF; 
}

void EIMU_V2::send_packet_without_payload(uint8_t cmd)
{
  // Build packet: start_byte + cmd + length + pos + float + checksum
  uint8_t packet[4];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 0; // msg length = 0

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 3);
  packet[3] = checksum;

  Wire.beginTransmission(slaveAddr);
  Wire.write(packet, sizeof(packet));
  Wire.endTransmission(true);
}

void EIMU_V2::write_data1(uint8_t cmd, uint8_t pos, float val)
{
  // Build packet: start_byte + cmd + length + pos + float + checksum
  uint8_t packet[1 + 1 + 1 + 1 + 4 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 5; // msg is uint8 + float = 5byte length
  packet[3] = pos;
  memcpy(&packet[4], &val, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 8);
  packet[8] = checksum;

  Wire.beginTransmission(slaveAddr);
  Wire.write(packet, sizeof(packet));
  Wire.endTransmission(true);
}

void EIMU_V2::write_data3(uint8_t cmd, float val0, float val1, float val2)
{
  // Build packet: start_byte + cmd + length + float*3 + checksum
  uint8_t packet[1 + 1 + 1 + 12 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 12; // msg is 3 float = 12byte length
  memcpy(&packet[3], &val0, sizeof(float));
  memcpy(&packet[7], &val1, sizeof(float));
  memcpy(&packet[11], &val2, sizeof(float));
  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 15);
  packet[15] = checksum;

  Wire.beginTransmission(slaveAddr);
  Wire.write(packet, sizeof(packet));
  Wire.endTransmission(true);
}

void EIMU_V2::write_data4(uint8_t cmd, float val0, float val1, float val2, float val3)
{
  // Build packet: start_byte + cmd + length + float*4 + checksum
  uint8_t packet[1 + 1 + 1 + 16 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 16; // msg is 4 float = 16byte length
  memcpy(&packet[3], &val0, sizeof(float));
  memcpy(&packet[7], &val1, sizeof(float));
  memcpy(&packet[11], &val2, sizeof(float));
  memcpy(&packet[15], &val3, sizeof(float));
  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 19);
  packet[19] = checksum;

  Wire.beginTransmission(slaveAddr);
  Wire.write(packet, sizeof(packet));
  Wire.endTransmission(true);
}

float EIMU_V2::read_data1()
{
  uint8_t buffer[4];
  float res;
  uint8_t dataSizeInBytes = Wire.requestFrom(slaveAddr, 4);
  for (size_t i = 0; i < dataSizeInBytes; i += 1)
  {
    uint8_t data = Wire.read();
    buffer[i] = data;
  }
  memcpy(&res, &buffer[0], sizeof(float));
  return res;
}

void EIMU_V2::read_data3(float &val0, float &val1, float &val2)
{
  uint8_t buffer[12];
  uint8_t dataSizeInBytes = Wire.requestFrom(slaveAddr, 12);
  for (size_t i = 0; i < dataSizeInBytes; i += 1)
  {
    uint8_t data = Wire.read();
    buffer[i] = data;
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
  memcpy(&val2, &buffer[8], sizeof(float));
}

void EIMU_V2::read_data4(float &val0, float &val1, float &val2, float &val3)
{
  uint8_t buffer[16];
  uint8_t dataSizeInBytes = Wire.requestFrom(slaveAddr, 16);
  for (size_t i = 0; i < dataSizeInBytes; i += 1)
  {
    uint8_t data = Wire.read();
    buffer[i] = data;
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
  memcpy(&val2, &buffer[8], sizeof(float));
  memcpy(&val3, &buffer[12], sizeof(float));
}

void EIMU_V2::readQuat(float &qw, float &qx, float &qy, float &qz)
{
  send_packet_without_payload(READ_QUAT);
  read_data4(qw, qx, qy, qz);
  read_data4(qw, qx, qy, qz);
}

void EIMU_V2::readRPY(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_RPY);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readRPYVariance(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_RPY_VAR);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readAcc(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_ACC);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readAccVariance(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_ACC_VAR);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readGyro(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_GYRO);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readGyroVariance(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_GYRO_VAR);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

void EIMU_V2::readMag(float &x, float &y, float &z)
{
  send_packet_without_payload(READ_MAG);
  read_data3(x, y, z);
  read_data3(x, y, z);
}

int EIMU_V2::setWorldFrameId(int id=1)
{
  float res;
  write_data1(SET_FRAME_ID, 0, (float)id);
  res = read_data1();
  res = read_data1();
  return (int)res;
}

int EIMU_V2::getWorldFrameId()
{
  float id;
  write_data1(GET_FRAME_ID, 0, 0.0);
  id = read_data1();
  id = read_data1();
  return (int)id;
}

float EIMU_V2::getFilterGain()
{
  float gain;
  write_data1(SET_FRAME_ID, 0, 0.0);
  gain = read_data1();
  gain = read_data1();
  return gain;
}