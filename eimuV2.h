#ifndef EIMU_V2_H
#define EIMU_V2_H

#include <Arduino.h>
#include <Wire.h>

class EIMU_V2
{
public:
  EIMU_V2(int);

  void readQuat(float &w, float &x, float &y, float &z);
  void readRPY(float &x, float &y, float &z);
  void readRPYVariance(float &x, float &y, float &z);
  void readAcc(float &x, float &y, float &z);
  void readAccVariance(float &x, float &y, float &z);
  void readGyro(float &x, float &y, float &z);
  void readGyroVariance(float &x, float &y, float &z);
  void readMag(float &x, float &y, float &z);

  int setWorldFrameId(int);

  int getWorldFrameId();

  float getFilterGain();


private:
  int slaveAddr;
  void send_packet_without_payload(uint8_t cmd);
  void write_data1(uint8_t cmd, uint8_t pos, float val);
  void write_data3(uint8_t cmd, float val0, float val1, float val2);
  void write_data4(uint8_t cmd, float val0, float val1, float val2, float val3);
  float read_data1();
  void read_data3(float &val0, float &val1, float &val2);
  void read_data4(float &val0, float &val1, float &val2, float &val3);

  //  Protocol Command IDs -------------
  const uint8_t START_BYTE = 0xBB;
  const uint8_t READ_QUAT = 0x01;
  const uint8_t READ_RPY = 0x02;
  const uint8_t READ_RPY_VAR = 0x03;
  const uint8_t READ_ACC = 0x05;
  const uint8_t READ_ACC_VAR = 0x09;
  const uint8_t READ_GYRO = 0x0B;
  const uint8_t READ_GYRO_VAR = 0x0F;
  const uint8_t READ_MAG = 0x11;
  const uint8_t GET_FILTER_GAIN = 0x1E;
  const uint8_t SET_FRAME_ID = 0x1F;
  const uint8_t GET_FRAME_ID = 0x20;
  //---------------------------------------------
};

#endif