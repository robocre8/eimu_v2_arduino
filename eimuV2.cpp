#include "eimuV2.h"

void clearI2CBuffer() {
  while (Wire.available()) {
    Wire.read();  // discard all leftover bytes
  }
}

EIMU_V2::EIMU_V2(int slave_addr)
{
  slaveAddr = slave_addr;
}

float EIMU_V2::readQuat(int pos_no)
{
  float quat = get("/quat", pos_no);
  return quat;
}

float EIMU_V2::readRPY(int pos_no)
{
  float rpy = get("/rpy", pos_no);
  return rpy;
}

float EIMU_V2::readRPYVariance(int pos_no)
{
  float rpy_var = get("/rpy-var", pos_no);
  return rpy_var;
}

float EIMU_V2::readAcc(int pos_no)
{
  float acc = get("/acc", pos_no);
  return acc;
}

float EIMU_V2::readAccVariance(int pos_no)
{
  float acc_var = get("/acc-var", pos_no);
  return acc_var;
}

float EIMU_V2::readGyro(int pos_no)
{
  float gyro = get("/gyro", pos_no);
  return gyro;
}

float EIMU_V2::readGyroVariance(int pos_no)
{
  float gyro_var = get("/gyro-var", pos_no);
  return gyro_var;
}

float EIMU_V2::readMag(int pos_no)
{
  float acc = get("/mag", pos_no);
  return acc;
}

bool EIMU_V2::setWorldFrameId(int id=1)
{
  return send("/frame-id", -1, id);
}

int EIMU_V2::getWorldFrameId()
{
  float id = get("/frame-id", -1);
  return (int)id;
}

float EIMU_V2::getFilterGain()
{
  float gain = get("/gain", -1);
  return gain;
}

float EIMU_V2::get(String cmd_route, int motor_no)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);

  masterSendData(msg_buffer);
  masterReceiveData();
  String dataMsg = masterReceiveData();

  float val = dataMsg.toFloat();
  return val;
}

bool EIMU_V2::send(String cmd_route, int motor_no, float val)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);
  msg_buffer += ",";
  msg_buffer += String(val, 3);

  masterSendData(msg_buffer);
  masterReceiveData();
  String data = masterReceiveData();
  if (data == "1")
    return true;
  else
    return false;
}

void EIMU_V2::masterSendData(String i2c_msg)
{
  Wire.beginTransmission(slaveAddr);
  Wire.print(i2c_msg);
  Wire.endTransmission(true);
}

String EIMU_V2::masterReceiveData()
{
  String i2c_msg = "";
  Wire.flush();
  uint8_t dataSizeInBytes = Wire.requestFrom(slaveAddr, 15);
  for (int i = 0; i < dataSizeInBytes; i += 1)
  {
    char c = Wire.read();
    i2c_msg += c;
  }
  int indexPos = i2c_msg.indexOf((char)255);
  if (indexPos != -1)
  {
    return i2c_msg.substring(0, indexPos);
  }
  i2c_msg.trim();
  return i2c_msg;
}