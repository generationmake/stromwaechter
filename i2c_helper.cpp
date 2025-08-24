#include "i2c_helper.h"

unsigned short int swap_bytes(unsigned short int input)
{
  return (((input & 0xff) << 8) | ((input >> 8) & 0xff));
}

void i2c_write(char address, char value)
{
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}
void i2c_write_byte(char address, char reg, char value)
{
  Wire.beginTransmission(address);
  Wire.write(reg); // power up continuous mode
  Wire.write(value);
  Wire.endTransmission();
}
void i2c_write_word(char address, char reg, short int value)
{
  Wire.beginTransmission(address);
  Wire.write(reg); // power up continuous mode
  Wire.write((value&0xff00)>>8);
  Wire.write(value&0x00ff);
  Wire.endTransmission();
}
char i2c_read_byte(char address, char reg)
{
  char x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,1);
  if(Wire.available()) x=Wire.read();
  Wire.endTransmission();
  return x;
}
char i2c_smbus_read_byte_data(char address, char reg)
{
  return i2c_read_byte(address, reg);
}
short int i2c_read_word(char address, char reg)
{
  short int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,2);
  if(Wire.available()) x=Wire.read();
  x=x<<8;
  if(Wire.available()) x=x|Wire.read();
  Wire.endTransmission();
  return x;
}
int i2c_read_wordp(char address, char reg)
{
  int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,3);
  if(Wire.available()) x=Wire.read();
  x=x<<8;
  if(Wire.available()) x=x|Wire.read();
  x=x<<4;
  if(Wire.available()) x=x|(Wire.read()>>4);
  Wire.endTransmission();
  return x;
}
short int i2c_read_word_rev(char address, char reg)
{
  short int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,2);
  if(Wire.available()) x=Wire.read();
  if(Wire.available()) x=x|(Wire.read()<<8);
  Wire.endTransmission();
  return x;
}
short int i2c_smbus_read_word_data(char address, char reg)
{
  return i2c_read_word_rev(address, reg);
}
