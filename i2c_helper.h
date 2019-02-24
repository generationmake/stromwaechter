#ifndef I2C_HELPER_H
#define I2C_HELPER_H
#include <Wire.h> // i2c-bib

unsigned short int swap_bytes(unsigned short int input);
void i2c_write(char address, char value);
void i2c_write_byte(char address, char reg, char value);
void i2c_write_word(char address, char reg, short int value);
char i2c_read_byte(char address, char reg);
char i2c_smbus_read_byte_data(char address, char reg);
short int i2c_read_word(char address, char reg);
int i2c_read_wordp(char address, char reg);
short int i2c_read_word_rev(char address, char reg);
short int i2c_smbus_read_word_data(char address, char reg);

#endif /* I2C_HELPER_H */
