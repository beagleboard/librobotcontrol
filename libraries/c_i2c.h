#ifndef _I2C_H_
#define _I2C_H_
/* 
 * Reference:
 * i2c dev driver:
 * i2c-dev.h:
 * https://github.com/MarkAYoder/BeagleBoard-exercises/blob/master/i2c/i2c-dev.h
 * 
 * High level library wrapper
 * https://github.com/Michael0310/SensorCape/blob/master/SensorCape_v0.2_clib_devel/lib/i2c.cpp
 * 
 * int32_tdef? 
 */
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdint.h>    // uint std 
//#include <linux/i2c-dev.h>
#include "i2c-dev.h"

#define DEFAULT_READ_TIMEOUT 1000
#define FILENAME_LENGTH  20
#define BUFFER_LENGTH   14
typedef struct i2c_t {
  /* data */
  uint8_t devAddr;
  uint8_t bus;
  int8_t file;
  // Parameterize 20
  char filename[FILENAME_LENGTH];
  /* buf */
  uint8_t buffer[BUFFER_LENGTH];

} i2c_t;

void i2c_init(i2c_t* i2c, uint8_t bus, uint8_t devAddr);
bool openConnection(i2c_t* i2c);
bool closeConnection(i2c_t* i2c);
int8_t readBytes(i2c_t* i2c, uint8_t regAddr, uint8_t length,  uint8_t *data, uint16_t timeout);
int8_t readByte(i2c_t* i2c, uint8_t regAddr, uint8_t *data, uint16_t timeout);
int8_t readWords(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout);
int8_t readWord(i2c_t* i2c, uint8_t regAddr, uint16_t *data, uint16_t timeout);
int8_t readBit(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
int8_t readBits(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t readBitsW(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout);
bool writeBytes(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint8_t* data);
bool writeByte(i2c_t* i2c, uint8_t regAddr, uint8_t data);
bool writeWords(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint16_t* data);
bool writeWord(i2c_t* i2c, uint8_t regAddr, uint16_t data);
bool writeBit(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool writeBitW(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint16_t data);
bool writeBits(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool writeBitsW(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
#endif
