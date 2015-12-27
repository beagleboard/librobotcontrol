/* 
 * Reference:
 * i2c dev driver:
 * i2c-dev.h:
 * https://github.com/MarkAYoder/BeagleBoard-exercises/blob/master/i2c/i2c-dev.h
 * 
 * High level library wrapper
 * https://github.com/Michael0310/SensorCape/
 * blob/master/SensorCape_v0.2_clib_devel/lib/i2c.cpp
 */
/*struct I2C{
  int address;
  int bus;
  int file;
  char filename[FILENAME_LENGTH];
  int8_t buffer[BUFFER_LENGTH]
};
*/
//#define I2CDEV_DEBUG  1
//#define I2CLIB_DEBUG  1

#include "c_i2c.h"

void i2c_init(i2c_t* i2c, uint8_t bus, uint8_t devAddr){
  i2c->file = 0;
  i2c->devAddr = devAddr;
  i2c->bus     = bus;
}

bool openConnection(i2c_t* i2c){
  snprintf(i2c->filename, FILENAME_LENGTH-1, "/dev/i2c-%d", i2c->bus);
  i2c->file = open(i2c->filename, O_RDWR);
  if (i2c->file < 0)
    return false;
  if(ioctl(i2c->file, I2C_SLAVE, i2c->devAddr) < 0)
    return false;
  return true;
}

bool closeConnection(i2c_t* i2c){
  i2c->devAddr = 0;
  if(close(i2c->file) < 0) return false;
  return true;
}

/** Read multiple bytes from an 8-bit device register.
 * Buffer loop from low to high
 * @param i2c I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t readBytes(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
#ifdef I2CDEV_DEBUG
  printf("I2C (0x");
  printf("%x", i2c->devAddr);
  printf(") reading");
  printf("%d", length);
  printf(" bytes from 0x");
  printf("%x", regAddr);
  printf("...");
#endif
    // Boundary Check
    if(length > BUFFER_LENGTH){
        length = BUFFER_LENGTH;
        // error message handling
#ifdef I2CLIB_DEBUG
        fprintf(stderr, "readByte data length is enforced as BUFFER_LENGTH!\n");
#endif 
    }else if(length < 0){
        length = 0; 
        // error message handling
#ifdef I2CLIB_DEBUG
        fprintf(stderr, "readByte data length is enforced as 0!\n");
#endif
    }
    int8_t count = 0;
    uint8_t status = 0;
//    uint32_t t1 = millis();
        // write first 
        if(write(i2c->file, &regAddr, 1)!=1) 
        /* error handling here */
          status = 1;
        // read later 
        if(read(i2c->file, data, length)!=length)
        /* error handling here */
          status = 1;  

        if(status == 0) {
          count = length;
        } else {
          count = -1 * status;
          // error message handling
#ifdef I2CLIB_DEBUG
          fprintf(stderr, "invalid readBytes 0x%x reading!\n", regAddr);
#endif
        }
    // check for timeout
 //   if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout
#ifdef I2CDEV_DEBUG
  printf(". Done.");
  printf("%d", count);
  printf(" read).");
#endif
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(i2c_t* i2c, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(i2c, regAddr, 1, data, timeout);
}

/** Read multiple words from a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (0 indicates failure)
 */
int8_t readWords(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    #ifdef I2CDEV_DEBUG
        printf("I2C (0x");
        printf("%x", i2c->devAddr);
        printf(") reading");
        printf("%d", length);
        printf(" words from 0x");
        printf("%x", regAddr);
        printf("...");
    #endif

    uint8_t i = 0;
    int8_t count = 0;
    // Boundary Check
    if(length > BUFFER_LENGTH){
        length = BUFFER_LENGTH;
        // error message handling
#ifdef I2CLIB_DEBUG
        fprintf(stderr, "readWords data length is enforced as BUFFER_LENGTH!\n"); 
#endif
    }else if(length < 0){
        length = 0; 
        // error message handling
#ifdef I2CLIB_DEBUG
        fprintf(stderr, "readWords data length is enforced as 0!\n");
#endif
    }
//    uint32_t t1 = millis();
       uint8_t intermediate[(uint8_t)length*2];
       uint8_t status = 0;
        // write first 
        if(write(i2c->file, &regAddr, 1)!=1)
        /* error handling here */
          status = 1;
        // read later 
        if(read(i2c->file, data, length*2)!=length*2)
        /* error handling here */
          status = 1;  

       if(status == 0) {
       //if(i2c->buffer[i] != -1) {
          count = length;
          for(i = 0; i < length; i++){
              data[i] = (((uint16_t)intermediate[2*i]) << 8) | intermediate[2*i + 1]; 
          }
       } else {
           count = -1 * status;
           //count = -1 * i2c->buffer[i];
           // error message handling
#ifdef I2CLIB_DEBUG
           fprintf(stderr, "invalid readWords 0x%x reading!\n", regAddr);
#endif
       }

//    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout
    #ifdef I2CDEV_DEBUG
        printf(". Done.");
        printf("%d", count);
        printf(" read).");
    #endif
    return count;
}

/** Read single word from a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readWord(i2c_t* i2c, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(i2c, regAddr, 1, data, timeout);
}

/** Read a single bit from an 8-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBit(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(i2c, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */

int8_t readBitW(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(i2c, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBits(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(i2c, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t readBitsW(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(i2c, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Write multiple bytes to an 8-bit device register.
 * @param [in] i2c pointer to i2c struct contains slave device handle
 * @param [in] regAddr First register address to write to
 * @param [in] length Number of bytes to write
 * @param [in] data Buffer to copy new data from
 * @return Status of operation (true = success)
 */

// i2c_smbus_write_byte_data executes the SMBUS "write byte" Protocol, returning
// negative errno else zero on success 

//int writeBytes(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint8_t* data){
bool writeBytes(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint8_t* data){
#ifdef I2CDEV_DEBUG
  printf("I2C (0x");
  printf("%x", i2c->devAddr);
  printf(") writing");
  printf("%d", length);
  printf(" bytes to 0x");
  printf("%x", regAddr);
  printf("...");
#endif
  int32_t status = 0;
  uint8_t i = 0;

  uint8_t writeData[(uint8_t)(length+1)]; 
  writeData[0] = regAddr; 
  for(i = 0; i < length; i++)
    writeData[i + 1] = data[i]; 

    if(write(i2c->file, writeData, length+1) != length+1){
      status = 1;
#ifdef I2CLIB_DEBUG
      fprintf(stderr, "writeBytes 0x%x error occurred\n", regAddr);
#endif
    }
#ifdef I2CDEV_DEBUG
    for (i=0; i<length; i++)
      printf("%x\t", data[i]);
    printf("\n");
 //   if(i + 1 < length) printf(" ");
#endif 
#ifdef I2CDEV_DEBUG
  printf(". Done.");
#endif
  return status == 0;
}

/** Write single byte to an 8-bit device register.
 * @param [in] i2c pointer to i2c struct contains slave device handle
 * @param [in] regAddr Register address to write to
 * @param [in] data New byte value to write
 * @return Status of operation (true = success)
 */
bool writeByte(i2c_t* i2c, uint8_t regAddr, uint8_t data) {
    return writeBytes(i2c, regAddr, 1, &data);
}


/** Write multiple words to a 16-bit device register.
 * @param [in] i2c pointer to i2c struct contains slave device handle
 * @param [in] regAddr First register address to write to
 * @param [in] length Number of words to write
 * @param [in] data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool writeWords(i2c_t* i2c, uint8_t regAddr, uint8_t length, uint16_t* data) {
#ifdef I2CDEV_DEBUG
  printf("I2C (0x");
  printf("%x", i2c->devAddr);
  printf(") writing");
  printf("%d", length);
  printf(" words to 0x");
  printf("%x", regAddr);
  printf("...");
#endif
    // No boundary Check
    // Function prototype:
    // (uint8_t)(data[i++]) // send MSB 
    // (uint8_t)(data[i]) // send LSB 
   uint8_t status = 0;
   uint8_t i = 0;

   // combining write data
   uint8_t writeData[(uint8_t)(length*2 + 1)];
   writeData[0] = regAddr;
   for (i = 0; i<length; i++){
      writeData[i*2 + 1] = (uint8_t)(data[i] >> 8);
      writeData[i*2 + 2] = (uint8_t)(data[i]);
   }

    if(write(i2c->file, writeData, length*2 + 1) != (length*2 +1))
      status = 1;
     // Error handling here
#ifdef I2CLIB_DEBUG
      fprintf(stderr, "writeWords 0x%x error occurred\n", regAddr);
#endif     
#ifdef I2CDEV_DEBUG
    for (i = 0; i<length; i++)
      printf("%x\t", data[i]);
    printf("\n");
#endif

#ifdef I2CDEV_DEBUG
   printf(". Done.");
#endif
   return status == 0;
}

/** Write single word to an 8-bit device register.
 * @param [in] i2c pointer to i2c struct contains slave device handle
 * @param [in] regAddr Register address to write to
 * @param [in] data New byte value to write
 * @return Status of operation (true = success)
 */
bool writeWord(i2c_t* i2c, uint8_t regAddr, uint16_t data) {
    return writeWords(i2c, regAddr, 1, &data);
}

/** write a single bit in an 8-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBit(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    // Using default timeout value
    readByte(i2c, regAddr, &b, DEFAULT_READ_TIMEOUT);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(i2c, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBitW(i2c_t* i2c, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(i2c, regAddr, &w, DEFAULT_READ_TIMEOUT);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(i2c, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBits(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(i2c, regAddr, &b, DEFAULT_READ_TIMEOUT) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(i2c, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param i2c I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBitsW(i2c_t* i2c, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(i2c, regAddr, &w, DEFAULT_READ_TIMEOUT) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(i2c, regAddr, w);
    } else {
        return false;
    }
}
