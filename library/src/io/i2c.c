/**
 * @file i2c.c
 *
 * @author James Strawson
 */

#include <stdint.h> // for uint8_t types etc
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs

#include <rc/i2c.h>

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)


/**
 * contains the current state of a bus. you don't need to create your own
 * instance of this, one for each bus is allocated here
 */
typedef struct rc_i2c_state_t {
	/* data */
	uint8_t devAddr;
	int fd;
	int initialized;
	int lock;
} rc_i2c_state_t;

static rc_i2c_state_t i2c[I2C_MAX_BUS+1];


// local function
static int __check_bus_range(int bus)
{
	if(unlikely(bus<0 || bus>I2C_MAX_BUS)){
		fprintf(stderr,"ERROR: i2c bus must be between 0 & %d\n", I2C_MAX_BUS);
		return -1;
	}
	return 0;
}


int rc_i2c_init(int bus, uint8_t devAddr)
{
	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;

	// if already initialized just set the device address
	if(i2c[bus].initialized){
		return rc_i2c_set_device_address(bus, devAddr);
	}

	// lock the bus during this operation
	i2c[bus].lock = 1;
	i2c[bus].initialized = 0;

	// open file descriptor
	char str[16];
	sprintf(str,"/dev/i2c-%d",bus);
	i2c[bus].fd = open(str, O_RDWR);
	if(i2c[bus].fd==-1){
		fprintf(stderr,"ERROR: in rc_i2c_init, failed to open /dev/i2c\n");
		return -1;
	}

	// set device adress
	if(unlikely(ioctl(i2c[bus].fd, I2C_SLAVE, devAddr)<0)){
		fprintf(stderr,"ERROR: in rc_i2c_init, ioctl slave address change failed\n");
		return -1;
	}
	i2c[bus].devAddr = devAddr;
	// return the lock state to previous state.
	i2c[bus].lock = 0;
	i2c[bus].initialized = 1;
	return 0;
}


int rc_i2c_close(int bus)
{
	if(unlikely(__check_bus_range(bus))) return -1;
	close(i2c[bus].fd);
	i2c[bus].devAddr = 0;
	i2c[bus].initialized = 0;
	i2c[bus].lock=0;
	return 0;
}



int rc_i2c_set_device_address(int bus, uint8_t devAddr)
{
	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_set_device_address, bus not initialized yet\n");
		return -1;
	}
	// if the device address is already correct, just return
	if(i2c[bus].devAddr == devAddr){
		return 0;
	}
	// if not, change it with ioctl
	if(unlikely(ioctl(i2c[bus].fd, I2C_SLAVE, devAddr)<0)){
		fprintf(stderr,"ERROR: in rc_i2c_set_device_address, ioctl slave address change failed\n");
		return -1;
	}
	i2c[bus].devAddr = devAddr;
	return 0;
}


int rc_i2c_read_byte(int bus, uint8_t regAddr, uint8_t *data)
{
	return rc_i2c_read_bytes(bus, regAddr, 1, data);
}


int rc_i2c_read_bytes(int bus, uint8_t regAddr, size_t count, uint8_t *data)
{
	int ret, old_lock;

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_read_bytes, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation, preserving old state to return to
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// write register to device
	ret = write(i2c[bus].fd, &regAddr, 1);
	if(unlikely(ret!=1)){
		fprintf(stderr,"ERROR: in rc_i2c_read_bytes, failed to write to bus\n");
		i2c[bus].lock = old_lock;
		return -1;
	}

	// then read the response
	ret = read(i2c[bus].fd, data, count);
	if(unlikely((size_t)ret!=count)){
		fprintf(stderr,"ERROR: in rc_i2c_read_bytes, received %d bytes from device, expected %d\n", ret, (int)count);
		i2c[bus].lock = old_lock;
		return -1;
	}

	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return ret;


}


int rc_i2c_read_word(int bus, uint8_t regAddr, uint16_t *data)
{
	return rc_i2c_read_words(bus, regAddr, 1, data);
}


int rc_i2c_read_words(int bus, uint8_t regAddr, size_t count, uint16_t *data)
{
	int ret, old_lock;
	size_t i;
	char buf[count*2];

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_read_words, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// write register to device
	ret = write(i2c[bus].fd, &regAddr, 1);
	if(unlikely(ret!=1)){
		fprintf(stderr,"ERROR: in rc_i2c_read_words, failed to write to bus\n");
		i2c[bus].lock = old_lock;
		return -1;
	}

	// then read the response
	ret = read(i2c[bus].fd, buf, count*2);
	if(ret!=(signed)(count*2)){
		fprintf(stderr,"ERROR: in rc_i2c_read_words, received %d bytes, expected %zu\n", ret, count*2);
		i2c[bus].lock = old_lock;
		return -1;
	}

	// form words from bytes and put into user's data array
	for(i=0;i<count;i++){
		data[i] = (((uint16_t)buf[i*2])<<8 | buf[(i*2)+1]);
	}

	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return 0;
}





int rc_i2c_write_bytes(int bus, uint8_t regAddr, size_t count, uint8_t* data)
{
	int ret, old_lock;
	size_t i;
	uint8_t writeData[count+1];

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_write_bytes, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// assemble array to send, starting with the register address
	writeData[0] = regAddr;
	for(i=0; i<count; i++) writeData[i+1]=data[i];

	// send the bytes
	ret = write(i2c[bus].fd, writeData, count+1);
	// write should have returned the correct # bytes written
	if(unlikely(ret!=(signed)(count+1))){
		fprintf(stderr,"ERROR in rc_i2c_write_bytes, bus wrote %d bytes, expected %zu\n", ret, count+1);
		i2c[bus].lock = old_lock;
		return -1;
	}
	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return 0;
}


int rc_i2c_write_byte(int bus, uint8_t regAddr, uint8_t data)
{
	int ret, old_lock;
	uint8_t writeData[2];

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_write_byte, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// assemble array to send, starting with the register address
	writeData[0] = regAddr;
	writeData[1] = data;

	// send the bytes
	ret = write(i2c[bus].fd, writeData, 2);

	// write should have returned the correct # bytes written
	if(unlikely(ret!=2)){
		fprintf(stderr,"ERROR: in rc_i2c_write_byte, system write returned %d, expected 2\n", ret);
		i2c[bus].lock = old_lock;
		return -1;
	}
	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return 0;
}


int rc_i2c_write_words(int bus, uint8_t regAddr, size_t count, uint16_t* data)
{
	int ret,old_lock;
	size_t i;
	uint8_t writeData[(count*2)+1];

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_write_words, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// assemble bytes to send
	writeData[0] = regAddr;
	for (i=0; i<count; i++){
		writeData[(i*2)+1] = (uint8_t)(data[i] >> 8);
		writeData[(i*2)+2] = (uint8_t)(data[i] & 0xFF);
	}

	ret = write(i2c[bus].fd, writeData, (count*2)+1);
	if(unlikely(ret!=(signed)(count*2)+1)){
		fprintf(stderr,"ERROR: in rc_i2c_write_words, system write returned %d, expected %zu\n", ret, (count*2)+1);
		i2c[bus].lock = old_lock;
		return -1;
	}
	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return 0;
}


int rc_i2c_write_word(int bus, uint8_t regAddr, uint16_t data)
{
	int ret,old_lock;
	uint8_t writeData[3];

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_write_words, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	old_lock = i2c[bus].lock;
	i2c[bus].lock = 1;

	// assemble bytes to send from data casted as uint8_t*
	writeData[0] = regAddr;
	writeData[1] = (uint8_t)(data >> 8);
	writeData[2] = (uint8_t)(data & 0xFF);

	ret = write(i2c[bus].fd, writeData, 3);
	if(unlikely(ret!=3)){
		fprintf(stderr,"ERROR: in rc_i2c_write_word, system write returned %d, expected 3\n", ret);
		i2c[bus].lock = old_lock;
		return -1;
	}
	// return the lock state to previous state.
	i2c[bus].lock = old_lock;
	return 0;
}


int rc_i2c_send_byte(int bus, uint8_t data)
{
	return rc_i2c_send_bytes(bus,1,&data);
}


int rc_i2c_send_bytes(int bus, size_t count, uint8_t* data)
{
	int ret;

	// sanity check
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_send_bytes, bus not initialized yet\n");
		return -1;
	}

	// lock the bus during this operation
	int old_lock= i2c[bus].lock;
	i2c[bus].lock = 1;

	// send the bytes
	ret = write(i2c[bus].fd, data, count);
	// write should have returned the correct # bytes written
	if(ret!=(signed)count){
		fprintf(stderr,"ERROR: in rc_i2c_send_bytes, system write returned %d, expected %zu\n", ret, count);
		i2c[bus].lock = old_lock;
		return -1;
	}

	// return the lock state to previous state.
	i2c[bus].lock = old_lock;

	return 0;
}






int rc_i2c_lock_bus(int bus)
{
	if(unlikely(__check_bus_range(bus))) return -1;
	int ret=i2c[bus].lock;
	i2c[bus].lock=1;
	return ret;
}


int rc_i2c_unlock_bus(int bus)
{
	if(unlikely(__check_bus_range(bus))) return -1;
	int ret=i2c[bus].lock;
	i2c[bus].lock=0;
	return ret;
}


int rc_i2c_get_lock(int bus)
{
	if(unlikely(__check_bus_range(bus))) return -1;
	return i2c[bus].lock;
}


int rc_i2c_get_fd(int bus) {
	if(unlikely(__check_bus_range(bus))) return -1;
	if(unlikely(i2c[bus].initialized==0)){
		fprintf(stderr,"ERROR: in rc_i2c_get_fd, bus not initialized yet\n");
		return -1;
	}
	return i2c[bus].fd;
}


#ifdef RC_AUTOPILOT_EXT
int rc_i2c_read_data(int bus, uint8_t regAddr, size_t length, uint8_t *data)
{
	return rc_i2c_read_bytes(bus, regAddr, length, data);
}
#endif // RC_AUTOPILOT_EXT