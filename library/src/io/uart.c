/**
 * @file uart.c
 *
 *
 *
 * @author     James Strawson
 * @date       3/6/2018
 */

#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h> // for timeval
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <string.h>
#include <sys/ioctl.h>
#include <math.h>

#include <rc/uart.h>

#define MAX_BUS		16
#define STRING_BUF	64

// Most bytes to read at once. This is the size of the Sitara UART FIFO buffer.
#define MAX_READ_LEN	128


static int   rc_uart_fd[MAX_BUS+1]; // file descriptors for all ports
static float rc_uart_bus_timeout_s[MAX_BUS+1]; // user-requested timeout in seconds for each bus
static int   rc_uart_shutdown_flag[MAX_BUS+1];


int rc_uart_init(int bus, int baudrate, float timeout_s, int canonical_en, int stop_bits, int parity_en)
{
	int tmpfd, tenths;
	char buf[STRING_BUF];
	struct termios config;
	speed_t speed; //baudrate

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR in rc_uart_init, bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(timeout_s<0.1f){
		fprintf(stderr,"ERROR in rc_uart_init, timeout must be >=0.1 seconds\n");
		return -1;
	}
	if(stop_bits!=1 && stop_bits!=2){
		fprintf(stderr,"ERROR in rc_uart_init, stop bits must be 1 or 2\n");
	}

	switch(baudrate){
	case (230400):
		speed=B230400;
		break;
	case (115200):
		speed=B115200;
		break;
	case (57600):
		speed=B57600;
		break;
	case (38400):
		speed=B38400;
		break;
	case (19200):
		speed=B19200;
		break;
	case (9600):
		speed=B9600;
		break;
	case (4800):
		speed=B4800;
		break;
	case (2400):
		speed=B2400;
		break;
	case (1800):
		speed=B1800;
		break;
	case (1200):
		speed=B1200;
		break;
	case (600):
		speed=B600;
		break;
	case (300):
		speed=B300;
		break;
	case (200):
		speed=B200;
		break;
	case (150):
		speed=B150;
		break;
	case (134):
		speed=B134;
		break;
	case (110):
		speed=B110;
		break;
	case (75):
		speed=B75;
		break;
	case (50):
		speed=B50;
		break;
	default:
		fprintf(stderr,"ERROR: int rc_uart_init, invalid baudrate. Please use a standard baudrate\n");
		return -1;
	}

	// close the bus in case it was already open
	rc_uart_close(bus);

	// open file descriptor for blocking reads
	snprintf(buf,sizeof(buf),"/dev/ttyO%d",bus);
	tmpfd = open(buf, O_RDWR | O_NOCTTY | O_NDELAY);
	if(tmpfd==-1){
		perror("ERROR: int rc_uart_init while opening file descriptor");
		fprintf(stderr,"device tree probably isn't loaded\n");
		return -1;
	}

	// get current attributes
	if(tcgetattr(tmpfd,&config)==-1){
		fprintf(stderr,"ERROR: int rc_uart_init, Cannot get uart attributes\n");
		close(tmpfd);
		return -1;
	}

	// wipe tc_config and start setting flags
	memset(&config,0,sizeof(config));

	// the following lines technically do nothing since we just wiped config
	// but they exist to allow easy fiddling and be more explicit about
	// which settings are in use
	if(canonical_en) config.c_lflag |= ICANON;
	else config.c_lflag &= ~ICANON;
	if(parity_en) config.c_cflag |= PARENB;
	else config.c_cflag &= ~PARENB;
	if(stop_bits==1) config.c_cflag &= ~CSTOPB;	// disable 2 stop bits (use just 1)
	else config.c_cflag |= CSTOPB; // enable 2 stop bits

	config.c_cflag &= ~CSIZE;	// wipe all size masks
	config.c_cflag |= CS8;		// set size to 8 bit characters
	config.c_cflag |= CREAD;	// enable reading
	config.c_cflag |= CLOCAL;	// ignore modem status lines

	// convert float timeout in seconds to int timeout in tenths of a second
	tenths = (timeout_s*10);

	// if VTIME>0 & VMIN>0, read() will return when either the requested number
	// of bytes are ready or when VMIN bytes are ready, whichever is smaller.
	// since we set VMIN to the size of the buffer, read() should always return
	// when the user's requested number of bytes are ready.
	config.c_cc[VMIN]=MAX_READ_LEN;
	config.c_cc[VTIME] = tenths+1;

	// set speed in config struct
	if(cfsetispeed(&config, speed)==-1){
		perror("ERROR: in rc_uart_init calling cfsetispeed");
		close(tmpfd);
		return -1;
	}
	if(cfsetospeed(&config, speed)==-1){
		perror("ERROR: in rc_uart_init calling cfsetospeed");
		close(tmpfd);
		return -1;
	}

	// flush and set attributes
	if(tcflush(tmpfd,TCIOFLUSH)==-1){
		perror("ERROR: in rc_uart_init calling tcflush");
		close(tmpfd);
	}
	if(tcsetattr(tmpfd, TCSANOW, &config) < 0) {
		fprintf(stderr,"cannot set uart%d attributes\n", bus);
		close(rc_uart_fd[bus]);
		return -1;
	}
	if(tcflush(tmpfd,TCIOFLUSH)==-1){
		perror("ERROR: in rc_uart_init calling tcflush");
		close(tmpfd);
		return -1;
	}

	// turn off the FNDELAY flag
	if(fcntl(tmpfd, F_SETFL, 0)==-1){
		perror("ERROR: in rc_uart_init calling fcntl");
		close(tmpfd);
		return -1;
	}
	if(tcflush(tmpfd,TCIOFLUSH)==-1){
		perror("ERROR: in rc_uart_init calling tcflush");
		close(tmpfd);
		return -1;
	}

	rc_uart_fd[bus]=tmpfd;
	rc_uart_bus_timeout_s[bus]=timeout_s;
	rc_uart_shutdown_flag[bus]=0;
	return 0;
}


int rc_uart_close(int bus)
{
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: uart bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	rc_uart_shutdown_flag[bus]=1;
	// if not initialized already, return
	if(rc_uart_fd[bus]==0) return 0;
	// flush and close
	tcflush(rc_uart_fd[bus],TCIOFLUSH);
	close(rc_uart_fd[bus]);
	rc_uart_fd[bus]=0;
	return 0;
}


int rc_uart_get_fd(int bus)
{
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: in rc_uart_get_fd, bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: in rc_uart_get_fd, uart%d not initialized yet\n", bus);
		return -1;
	}
	return rc_uart_fd[bus];
}


int rc_uart_flush(int bus)
{
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: in rc_uart_flush, bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: in rc_uart_flush, uart%d must be initialized first\n", bus);
		return -1;
	}
	if(tcflush(rc_uart_fd[bus],TCIOFLUSH)==-1){
		perror("ERROR in rc_uart_flush:");
		return -1;
	}
	return 0;
}


int rc_uart_write(int bus, uint8_t* data, size_t bytes)
{
	int ret;
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: uart bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(bytes<1){
		fprintf(stderr,"ERROR: number of bytes to send must be >1\n");
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	ret=write(rc_uart_fd[bus], data, bytes);
	if(ret==-1) perror("ERROR in rc_uart_write");
	return ret;
}


int rc_uart_read_bytes(int bus, uint8_t* buf, size_t bytes)
{
	int bytes_to_read, ret;
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: uart bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(bytes<1){
		fprintf(stderr,"ERROR: number of bytes to read must be >=1\n");
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}

	/*
	// A single call to 'read' just isn't reliable, don't do it.
	// But if you really want to, this commented section is how you do it
	if(bytes<=MAX_READ_LEN){
		// small read, return in one read() call
		// this uses built-in timeout instead of select()
		ret = read(rc_uart_fd[bus], buf, bytes);
		return ret;
	}
	*/

	// any read under 128 bytes should have returned by now.
	// everything below this line is for longer extended reads >128 bytes
	fd_set set; // for select()
	struct timeval timeout;
	int bytes_read; // number of bytes read so far
	int bytes_left; // number of bytes still need to be read

	bytes_read = 0;
	bytes_left = bytes;

	// set up the timeout OUTSIDE of the read loop. We will likely be calling
	// select() multiple times and that will decrease the timeout struct each
	// time ensuring the TOTAL timeout requested by the user is honoured instead
	// of the timeout value compounding each loop.
	timeout.tv_sec = (int)rc_uart_bus_timeout_s[bus];
	timeout.tv_usec = (int)(1000000*fmod(rc_uart_bus_timeout_s[bus],1));

	// exit the read loop once enough bytes have been read
	// or the the shutdown signal flag is set
	while((bytes_left>0) && rc_uart_shutdown_flag[bus]==0){
		FD_ZERO(&set); /* clear the set */
		FD_SET(rc_uart_fd[bus], &set); /* add our file descriptor to the set */
		ret = select(rc_uart_fd[bus] + 1, &set, NULL, NULL, &timeout);
		if(ret == -1){
			// select returned and error. EINTR means interrupted by SIGINT
			// aka ctrl-c. Don't print anything as this happens normally
			// in case of EINTR/Ctrl-C just return how many bytes got read up
			// until then without raising alarms.
			if(errno!=EINTR){
				perror("ERROR in rc_uart_read_bytes calling select");
				return -1;
			}
			return bytes_read;
		}
		else if(ret == 0){
			// timeout
			return bytes_read;
		}
		else{
			// There was data to read. Read up to the number of bytes left
			// and no more. This most likely will return fewer bytes than
			// bytes_left, but we will loop back to get the rest.

			// read no more than MAX_READ_LEN at a time
			if(bytes_left>MAX_READ_LEN)	bytes_to_read = MAX_READ_LEN;
			else bytes_to_read = bytes_left;
			ret=read(rc_uart_fd[bus], buf+bytes_read, bytes_to_read);
			if(ret<0){
				perror("ERROR: in uart_read_bytes");
				return -1;
			}
			else if(ret>0){
				// success, actually read something
				bytes_read += ret;
				bytes_left -= ret;
			}
		}
	}
	return bytes_read;
}


int rc_uart_read_line(int bus, uint8_t* buf, size_t max_bytes)
{
	int ret;
	char temp;
	fd_set set;
	struct timeval timeout;
	int bytes_read=0;

	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: in rc_uart_read_line, bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(max_bytes<1){
		fprintf(stderr,"ERROR: in rc_uart_read_line, max_bytes must be >=1\n");
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: in rc_uart_read_line, uart%d must be initialized first\n", bus);
		return -1;
	}

	// set up the timeout OUTSIDE of the read loop. We will likely be calling
	// select() multiple times and that will decrease the timeout struct each
	// time ensuring the TOTAL timeout requested by the user is honoured instead
	// of the timeout value compounding each loop.
	timeout.tv_sec = (int)rc_uart_bus_timeout_s[bus];
	timeout.tv_usec = (int)(1000000*fmod(rc_uart_bus_timeout_s[bus],1));

	// exit the read loop once enough bytes have been read
	// or the shutdown flag is set
	while(bytes_read<(signed)max_bytes && rc_uart_shutdown_flag[bus]==0){
		FD_ZERO(&set); /* clear the set */
		FD_SET(rc_uart_fd[bus], &set); /* add our file descriptor to the set */
		ret = select(rc_uart_fd[bus] + 1, &set, NULL, NULL, &timeout);
		if(ret==-1){
			// select returned and error. EINTR means interrupted by SIGINT
			// aka ctrl-c. Don't print anything as this happens normally
			// in case of EINTR/Ctrl-C just return how many bytes got read up
			// until then without raising alarms.
			if(errno!=EINTR){
				perror("ERROR in rc_uart_read_line calling select");
				return -1;
			}
			return bytes_read;
		}
		else if(ret == 0){
			// timeout
			return bytes_read;
		}
		else{
			// There was data to read. Read one byte;
			ret=read(rc_uart_fd[bus], &temp, 1);
			if(ret<0){
				perror("ERROR in rc_uart_read_line calling read");
				return -1;
			}
			else if(ret==1){
				// success, actually read something
				if(temp=='\n') return bytes_read;
				else{
					*(buf+bytes_read)=temp;
					bytes_read++;
				}
			}
		}
	}
	return bytes_read;
}


int rc_uart_bytes_available(int bus)
{
	int out;
	// sanity checks
	if(bus<0 || bus>MAX_BUS){
		fprintf(stderr,"ERROR: uart bus must be between 0 & %d\n", MAX_BUS);
		return -1;
	}
	if(rc_uart_fd[bus]==0){
		fprintf(stderr,"ERROR: uart%d must be initialized first\n", bus);
		return -1;
	}
	if(ioctl(rc_uart_fd[bus], FIONREAD, &out)==-1){
		perror("ERROR in rc_uart_bytes_available calling ioctl");
		return -1;
	}
	return out;
}
