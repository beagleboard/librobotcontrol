/**
 * <rc_usefulincludes.h>
 *
 * This may be used as an "uber-include" at the beginning of a c file to include
 * the most common C libs without having a cluttered list. This is technically
 * bad practice and you should include only the headers necessary for your
 * program to operate. However, this can save time while quickly prototyping and
 * remains for backwards compatability.
 *
 * @addtogroup Useful_Includes
 * @ingroup Deprecated
 * @{
 */


#ifndef RC_USEFUL_INCLUDES
#define RC_USEFUL_INCLUDES


#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>	// for uint8_t types etc
#include <sys/stat.h>
#include <time.h>	// usleep, nanosleep
#include <math.h>	// atan2 and fabs
#include <signal.h>	// capture ctrl-c
#include <pthread.h>	// multi-threading
#include <linux/input.h>// buttons
#include <poll.h>	// interrupt events
#include <sys/mman.h>	// mmap for accessing eQep
#include <sys/socket.h>	// udp socket
#include <netinet/in.h>	// udp socket
#include <sys/time.h>
#include <arpa/inet.h>	// udp socket
#include <ctype.h>	// for isprint()
#include <sys/select.h>	// for read timeout


// Useful Constants
#ifndef DEG_TO_RAD
#define DEG_TO_RAD	0.0174532925199
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG	57.295779513
#endif

#ifndef PI
#define PI		M_PI
#endif

#ifndef TWO_PI
#define TWO_PI		(2.0 * M_PI)
#endif


// Useful Macros
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])
#endif

#ifndef min
#define min(a, b) 	((a < b) ? a : b)
#endif

#endif // RC_USEFUL_INCLUDES

///@} end group Deprecated
