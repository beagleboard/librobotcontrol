/*******************************************************************************
* gps.c
*
* This file contains all gps related functions and is compiled into 
* robotics_cape.so but kept here separately for tidyness. This references 
* the nmealib.
*
*******************************************************************************/

#include "../roboticscape-usefulincludes.h"
#include "../roboticscape.h"
#include "../roboticscape-defs.h"
#include "../nmealib/nmea/nmea.h"


#define GPS_UART_BUS 		2
#define GPS_UART_TIMEOUT	1.0
#define GPS_BUFFER_SIZE		128

/*******************************************************************************
* Local Global Variables
*******************************************************************************/
int running;
int is_new_gps_data;
int is_gps_active_flag;
pthread_t gps_listener_thread;

//NMEA Structs
nmeaINFO info;
nmeaPARSER parser;
nmeaPOS dpos;

/*******************************************************************************
* Local Function Declarations
*******************************************************************************/
void* gps_listener(void *ptr); //background thread

/*******************************************************************************
* int initialize_gps(int baud)
* 
* returns -1 for failure or 0 for success
* This starts the background thread listener which listens
* for serials packets on that interface.
*******************************************************************************/ 
int initialize_gps(int baud){
	
	if(initialize_uart(GPS_UART_BUS, baud, GPS_UART_TIMEOUT)){
		printf("Error, failed to initialize UART%d for GPS\n", GPS_UART_BUS);
		return -1;
	}
	
	pthread_create(&gps_listener_thread, NULL, gps_listener, (void*) NULL);
	is_new_gps_data = 0;

	#ifdef DEBUG
	printf("GPS Thread Started\n");
	#endif
	
	return 0;
}




/*******************************************************************************
* @ void* gps_listener(void *ptr)
* 
* This is a local function that is started as a background thread by 
* initialize_gps(). This monitors the serial port and interprets data
* for each packet.
*******************************************************************************/
void* gps_listener(void *ptr){
	char buf[GPS_BUFFER_SIZE]; // large serial buffer to catch doubled up packets
	int ret;
	

	//Zero out NMEA settings
    nmea_zero_INFO(&info);
    nmea_parser_init(&parser);

    // flush the hardware buffer
	flush_uart(GPS_UART_BUS);
	
	// running will become 0 when stop_gps_service() is called
	// or cleanup_cape() will set state to exiting
	running = 1;
	while(running && rc_get_state()!=EXITING){
		memset(&buf, 0, sizeof(buf)); // clear buffer
		// read the buffer and decide what to do
		//ret = uart_read_line(GPS_UART_BUS, GPS_BUFFER_SIZE, buf);
		ret = uart_read_bytes(GPS_UART_BUS, GPS_BUFFER_SIZE, buf);
		if(ret<0){ //error
			printf("ERROR reading uart %d\n", GPS_UART_BUS);
			printf("stopping gps listener\n");
			running=0;
			goto END;
		}
		else if(ret==0){ //timeout
			#ifdef DEBUG
				printf("GPS Timeout\n");
			#endif
			is_gps_active_flag=0; // indicate connection is no longer active
			continue;
		}
		
		#ifdef DEBUG
			printf("GPS read %d bytes, ", ret);
		#endif
		
		//Parse raw NMEA Data
        nmea_parse(&parser, buf, ret, &info);

        //Converts lat and long data in .info struct to radians 
        nmea_info2pos(&info, &dpos);

        printf("Lat: %f, Lon: %f, Sig: %d, Fix: %d\n", info.lat, info.lon, info.sig, info.fix);

        is_new_gps_data = 1;
        is_gps_active_flag = 1;

END: ;
	}

	#ifdef DEBUG
		printf("exiting gps_listener_thread\n");
	#endif

	return NULL;
}

/*******************************************************************************
* @ int stop_gps_service()
* 
* signals the serial_parser_thread to stop and allows up to 1 second for the 
* thread to  shut down before returning.
*******************************************************************************/
int stop_gps_service(){
	int ret = 0;
	
	if (running){
		running = 0; // this tells serial_parser_thread loop to stop
		// allow up to 1.5 seconds for thread cleanup
		timespec thread_timeout;
		clock_gettime(CLOCK_REALTIME, &thread_timeout);
		timespec_add(&thread_timeout, 1.5);
		int thread_err = 0;
		thread_err = pthread_timedjoin_np(gps_listener_thread, NULL, 
															   &thread_timeout);
		if(thread_err == ETIMEDOUT){
			printf("WARNING: gps_listener_thread exit timeout\n");
			ret = -1;
		}
	}
	
	running = 0;
	return ret;
}


