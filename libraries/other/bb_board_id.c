/*******************************************************************************
* Board identification
*
* Because we wish to support different beagleboard products with this same
* library, we must internally determine which board we are running on to decide
* which pins to use. We make these functions available to the user in case they
* wish to do the same. 
* See the check_board example for a demonstration.
*******************************************************************************/

#include "../usefulincludes.h"
#include "../roboticscape.h"

#define MODEL_DIR "/proc/device-tree/model"
#define BUF_SIZE 128

// current board stored in memory as enum for fast access
bb_board_t board;

// global variable always initialized at 0
// set to 1 once the board id has been pulled from /proc/
int has_checked; 


/*******************************************************************************
* return global variable 'board' from device tree
* then store it for later use
*******************************************************************************/
bb_board_t get_bb_board_from_device_tree(){
	char c[BUF_SIZE];
    FILE *fd;

    if ((fd = fopen(MODEL_DIR, "r")) == NULL)
    {
        printf("ERROR: can't open %s \n", MODEL_DIR);
        has_checked = 1;
		return UNKNOWN_BOARD;     
    }

    // read model
    memset(c, 0, BUF_SIZE);
    fgets(c, BUF_SIZE, fd);
    fclose(fd);

    // now do the checks
    if(		strcmp(c, "TI AM335x BeagleBone Black" 		   )==0) board=BB_BLACK;
    else if(strcmp(c, "TI AM335x BeagleBone Blue"		   )==0) board=BB_BLUE;
    else if(strcmp(c, "TI AM335x BeagleBone Black Wireless")==0) board=BB_BLACK_W;
    else if(strcmp(c, "TI AM335x BeagleBone Green"		   )==0) board=BB_GREEN;
    else if(strcmp(c, "TI AM335x BeagleBone Green Wireless")==0) board=BB_GREEN_W;
    else board = UNKNOWN_BOARD;

    // mark has-checked as 1 to prevent future slow checks
    has_checked = 1;
    return board;
}


/*******************************************************************************
* return global variable 'board' if already read from device tree
* otherwise get from device tree which stores it for later use
*******************************************************************************/
bb_board_t get_bb_board(){
	if(has_checked) return board;
	else return get_bb_board_from_device_tree();
}


/*******************************************************************************
* print global variable 'board'
* if it hasn't been checked yet, do so first.
*******************************************************************************/
void print_bb_board(){
	if(has_checked==0) get_bb_board_from_device_tree();

	switch(board){
	case(UNKNOWN_BOARD):
		printf("UNKNOWN_BOARD");
		break;
	case(BB_BLACK):
		printf("BB_BLACK");
		break;
	case(BB_BLACK_W):
		printf("BB_BLACK_W");
		break;
	case(BB_GREEN):
		printf("BB_GREEN");
		break;
	case(BB_GREEN_W):
		printf("BB_GREEN_W");
		break;
	case(BB_BLUE):
		printf("BB_BLUE");
		break;
	default:
		printf("ERROR: invalid case in print_bb_board()\n");
		break;
	}

	return;
}