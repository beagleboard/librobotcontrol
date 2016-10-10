/*******************************************************************************
* check_model.c
*
* James Strawson 2016
* print to the screen a human-readable string version of the bb_board_t enum
* of the board that this code is currently running on.
*******************************************************************************/

#include "../../libraries/usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	printf("\nCurrently running on a:\n");
	print_bb_model();
	printf("\n");
	return 0;
}
