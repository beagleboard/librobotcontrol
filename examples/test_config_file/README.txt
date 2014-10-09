Bare Minimum Skeleton for Robotics Cape Projects
James Strawson - 2013

Project Description:
This is meant to be a starting point for future projects. 
There are at least three files inside each project.

File Structure:
project_name.c	- This is your main source file containing 
Makefile        - You must update the first line of this with your new 
				  project name for make to recognize the new .c file
README.txt 	    - This is the file you are reading now. You should 
				  update this with your own project description and 
				  instructions.
				  
Compile Instructions:
make clean - Removes compiled object and executable files.
make - Links and compiles your source file into an executable. Run with "./project_name"
make install - Adds a copy of your executable to /usr/bin. Use this to keep a copy of your stable programs that can be executed from any directory.