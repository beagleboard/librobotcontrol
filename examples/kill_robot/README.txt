kill_robot
James Strawson - 2013

Project Description:
Force quits the last running process using the robotics cape resources.
Opening any other program containing initialize_cape() will send SIGINT
to the last running process and create a new lockfile.

Compile Instructions:
make clean - Removes compiled object and executable files.
make - Links and compiles your source file into an executable. Run with "./project_name"
make install - Adds a copy of your executable to /usr/bin. Use this to keep a copy of your stable programs that can be executed from any directory.