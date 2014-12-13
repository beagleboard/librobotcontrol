kill_robot
James Strawson - 2014

Project Description:
Quits the last running process that used the initialize_cape() function with the SIGINT signal. If a project is started in the background, such as automatically in Auto_Run_Script.sh, then you can use kill_robot to stop it without finding the process ID. Similarly, opening any other program containing initialize_cape() will send SIGINT to the last running process just like kill_robot.

This works because the initialize_cape() function creates a lockfile /tmp/robotics.lock with the current process ID. This is the same PID that the linux resource monitor program "top" gives you. When a robotics project shuts down cleanly with a cleanup_cape() function call, this file is removed. If it still exists, the kill_robot or any other example program will send the SIGINT signal to tell it to shut down before starting itself. This makes sure there are no conflicting uses of robotics cape resources.

Compile Instructions:
make clean - Removes compiled object and executable files.
make - Links and compiles your source file into an executable.
make install - Adds a copy of your executable to /usr/bin. Use this to keep a copy of your stable programs that can be executed from any directory.