test_config_file
James Strawson - 2014

Project Description:
When writing and testing new code, one often sets constants in the C code as #defines. Unfortunately this requires you to recompile each time you change a constant. Alternatively, you can create a config file which is loaded each time your program starts. This example demonstrates how to define a set of configuration values in a struct and load it when the program starts. 

The config file that is created is "/root/robot_config/test_config.txt". For convenience, I suggest keeping your config files in the same place for each project.