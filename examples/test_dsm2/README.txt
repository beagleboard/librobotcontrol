test_dsm2
James Strawson - 2014

Project Description:
Running the test_dsm2 executable will print out the normalized dsm2 receiver values data to the terminal. Make sure the transmitter and receiver are paired before testing. Use the pair_dsm2 example if you don't have a bind plug and standard receiver. The satellite receiver remembers which transmitter it is paired to, not your BeagleBone. 

If the values you read are not normalized between +-1, then you should run the calibrate_dsm2 example to save your particular transmitter's min and max channel values.
