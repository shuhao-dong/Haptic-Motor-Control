# Haptic-Motor-Control
Using ESP32E to control a voice coil actuator

Pleae find the latest version of the programme for parallel programming: Parallel function programme_Sept_2022

Update notes:
1. Fix the bug where the pointer could not locate the correct memory position of the DAC.
2. Introducing the parallel programming, allowing background stepper motor rotation.
3. Add manual rotation of the motor by the joystick y-axis output.

For pre-test and final test:  Test number and orientation assignment:
1. The Matlab programme is to assign the test number (1-9) and orientation assignment (0 or 1).
2. 'acceleration_data_processing' is the data analysis file for displacement data obtained from Optitrack. 

Update: 24/10/2022
1. Fix the baud rate being to low resulting in slower haptic motor motion. 

Update: 22/11/2022
1. Add IMU sensor reading function

Wait to do:
1. Build low pass filter for data analysis 
