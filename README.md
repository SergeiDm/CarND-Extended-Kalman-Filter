# CarND-Extended-Kalman-Filter-Project
## Project Description
This project illustrates Sensor Fusion flow based on an Extended Kalman Filter for tracking an object.The Flow includes prediction and updating steps. The last one uses data from LIDAR and RADAR.

## Project files
The project includes the following folder/files:
- input_data – the folder with samples of measurement data from both LIDAR and RADAR.
- output_data - the folder with output files, produced by the project pipeline.
- Input_Output_Data_Format.txt - description input and output files formats.
- CMakeLists.txt - the file for building program.
- Source - the folder with c++ files with Extended Kalman Filter algorithm.

## Compiling and running the project
The project can be compiled and run by using the following command: 
`mkdir build && cd build`
`cmake .. && make`
`./ExtendedKF path/to/input.txt path/to/output.txt`

## Results
The output results introduced in "output_data" folder. Additionaly, here are RMSE values (calculated for 2D position and 2D velocity) for data 1:
- 0.065165
- 0.0605294
- 0.5497
- 0.544984

and data 2:
- 0.185692
- 0.190208
- 0.47926
- 0.827925
