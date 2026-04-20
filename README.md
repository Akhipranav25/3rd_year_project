# 3rd_year_project
Autonomous docking system for spacecraft
<br>
This repository contains all CAD and code files for the project.
<br>
The C++ code (arduino_robot_control_code_9.4.ino) is to be run on an Arduino microcontroller and the Python code (distance+pose+estimation+Arduino_v6.4.py) is intended to be run on a Raspberry Pi (recommended Raspberry Pi 4 Model B). The Arduino needs to be connected to the Raspberry Pi via USB for the programs to work. The Arduino program can work using inputs typed in the serial monitor on the Arduino IDE, but the Python program will not run if the Arduino is not connected. The Arduino does not have to have the corresponding program loaded, but there will likely be input buffer overflows if a different program (or no program) is used.
<br>
The STL files can be directly imported to a slicing software and 3D printed, while the STEP files can be imported into a 3D modelling software and edited. The PT-GD401.step file is the original scissor lift design from the source (GrabCAD [https://grabcad.com/library/stepper-motorized-labjack-pt-gd401-and-pt-gd402-1]) and can be edited or just used as a guide for how to connect the parts. Note that some parts, particularly the top and bottom covers are not critical and the design will function without them.
