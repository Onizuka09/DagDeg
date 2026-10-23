# DagDeg
 DagDeg is a line follwing Robot.  
![pic](docs/robot_pic.png)

We create this line follower robot to participate in the ROBOCup 2024 competitions organized by ENSI. 
You can find the Robot datasheet and design here: [link](docs/Technical_docs.pdf)
# Robot Creation 
To detect the line we used a combination of QTR8A IR Sensor and 2 digital IR sensors. 
![map](docs/map.png)
later We figure out that we need another 2 IR sensors, but we didn't have the time nor the sensors to add. 

As seen in this [video](docs/robot_vid.mp4),the robot couldn't pass the hexagonal section. 

# The logic 
So we used A pid algorithm for line following and a state machine to help us navigate the different sections of the map.
in each state we manipulate the error of the far left and right IR sensor, so we can either increase or decrease the speed to have sharp and smooth turns, this was the objective. 

The source code is organized in to separate modules and a main file that implements the state machine. 
- `Bluetooth_conf` modules implements the bluetooth handler and BL initiation, used to configure the PID values and the robot state via a mobile app we developed using MIT app inventor.
- `IR_sensor` module: implements the initiation and implementation of both the PID and IR sensors
- `motot_control` module: implements the initiation and the configuration of the motor 
- `state.h` contains the states definition.


# What we did
We lost, once we put the robot on the map it didn't even follow the track and get out. 
the problem was the digital sensors we used, their calibration was via a resistor (hardware not software). the sensors were not calibrated on the competition map, and since we didn't have access to it we couldn't calibrate the sensor. 
- The outcome is we need analog sensor so we can implement our own calibration algorithm in code. 
It was a fruitful experience, we learned a lot, the next one will try to fix the issues and see what happen. 
## Update 
update at line 13 in the QTRlibrary, changes to  `_maxValue=4095`
, this is done to update the resolution to support 12 bit

