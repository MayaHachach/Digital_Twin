Before building:

install the Eigen3 and tf2 using the following commands:
sudo apt install libeigen3-dev
sudo apt install libtf2-dev

To make sure the above library were installed, you can check the folder: /usr/include
on terminal: cd /usr/include 
and then u can examine the folders and files

-----------------------------------------------------------------------------------------

Before running:
- go to the param file and edit the following information:

-----------------------------------------------------------------------------------------

How to run:
- build the system using colcon build

If you dont have Day 0 information:
- run the day0 creation standalone node using the following command:\
ros2 run communication day0_creation

After having the day0 information:
- run the launch file: ros2 launch communication communication.launch.py

to communicate with hololens:
- run in your terminal: hostname -I and copy the ip address
- open ROS-TCP-Endpoint-main-ros2/launch/endpoint.py
- change the 'ROS_IP' default value to the copied value.
- run the launch file using: ros2 launch ros_tcp_endpoint endpoint.py

-----------------------------------------------------------------------------------------

Main functionalities and how to use them:

1- Day 0 extraction:

2- Automatic update:

3- Offline object handling:

4- Human Correction:
To start human correction, first, you need to send the current objects with their corresponding locations to the XR-Agent,
thus you need to run the request_STOD_server (exists in the launch file), and request_STOD_client from the XR-Agent.

Note: Human correction should only be applied on objects while in static state (cant be applied on objects while moving)

5- Status Monitoting:

For adding a new robot type:

6- Logging:

The logging process is done automatically on each update in the digital twin. 
The logged information include timestamp, all objects, each with their corresponding class name, id, pose, topic name, and status. 
The json file is saved in the share folder of the package, to access the json file, go to:
cd {path to workspace}/Digital_twin/install/communication/share/communication
To request the history, you need to run the "request_history_server" node and run the request_history_client (exists in the launch file)

7- Day to day update:
If the system was turned off from day to day, upon omniverse initialization and system initialization, you need to send the recent object locations to omniverse, 
for that you need to run the request_STOD_server (exists in the launch file), and request_STOD_client from omniverse.
Note: the STOD is sent to omniverse only once per system run,
        if for any reason, you need to resend the STOD again to omniverse, rerun the ros system (ctrl + c)

8- Launch file:
The launch file runs the following nodes:
- communicator: communicates with omniverse and XR-agent at all times, manages updates and logs in the json file per update.
- request_history_server: sends the json file to omniverse for replay.
- request_STOD_server: send the most recent object locations for omniverse upon initialization and for the XR-Agent upon request.

