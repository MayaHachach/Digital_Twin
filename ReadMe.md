# Digital Twin Communication Package

## Overview
The Communication package is a ROS 2 package that serves as the backbone for the Digital Twin system, facilitating real-time communication between physical robots, the digital twin environment, and human operators through XR interfaces. It provides robust services for object tracking, state management, and historical data logging.

## Environment

- **Operating System**: Ubuntu 22.04 LTS  
- **ROS 2 Distribution**: Humble

## Prerequisites

### Required Dependencies
```bash
# Install Eigen3
sudo apt install libeigen3-dev

# Install tf2
sudo apt install libtf2-dev
```

Verify the installations by checking the `/usr/include` directory:
```bash
cd /usr/include
# Verify eigen3 and tf2 directories exist
```

## Installation and Setup

1. **Day 0 Information Setup**
   - If you don't have Day 0 information:
     ```bash
     ros2 run communication day0_creation
     ```
   - If you have Day 0 information, skip this step and run the initial Configuration that subscribers to the day 0 information published by the visualizer on /initialSTOD topic

2. **Initial Configuration**
   ```bash
   # Run the setup node
   ros2 run communication setup_node
   ```
   - A configuration file will be auto-generated at `/Digital_Twin/src/communication/config/robots.yaml`
   - Edit this file to specify:
     - Odometry topic names
     - Status topic names
     - Note: Topics starting with "e.g." are ignored

3. **Build and Source**
   ```bash
   # Build the workspace
   colcon build

   # Source the workspace
   source install/setup.bash
   ```

## Running the System

### Basic Launch
```bash
ros2 launch communication communication.launch.py
```

### HoloLens Integration
1. Get your IP address:
   ```bash
   hostname -I
   ```
2. Update the ROS-TCP-Endpoint configuration:
   - Open `ROS-TCP-Endpoint-main-ros2/launch/endpoint.py`
   - Set `ROS_IP` to your IP address
3. Launch the endpoint:
   ```bash
   ros2 launch ros_tcp_endpoint endpoint.py
   ```

## Core Functionalities

### 1. Automatic Update (Online Object Handling)
- Real-time tracking of objects through odometry data
- Automatic transformation to global frame
- Continuous state updates in the digital twin

### 2. Offline Object Handling
- Manages static objects (chairs, tables, etc.)
- Supports HoloLens visualization
- Implements LIFO (Last In, First Out) object replacement
- Human verification system for object presence

### 3. Human Correction System
- Requires static object state at correcion time t.
- Integration with XR-Agent for verification
- Process:
  - Initializes `request_STOD_server` (included in the launch file)
  - XR Agent requests the STOD through `request_STOD_client`

### 4. Status Monitoring
- Battery percentage tracking
- Motor temperature monitoring
- Navigation path visualization
- Other Real-time status updates depending on each robot

### 5. Robot Type Integration For Status Monitoring
- Extensible system for different robot types
- Custom subscription handling per robot type
- Configurable monitoring parameters

### 6. Logging System
- Automatic logging on each digital twin update
- Logged information includes:
  - Timestamp
  - Object details (class name, ID, pose)
  - Topic names
  - Status information
- Log location: `{workspace_location}/Digital_twin/src/communication/history/initial_history.JSON`
- Request logs using:
  ```bash
  # Run history server
  ros2 run communication request_history_server (included in launch file)
  # Run history client from the location you want to have your history log in using:

  ```

### 7. Day-to-Day Updates
- System initialization handling
- Omniverse synchronization
- Process:
  1. Run `request_STOD_server`
  2. Run `request_STOD_client` from Omniverse
- Note: STOD is sent to Omniverse once per system run
- To resend STOD, restart the ROS system (Ctrl+C)

## Launch File Components

The main launch file (`communication.launch.py`) initializes:
- `communicator_node`: Core communication with Omniverse and XR-agent
- `request_history_server`: JSON file management for replay
- `request_STOD_server`: Object location management

## Available Services

### 1. STOD (State of Digital Twin) Service
- **Service Name**: `/request_STOD`
- **Purpose**: Retrieves current state of all objects in the digital twin
- **Usage**:
  ```bash
  # Server (included in launch file)
  ros2 run communication request_STOD_server
  
  # Client (from XR-Agent or Omniverse)
  ros2 run communication request_STOD_client
  ```

### 2. History Service
- **Service Name**: `/request_history`
- **Purpose**: Retrieves historical data of the digital twin
- **Usage**:
  ```bash
  # Server
  ros2 run communication request_history_server
  
  # Client (included in launch file)
  ros2 run communication request_history_client
  ```

## Troubleshooting

### Common Issues
1. **Missing Dependencies**
   - Ensure Eigen3 and tf2 are properly installed
   - Verify installations in `/usr/include`

2. **Configuration Issues**
   - Check `robots.yaml` for correct topic names
   - Ensure no topics start with "e.g."

3. **HoloLens Connection**
   - Verify IP address configuration
   - Check network connectivity
   - Ensure ROS-TCP-Endpoint is running

