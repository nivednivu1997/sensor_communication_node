# Sensor Communication Node

This ROS 2 node communicates with a sensor over TCP using a custom protocol. It automatically starts the sensor on launch, decodes status messages, and publishes the data to appropriate topics. Additionally, it provides services to dynamically start and stop the sensor with a custom interval.

## Features

### ✅ Automatic Start on Launch
- The node automatically sends a **Start Command** to the sensor on launch using the interval specified as a parameter.

### ✅ Decodes Status Messages
- The node decodes status messages received from the sensor and publishes the data to the following topics:

| Topic Name       | Message Type     | Description |
|-----------------|-----------------|-------------|
| `/supply_voltage` | `std_msgs/Float32` | Voltage supplied to the sensor (in volts). |
| `/env_temp`      | `sensor_msgs/Temperature` | Temperature of the sensor (in Celsius). |
| `/imu_data`      | `sensor_msgs/Imu` | IMU data containing orientation (as a quaternion) of the sensor. |

### ✅ Dynamic Start/Stop Control
- The node provides two services to dynamically control the sensor:

| Service Name       | Request Type      | Response Type     | Description |
|-------------------|------------------|------------------|-------------|
| `/start_sensor`  | `std_msgs/Int32`  | `std_msgs/Bool`  | Starts the sensor with a custom interval. |
| `/stop_sensor`   | `std_srvs/Empty`  | `std_msgs/Bool`  | Stops the sensor. |


## Installation and Usage

- Clone the Repository

git clone https://github.com/nivednivu1997/sensor_communication_node.git
cd sensor_communication_node

- Build the ROS 2 Workspace
  
colcon build &&
source install/setup.bash

- Run the Node
ros2 run course1_pkg sensor_node

Start the Sensor with a Custom Interval
ros2 service call /start_sensor std_msgs/Int32 "{data: 500}"

Stop the Sensor
ros2 service call /stop_sensor std_srvs/Empty




