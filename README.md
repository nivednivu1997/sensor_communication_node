# sensor_communication_node
This ROS 2 node communicates with a sensor over TCP using a custom protocol. It automatically starts the sensor on launch, decodes status messages, and publishes the data to appropriate topics. Additionally, it provides services to dynamically start and stop the sensor with a custom interval.

Features
Automatic Start on Launch:

The node automatically sends a Start Command to the sensor on launch using the interval specified as a parameter.

Decodes Status Messages:

The node decodes status messages received from the sensor and publishes the data to the following topics:

Supply Voltage: Published as Float32 on the supply_voltage topic.

Environment Temperature: Published as Temperature on the env_temp topic.

IMU Data: Published as Imu on the imu_data topic.

Dynamic Start/Stop Control:

The node provides two services to dynamically control the sensor:

Start Service: Starts the sensor with a custom interval.

Stop Service: Stops the sensor.

Topics and Data Types
The node publishes sensor data to the following topics:

Topic Name	Message Type	Description
supply_voltage	Float32	Voltage supplied to the sensor (in volts).
env_temp	Temperature	Temperature of the sensor (in Celsius).
imu_data	Imu	IMU data containing orientation (as a quaternion) of the sensor.
Services
The node provides the following services for dynamic control:

1. Start Service
Service Name: /start_sensor

Request Type: std_msgs/Int32

The data field specifies the interval (in milliseconds) at which the sensor should send status messages.

Response Type: std_msgs/Bool

The data field indicates success (true) or failure (false).

Example Usage:
bash
Copy
ros2 service call /start_sensor std_msgs/Int32 "{data: 500}"
This command starts the sensor with an interval of 500 milliseconds.

2. Stop Service
Service Name: /stop_sensor

Request Type: std_srvs/Empty

No input is required.

Response Type: std_msgs/Bool

The data field indicates success (true) or failure (false).

Example Usage:
bash
Copy
ros2 service call /stop_sensor std_srvs/Empty
This command stops the sensor.

Parameters
The node uses the following parameters:

Parameter Name	Default Value	Description
sensor_ip	192.168.1.1	IP address of the sensor.
sensor_port	2000	TCP port of the sensor.
interval	1000	Default interval (in milliseconds) for the sensor to send status messages.
How It Works
On Launch:

The node connects to the sensor using the specified IP address and port.

It automatically sends a Start Command to the sensor with the default interval.

Receiving Data:

The node listens for status messages from the sensor.

Each status message is decoded, and the data is published to the appropriate topics.

Dynamic Control:

The user can start or stop the sensor dynamically using the provided services.

Installation and Usage
Clone the Repository:

bash
Copy
git clone <repository_url>
cd <repository_directory>
Build the ROS 2 Workspace:

bash
Copy
colcon build
Run the Node:

bash
Copy
ros2 run <package_name> sensor_node
Start the Sensor with a Custom Interval:

bash
Copy
ros2 service call /start_sensor std_msgs/Int32 "{data: 500}"
Stop the Sensor:

bash
Copy
ros2 service call /stop_sensor std_srvs/Empty
Decoding Status Messages
The node decodes status messages received from the sensor. Each status message contains the following data:

Field	Data Type	Description
SUPPLY_VOLTAGE	uint16	Voltage supplied to the sensor (in millivolts).
ENV_TEMP	int16	Temperature of the sensor (in deci-Celsius).
YAW	int16	Yaw angle of the sensor (in deci-degrees).
PITCH	int16	Pitch angle of the sensor (in deci-degrees).
ROLL	int16	Roll angle of the sensor (in deci-degrees).
The node converts the data into appropriate units and publishes it to the topics.

Dynamic Start/Stop Control
The node provides two services for dynamic control:

Start Service:

Allows the user to start the sensor with a custom interval.

The interval must be a positive integer (in milliseconds).

Stop Service:

Allows the user to stop the sensor.

These services provide a flexible way to control the sensor during runtime.

Dependencies
ROS 2 (tested on Humble Hawksbill).

Python 3.8+.

socket, struct, and threading Python libraries.
