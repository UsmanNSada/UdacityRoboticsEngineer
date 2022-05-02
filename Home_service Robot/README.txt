The Packages used in this project that covers Localization, Mapping and Navigation
SLAM Gmapping
amcl demo, keyboard teleop(Turtlebot teleop)
pickobjects
add_markers
turtlebot gazebo package in the turtlebot_simulator folder : amcl_demo.launch gmapping_demo.launch

The Package that covers Localization:
SLAM Gmapping uses laser scan, robot base odometry values and landmarks to perform Simultaneous -
-localization and mapping. For this project: the map is already given. And the SLAM has been performed
already. To perform SLAM, the robot would need to navigate through the map taking distance measurements and 
observations using its Lidar laser scans along with its odometry values to determine how far it has moved given 
its previous and initial starting positions.
The robot will navigate through all of the map to rectify its constructed map and position assumption-
upon seeing and observing a previously
seen landmark.

As no mapping is required for this project, the key challenge for the robot is to identify its position within 
the given map already constructed in the Map-my-world project.
AMCL package package within the turtlebot Gazebo full pack establishes this. It assigns particles with indications-
about the robot direction randomly across the given map. As the robot moves and records lidar-laser scan observations,
the AMCL (adaptive monte carlo localization) node assigns weights to the particles based on the scan matching of the-
particles and the observed scans of the robot itself. Particles with Scans that resemble and are more closely matched with 
the robot are assigned higher weights. Resampling is done and particles with lowest weight values are discarded. 
This cycle is continued until only particles that are highly matched with the robots observations are left. These particles -
are highly indicitive of the robots location within the given map.

For the navigation, There are several packages that accomplish it. Such packages include the pickobjects package and-
the Turtlebot Teleop package. 
Firstly, the turtlebot teleop package is used to manually control the robot to navigate to desired locations given a map
and known robot poses. The teleop can be controlled by keyboard to navigate the robot in 6D directions and also, it provides-
rotation capability. While moving the robot, the AMCL node consistently performs localization and scan matching to accurately-
determine the robots movement and poses as it travels within the given map.

Secondly, the pickobjects package navigates the robot without the need of a manual control of the robots movements.
The pickobjects node uses a client-server model using a simpleaction client to send target location requests to move_base server.
target location coordinate is given to the move_base server and in turn, it uses a Djikstra's path-planning algorithm to determine
the best path for the robot to follow given a map and the known poses of the robot. The move_base server only accepts goal target-
coordinate values within the given map. This enables to robot to use its current coordinate pose position and determine a path-
to the target coordinate requested from the move_base server.

Add_Markers package. This package simulates virtual objects in given coordinates. The location is given in the node and in turn, the
ros node publishes the specified type of virtual object at the given coordinate location.
The add_markers node used for the home_service shell file uses object oriented approach to automatically publish the virtual object-
at the initial location known as the pickup location of the virtual object. A subscriber is added to subscribe to odometry values.
These values are consistently processed in a subscriber callBack function. The callback function constantly checks weather the robot-
odometry values indicate that the robot is at the initial object pickup location and the final object drop-off location. If the robot-
is at the pickup zone within distance of 0.1 in Ros odom units then the virtual object is picked up and thus dissapears. This then,
adds a new target location for the virtual object, the final target location that the robot is expected to reach in the pickobjects-
node. However, the virtual object dissapears as the robot has initially reached the first location. Immediately the robot reaches the-
final target location, the virtual object appears at the final target location after about 5 seconds and never dissapears.