# turtlebot_commands
Setting up various publishers and subscribers for TurtlleBot
# Launch Turtlebot with  customized world
In order to launch the turtlebot in world1.world follow the following steps:


Step 1: Download the world1.world file to the location '/opt/ros/indigo/share/turtlebot_gazebo/worlds'

Step 2: Launch the turtlebot using the command


roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/world1.world

ROBOT_INITIAL_POSE="-x -8.0 -y -8.0 -z 0.0" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/world1.world
