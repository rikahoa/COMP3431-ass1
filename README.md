On bot:

* run `ssh <zid>@<bot>`
* if not done, add `source /opt/ros/indigo/setup.bash` to .bashrc
* add your ssh key, to avoid setting password next time.
* `roslaunch comp3431 turtlebot.launch`
* to contorl, run `roslaunch turtlebot_teleop keyboard_teleop.launch`

On your machine:

* `source ros_export` to automatically setup ROS_MASTER_URI and ROS_IP (change bot name if necessary)
* play
