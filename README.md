On bot:

* run `ssh <zid>@<bot>`
* if not done, add `source /opt/ros/indigo/setup.bash` to .bashrc
* add your ssh key, to avoid setting password next time.

On your machine:

* `source ros_export` to automatically setup ROS_MASTER_URI and ROS_IP (change bot name if necessary). You can name it 'ros_export.<name>' to make it private - it is untracked on git.
* Place beacons directory in `src/ass1/src/beacons.launch`.
* Run `roslaunch ass1 ass1.launch` to begin. 

Follower:
* Run `roslaunch ass1 follower.launch` instead.
