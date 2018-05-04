cat > /home/turtlebot/catkin_ws/src/greeter_robot/data/welcome$1.sh <<EOF
#!/bin/bash
rosrun sound_play say.py "Welcome to Halligan $1!"
EOF
cd /home/turtlebot/catkin_ws/src/greeter_robot/data
chmod +x welcome.sh

