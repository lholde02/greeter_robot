#!/bin/bash
echo Createing a welcome message for $1
cat > /home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/welcome$1.sh <<EOF
#!/bin/bash
rosrun sound_play say.py "Welcome to Halligan $1!"
EOF
chmod +x /home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/welcome$1.sh

