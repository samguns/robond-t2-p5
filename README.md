git clone https://github.com/ros-perception/slam_gmapping.git

git clone https://github.com/turtlebot/turtlebot.git

git clone https://github.com/turtlebot/turtlebot_interactions.git

git clone https://github.com/turtlebot/turtlebot_simulator.git

rosdep install --from-paths src --ignore-src -r -y
