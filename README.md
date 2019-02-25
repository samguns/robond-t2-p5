rosdep install --from-paths catkin_ws/src --ignore-src -r -y

cd catkin_ws

catkin_make

source devel/setup.bash

cd src/ShellScripts/

./home_service.sh
