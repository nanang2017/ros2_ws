colcon build --packages-select camera
source install/local_setup.bash

terminal 1)
ros2 launch gazebo_ros gazebo.launch.py world:=src/camera/camera.world

terminal 2)
ros2 run camera image_processor


image viewed through cv2 window!