# New xela_model that uses Wonik's ROS2 software stack (based on ros2_control)

Adding xela_model to run on the ros2_control-based software stack recently released by Wonik.
The existing xela_model (using allegro_hand_linux_v4's ros2_rosource) will be retained, while the new model will be added.
The new model will leverage the existing xela_model as much as possible to avoid redundant development.


[Task #179](http://invokelee.iptime.org:10003/issues/179): [Model] Add a new allegrohand xela_model for allegro_hand_ros2 controller(Wonik)

[Task #180](http://invokelee.iptime.org:10003/issues/180): [Dev] Create a moveit2 config pkg for the new allegrohand xela model