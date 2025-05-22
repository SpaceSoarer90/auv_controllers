ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro xacro/main.urdf.xacro)"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$HOME/workspaces/sisid_ws/src/sisid_description/gz/worlds/sisid_world.sdf -v4"
ros2 run ros_gz_sim create -topic "robot_description" -name "sisid_debug" -z "-1.0"
ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
ros2 run ros_gz_bridge parameter_bridge /model/sisid_debug/joint/{left,right,vert}_thruster_prop_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double

# i'm not sure if this one is necessary
ros2 run tf2_ros static_transform_publisher --x -0.29396250 --y 0.12237500 --frame-id base_link --child-frame-id left_thruster_prop_link &
ros2 run tf2_ros static_transform_publisher --x -0.29396250 --y -0.12237500 --frame-id base_link --child-frame-id right_thruster_prop_link &
ros2 run tf2_ros static_transform_publisher --x -0.00331250 --z -0.00436704 --frame-id base_link --child-frame-id vert_thruster_prop_link &

ros2 run controller_manager left_thruster_prop_controller
