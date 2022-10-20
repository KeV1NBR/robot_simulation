
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["J1","J3","J5"], points: [{positions: [3.1416, 1.57097, 1.57097], time_from_start: [10,0]}]}' -1
