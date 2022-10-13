rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["J1","J3","J5"], points: [{positions: [0, 0, 0], time_from_start: [5,0]}]}' -1
