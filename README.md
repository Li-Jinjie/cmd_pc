# CMD PC

CMD PC is a repository for trajectory generation.

[TOC]

## Installation

1. Follow the installation in https://github.com/Li-Jinjie/oop_qd_onbd
2. `git clone https://github.com/Li-Jinjie/cmd_pc.git`
5. `catkin build` to build the whole workspace. Done!

## Getting Started

Before each running:  `cd /path_to_workspace` and then `source devel/setup.bash`

- If you want to make one quadrotor fly, just run `roslaunch oop_qd_onbd one_qd_nmpc.launch`
- Then run `rosrun cmd_pc planner_node.py ` to send trajectories!

## License

GPLv3. Please also open-source your project if you use the code from this repository. Let's make the whole community better!
