### Fake Laser Scanner Publisher Node

This package generates fake laser scans based on the laser parameters given to it.

This node publishes laser scans to the topic base_scan.

A static transform publisher is set up and the laser link is at the height of about 0.5meters.

The Scanner parameters can be set in the config.yaml file.

The default laser parameters are based on the Sick Tim 571

### Setup & Running Instructions
Clone this package into your workspace.

##### To create a Workspace
>$ mkdir -p my_workspace/src

>$ cd ~/my_workspace/src

>$ catkin_init_workspace

>$  cd ..

>$ catkin_make

>$ source devel/setup.bash

Once your workspace has been setup,

go to the src folder of the workspace and git clone this package.

then,
>$ cd ..

>$ catkin_make

##### To Run the package

>$ roslaunch base_scan base_scan_node.launch

##### To visualize the laser scans

The laser scans could be visualized using Rviz

>$ rviz

If needed, the config file for rviz can be found in the config folder of this package.
