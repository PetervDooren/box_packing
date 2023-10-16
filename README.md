# box_packing

## Installation:
1. Bootstrap the package manager

   ```bash
   source <(wget -O - https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/bootstrap.bash)  # for default ROS1
   ```

2. Change targets branch

   ```bash
   tue-env targets && git checkout peter/flexcraft
   ```
2. Install targets

   ```bash
   tue-get install ros-box_packing
   tue-get install ros-tools
   ```

3. Build sources

   ```bash
   tue-make
   source ~/.bashrc  # Or open a new terminal


## running the demo
after sourcing the workspace.
then type:
`roslaunch tools panda_start`

To activate the controller call the trigger service
`rosservice call /my_controller/trigger "{}"`
