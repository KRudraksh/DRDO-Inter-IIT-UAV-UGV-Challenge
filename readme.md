# Inter IIT 2022: Drdo UAV guided UGV IIT Bombay Meta package

### Instructions

- Clone this package in catkin_ws/src

- Go to catkin_ws root directory and build the package

  ```bash
  cd ~/catkin_ws
  catkin build
  source devel/setup.bash
  #To add this line in .bashrc 
  echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
  ```

  > Important: Make sure to delete folder $HOME/.gazebo/models/gimbal_small_2d if at all it exists.

- To run worlds, first launch arducopter SITL 

  ```bash
  sim_vehicle.py -v ArduCopter --console --map
  ```

  Then launch mavros and gazebo using the command

  ```bash
  roslaunch interiit22 drdo_world1.launch
  ```

  Similarly to launch world 2 just replace world2 in above command 

  ```bash
  roslaunch interiit22 drdo_world2.launch
  ```


