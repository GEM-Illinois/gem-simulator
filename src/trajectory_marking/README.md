# trajectory_marking

This rospkg presents a demo of visualizing the future predicted position of the GEM vehicle in the Gazebo environment. A model for a marker in Gazebo is used to display the future predicted position.

## Usage

Once inside the `geminit19` directory, run `catkin_make`. If `catkin_make` fails due to a parse error with `src/CMakeLists.txt`, delete `src/CMakeLists.txt` and run `catkin_make` again.

Once `catkin_make` has executed successfully, run `source devel/setup.bash`.

Then, while `roscore` is running, run `roslaunch trajectory_marking trajectory_marking.launch`. This will open a Gazebo environment with a model of the GEM vehicle in a warehouse.

While the Gazebo environment is open, navigate to `geminit19/src/trajectory_marking/src`, and run `python trajectory_marking.py &` to run the trajectory marking logic in the background. Next, run `python demo.py` to view the demo.