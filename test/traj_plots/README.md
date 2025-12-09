# Plot moveit_msgs/msg/DisplayTrajectory
This directory contains a python script that can be used to plot a trajectory recorded with ros2 topic echo

## Recording the Trajectory
1) Prepare the Topic recording:
```bash
cd <this_directory>
ros2 topic echo --once /display_planned_path moveit_msgs/msg/DisplayTrajectory > <file_name>.yaml
```
2) Send a planning request of any kind to the MoveIt MoveGroup
3) Call the plotting script, passing the collected data. Activating the venv is only necessary once in the same terminal session.
```bash
source ../.venv/bin/activate
python3 plot_traj.py <file_name>.yaml
```

>[!NOTE]
>
> If the trajectory is too long, `ros2 topic echo` truncates parts of the printed message which will truncate the displayed trajectory as well.




