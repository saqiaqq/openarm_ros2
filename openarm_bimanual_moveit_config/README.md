# MoveIt2 on Bimanual Openarms

Ensure the ROS2 packages and dependencies are installed by following the instructions in `openarm_ros2/README.md`.

## Physical Hardware
1. Run `init_can.sh` from `openarm_bringup/utils`. 
   By default, can0 is the right arm and can1 is the left arm, but this can be adjusted in the ros2_control definition in `openarm_description/urdf/openarm.ros2_control.xacro`.

2. Optionally, start the head-mounted realsense camera. This enables the octomap occupancy grid for planning around obstacles.
   
```sh
ros2 launch openarm_bimanual_bringup depth_camera.launch.py
```

## Launch the demo

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

## Remote RViz (laptop) + robot stack (board)

On the **robot PC** (CAN + `ros2_control` + `move_group`):

```sh
ros2 launch openarm_skills skills.launch.py
# or: ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

On the **laptop** (RViz only, same `ROS_DOMAIN_ID`):

```sh
ros2 launch openarm_bimanual_moveit_config remote_rviz.launch.py
```

RViz MotionPlanning tips:

- Prefer planning group **`right_arm`** or **`left_arm`**, not **`upper_body`**, unless both arms must move together.
- After dragging the goal, click **Plan** then **Execute** immediately (do not Execute an old plan).
- In the Planning tab, set **Start State** to **`<current>`** (not a saved state).
- If Execute fails with `start point deviates from current robot state`, re-Plan; tolerance is configured in `config/trajectory_execution.yaml` (default 0.05 rad).

If logs show `Goal reached, success!` but the real arm does not move, check on the **robot PC**:

```sh
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

Controllers must be `active` and `/joint_states` must change when you move the arm by hand.
