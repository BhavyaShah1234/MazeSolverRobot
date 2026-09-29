# maze_ws

A ROS 2 Jazzy + MoveIt2 workspace: a Franka FR3 with a laser rangefinder solves a physical
maze it can only see from a fixed overhead camera, built with a real simulation/hardware
boundary -- `maze_solver` has no Gazebo dependency at all, and can in principle be pointed at
a real FR3 + camera by changing launch arguments, not code.

This is a rebuild of an earlier prototype (`ros2_ws`) that worked but coupled the solver to
simulation details (a Gazebo-scoped TF frame name, pixel-space messages, a base pose baked in
via URDF surgery, the environment directly commanding the arm). Everything that project could
do, this one does too, restructured so hardware and simulation are genuinely decoupled.

## Packages

| Package | What it is |
|---|---|
| `franka_description`, `franka_ros2`, `libfranka` | Franka's own submodules (arm URDF, ROS 2 driver, control library) |
| `maze_interfaces` | `SolveMaze` action -- the one interface between the simulation-side referee and the solver |
| `maze_description` | The FR3 + laser-rangefinder xacro. Base pose and the sim/hardware switch are plain xacro arguments, not something patched into the URDF after processing |
| `maze_moveit_config` | SRDF, kinematics/joint-limits/OMPL config, and `move_group` launch, built with `MoveItConfigsBuilder` |
| `maze_bringup` | Robot bring-up: `robot_state_publisher`, `ros2_control` controllers, and the `use_sim` argument that switches between Gazebo and real/mock hardware |
| `maze_gazebo` | Sim-only: world, overhead camera model, maze wall generation, and the referee node that spawns a maze and calls `SolveMaze` in a loop |
| `maze_solver` | Hardware-agnostic: perception (camera -> metric occupancy grid), planning (A* + path simplification), and control (MoveIt2 Cartesian path + execution) |

## Quick start (simulation)

```bash
ros2 launch maze_bringup bringup.launch.py use_sim:=true base_x:=-0.08
ros2 launch maze_moveit_config move_group.launch.py use_sim:=true base_x:=-0.08
ros2 launch maze_solver maze_solver.launch.py
```

The referee node (part of `maze_bringup`'s Gazebo path) spawns a maze, calls `SolveMaze`, and
on any result deletes and respawns a new one -- this loops forever with no manual intervention.
`base_x`/`base_y`/`base_z`/`base_yaw` (passed to both launches identically) set where the arm is
bolted down; `-0.08` for `base_x` balances the arm's reach into the maze against keeping it out
of the overhead camera's view (see `maze_gazebo/config/scene.yaml` for the full reasoning).

## Pointing this at a real robot

```bash
ros2 launch maze_bringup bringup.launch.py use_sim:=false robot_ip:=<your FR3's IP> base_x:=-0.08
ros2 launch maze_moveit_config move_group.launch.py use_sim:=false base_x:=-0.08
ros2 launch maze_solver maze_solver.launch.py use_sim_time:=false
```

No `maze_gazebo` involved, and nothing in `maze_solver` needs editing: `perception_node` reads
the camera's live TF pose and each image's own `header.frame_id` rather than a hardcoded
Gazebo-scoped name, and `control_node` talks to MoveIt2 and `/joint_states` the same way
regardless of what's underneath. You'd need a real camera node publishing
`sensor_msgs/Image`+`CameraInfo` on `/overhead_camera/...` with a genuine, tf-published frame
(any name), mounted so it can see the maze from directly overhead. This hasn't been tested
against real hardware -- there isn't one available in this environment -- but nothing in
`maze_solver`'s code, launch files, or config references `gazebo`, `gz`, or any Gazebo-scoped
name anywhere (verified by grep, not just by design).

## First-time setup (new machine)

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic (simulation only)

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz
```

### 3. Install MoveIt2, ros2_control, and other dependencies

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

`mazelib` (maze generation, used by `maze_gazebo`'s referee node) has no rosdep key:

```bash
/usr/bin/python3 -m pip install --break-system-packages mazelib
```

(`/usr/bin/python3`, not whatever `python3` resolves to on `PATH` -- that's the interpreter
`colcon build`-generated executables actually shebang to, which on a machine with its own
externally-managed virtualenv on `PATH` is a different Python environment.)

### 4. Clone and build

```bash
git clone --recurse-submodules <this repo> maze_ws
cd maze_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`colcon.meta` at the workspace root passes `CMAKE_POLICY_VERSION_MINIMUM=3.5` to `libfranka`,
needed for it to build under a modern CMake.
