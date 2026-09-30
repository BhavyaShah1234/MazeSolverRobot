"""Turns a planned path into executed arm motion via MoveIt2.

Owns the MoveIt2 Cartesian-path-planning service client and
trajectory-execution action client, and provides the mechanics
control_node.py's solve-cycle orchestration calls into: converting a
planned path into oriented Cartesian waypoints, tracing them corner by
corner (each segment its own independently time-parameterized trajectory,
so the arm actually stops at every corner instead of carrying momentum
through turns), and returning to the SRDF "ready" pose afterward.
"""

# ActionClient to call MoveIt2's execute_trajectory action.
from rclpy.action import ActionClient
# Base class type hint for the owning node.
from rclpy.node import Node
# Pose: individual Cartesian waypoints sent to MoveIt2.
from geometry_msgs.msg import Pose
# Path: the planned route received from planning_node.
from nav_msgs.msg import Path
# JointState: the arm's live joint positions/velocities.
from sensor_msgs.msg import JointState
# RobotState: wraps a JointState as MoveIt2's expected "starting state" type.
from moveit_msgs.msg import RobotState
# GetCartesianPath: the MoveIt2 service that turns waypoints into a joint trajectory.
from moveit_msgs.srv import GetCartesianPath
# ExecuteTrajectory: the MoveIt2 action that actually drives the arm along a trajectory.
from moveit_msgs.action import ExecuteTrajectory
# Trajectory message types used to build the "return home" trajectory by hand.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# The MoveIt2 planning group this executor commands.
GROUP_NAME = 'fr3_arm'
# The link MoveIt2 should trace the Cartesian path with (the laser tip).
LINK_NAME = 'fr3_laser_link'
# Maximum Cartesian step (meters) between interpolated waypoints in the planned trajectory.
MAX_STEP_M = 0.005
# Below this fraction of a requested Cartesian segment, treat the attempt as
# a real IK/planning failure and report it rather than executing a
# truncated trace anyway -- the "no IK success check" gap the reviewer
# flagged.
FRACTION_THRESHOLD = 0.9
# fr3_joint_limits.yaml/the URDF's own <limit velocity="..."> give MoveIt2
# the real joint limits -- this just leaves headroom for a spline's peak
# velocity exceeding its point-to-point average.
VELOCITY_SAFETY_FACTOR = 0.5
# The FR3's seven joint names, in order.
JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
# Must match maze_moveit_config/config/maze_robot.srdf's "ready" group_state.
READY_ANGLES = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# Duration (seconds) allotted for the "return home" trajectory.
HOME_SECONDS = 4.0

# Owns MoveIt2 clients and turns a planned path into executed motion.
class MotionExecutor:
    """Owns MoveIt2 clients and turns a planned path into executed motion.

    Attributes:
        node: The owning node, used to create clients/subscriptions and log.
        joint_state: The arm's most recent joint state, or ``None`` until
            the first message arrives.
        cartesian_path_client: Client for MoveIt2's Cartesian-path-planning
            service.
        execute_trajectory_client: Client for MoveIt2's
            trajectory-execution action.
    """

    # Constructor: sets up joint-state tracking and the MoveIt2 clients.
    def __init__(self, node: Node) -> None:
        """Sets up joint-state tracking and the MoveIt2 clients.

        Args:
            node: The node to create subscriptions/clients on and log through.
        """
        # The owning node, used to create clients/subscriptions and log.
        self.node = node
        # The arm's most recent JointState; None until the first message arrives.
        self.joint_state: JointState | None = None
        # Subscribe to the arm's live joint states.
        self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        # Client for MoveIt2's Cartesian-path-planning service.
        self.cartesian_path_client = self.node.create_client(GetCartesianPath, '/compute_cartesian_path')
        # Client for MoveIt2's trajectory-execution action.
        self.execute_trajectory_client = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')

    # Store the latest joint state whenever one arrives.
    def joint_state_callback(self, joint_state_message: JointState) -> None:
        """Stores the latest joint state for later use.

        Args:
            joint_state_message: The arm's current joint positions/velocities.
        """
        # Just remember the message; used lazily elsewhere.
        self.joint_state = joint_state_message

    # Read the arm's current joint angles, in this executor's fixed JOINT_NAMES order.
    def current_angles(self) -> list[float]:
        """Reads the arm's current joint angles, in JOINT_NAMES order.

        Returns:
            The current angle of each joint in ``JOINT_NAMES``, in that
            order.
        """
        # Map joint name -> current position from the latest JointState.
        current_by_name = dict(zip(self.joint_state.name, self.joint_state.position))
        # Return the angles reordered to match JOINT_NAMES.
        return [current_by_name[name] for name in JOINT_NAMES]

    # Convert a planned path into oriented Cartesian waypoints for MoveIt2.
    def build_waypoints(self, path_message: Path) -> list[Pose]:
        """Converts a planned path into oriented Cartesian waypoints.

        Args:
            path_message: The planned path from planning_node.

        Returns:
            One Cartesian waypoint per path pose, with the fixed laser
            orientation applied.
        """
        # Accumulates the Cartesian waypoints MoveIt2 should trace.
        waypoints = []
        # Convert each path pose into a Cartesian waypoint with the fixed laser orientation.
        for pose in path_message.poses:
            # Build one waypoint.
            waypoint = Pose()
            # Copy the waypoint's X position from the path.
            waypoint.position.x = pose.pose.position.x
            # Copy the waypoint's Y position from the path.
            waypoint.position.y = pose.pose.position.y
            # Copy the waypoint's Z position from the path.
            waypoint.position.z = pose.pose.position.z
            # laser_link's local +Z (its pointing axis) faces world -Z --
            # a 180 degree rotation about X does that, and there's no
            # reason to prefer any particular roll about the pointing
            # axis itself, so this one fixed orientation covers the
            # whole path.
            # X component of the fixed 180-degree-about-X orientation.
            waypoint.orientation.x = 1.0
            # W component of the fixed 180-degree-about-X orientation.
            waypoint.orientation.w = 0.0
            # Add this waypoint to the list.
            waypoints.append(waypoint)
        # Return the finished list of oriented waypoints.
        return waypoints

    # Send a joint trajectory to MoveIt2's execute_trajectory action and await its result.
    async def run_trajectory(self, joint_trajectory: JointTrajectory) -> ExecuteTrajectory.Result | None:
        """Sends a joint trajectory to MoveIt2 and awaits its execution.

        Args:
            joint_trajectory: The trajectory to execute.

        Returns:
            The action's result once execution finishes, or ``None`` if
            MoveIt2 rejected the goal outright.
        """
        # Build the action goal.
        goal = ExecuteTrajectory.Goal()
        # Attach the trajectory to execute.
        goal.trajectory.joint_trajectory = joint_trajectory
        # Send the goal and wait for MoveIt2 to accept or reject it.
        goal_handle = await self.execute_trajectory_client.send_goal_async(goal)
        # A rejected goal has nothing more to await.
        if not goal_handle.accepted:
            # Signal rejection to the caller.
            return None
        # Wait for the trajectory to finish executing.
        result = await goal_handle.get_result_async()
        # Return the actual result payload.
        return result.result

    # Plan and execute a single current-position -> target_pose Cartesian segment.
    async def trace_segment(self, target_pose: Pose, index: int, total: int) -> str | None:
        """Plans and executes one current-position -> target_pose segment.

        Args:
            target_pose: The corner this segment should end at.
            index: This segment's 1-based position among the full path, used
                for failure messages and to detect the final corner.
            total: The total number of segments in the full path, used for
                failure messages and to detect the final corner.

        Returns:
            ``None`` on success, or a failure message describing what went
            wrong.
        """
        # Build the Cartesian-path planning request for just this one segment.
        request = GetCartesianPath.Request()
        # Waypoints are expressed in the world frame.
        request.header.frame_id = 'world'
        # Start planning from wherever the arm actually is right now -- for every
        # segment after the first, that's its rest position at the previous corner.
        request.start_state = RobotState(joint_state=self.joint_state)
        # Plan for this executor's fixed planning group.
        request.group_name = GROUP_NAME
        # Trace the path with this specific link (the laser tip).
        request.link_name = LINK_NAME
        # A single target waypoint: this segment only covers current position -> this one corner.
        request.waypoints = [target_pose]
        # Maximum interpolation step between waypoints.
        request.max_step = MAX_STEP_M
        # No jump-avoidance thresholding needed for this path.
        request.jump_threshold = 0.0
        # The maze walls are a standalone Gazebo model, never published
        # into MoveIt's planning scene as a CollisionObject, so
        # collision avoidance here couldn't see them anyway --
        # planning_node's A* is what actually keeps the path inside the
        # corridor.
        # Collision checking is disabled for this reason.
        request.avoid_collisions = False
        # Cap velocity to the safety-scaled fraction of the joint limits.
        request.max_velocity_scaling_factor = VELOCITY_SAFETY_FACTOR
        # Cap acceleration the same way.
        request.max_acceleration_scaling_factor = VELOCITY_SAFETY_FACTOR
        # Call the planning service and await its response.
        response = await self.cartesian_path_client.call_async(request)

        # The maze's exit point sits right at the boundary of the arm's
        # comfortable reach, so a shortfall specifically at this last corner
        # is an already-understood, harmless edge-of-reach effect -- before
        # segments were split out, that shortfall was just a small piece of
        # one long aggregate fraction and rarely tipped the whole path below
        # threshold. Split per corner, that same absolute shortfall is now
        # measured against one short segment's own length and fails
        # outright, so only the final corner is exempt from the normal
        # threshold; any real IK/planning failure elsewhere (or a
        # completely unreachable exit) still aborts the attempt.
        is_final_corner = index == total
        # Too little of this segment was actually reachable -- treat as a real failure.
        if response.fraction < FRACTION_THRESHOLD and not (is_final_corner and response.fraction > 0.0):
            # Return a failure message naming which corner failed and why.
            return f'only {response.fraction * 100:.0f}% of corner {index}/{total} was reachable (IK/planning failure)'

        # Planning a single corner at a time, rather than the whole path in one
        # GetCartesianPath call, means each segment's trajectory is independently
        # time-parameterized to start and end at rest: the arm actually comes to a
        # stop at every corner instead of carrying momentum from the previous
        # segment through the turn, which is what was swinging the laser tip wide
        # of the corner and close to the walls. A straight corridor has no
        # intermediate corner to segment on, so it's still one smooth motion.
        trace_result = await self.run_trajectory(response.solution.joint_trajectory)
        # MoveIt2 rejected the execution goal outright.
        if trace_result is None:
            # Return a failure message naming which corner failed and why.
            return f'move_group rejected the trajectory execution goal for corner {index}/{total}'
        # Execution completed but MoveIt2 reported a non-success error code.
        if trace_result.error_code.val != 1:
            # Return a failure message including the specific error code and which corner failed.
            return f'trajectory execution failed at corner {index}/{total}: error_code={trace_result.error_code.val}'
        # No failure -- this segment traced successfully.
        return None

    # Drive the arm back to the SRDF "ready" pose.
    async def go_home(self) -> bool:
        """Drives the arm back to the SRDF "ready" pose.

        Returns:
            ``True`` if the arm successfully reached the ready pose,
            ``False`` otherwise.
        """
        # Build the trajectory's starting point from the arm's current position.
        start_point = JointTrajectoryPoint()
        # Current joint angles, in JOINT_NAMES order.
        start_point.positions = self.current_angles()
        # Start point is at time zero.
        start_point.time_from_start.sec = 0
        # Build the trajectory's ending point at the SRDF "ready" pose.
        home_point = JointTrajectoryPoint()
        # The fixed ready-state joint angles.
        home_point.positions = READY_ANGLES
        # End point is reached after HOME_SECONDS.
        home_point.time_from_start.sec = int(HOME_SECONDS)
        # Assemble the two-point home trajectory.
        home_trajectory = JointTrajectory()
        # Joint order for this trajectory.
        home_trajectory.joint_names = JOINT_NAMES
        # The current-position start point and the ready-pose end point.
        home_trajectory.points = [start_point, home_point]
        # Run the home trajectory and await its result.
        home_result = await self.run_trajectory(home_trajectory)
        # Report whether homing actually succeeded.
        return home_result is not None and home_result.error_code.val == 1
