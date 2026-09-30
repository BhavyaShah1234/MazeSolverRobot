"""Hosts the SolveMaze action and drives the arm through the digitized maze.

Orchestrates one full perception -> plan -> trace -> home cycle per
``SolveMaze`` goal: triggers perception, waits for a path, traces it corner
by corner via MoveIt2's Cartesian-path service and trajectory-execution
action, and returns the arm to its SRDF "ready" pose afterward.
"""

# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# ActionClient to call MoveIt2's execute_trajectory action; ActionServer to host SolveMaze; ServerGoalHandle for type hints.
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# Future: used to await a /path message arriving on a plain topic subscription.
from rclpy.task import Future
# Pose: individual Cartesian waypoints sent to MoveIt2.
from geometry_msgs.msg import Pose
# Path: the planned route received from planning_node.
from nav_msgs.msg import Path
# JointState: the arm's live joint positions/velocities.
from sensor_msgs.msg import JointState
# Empty: the trigger message sent to (re)start perception.
from std_msgs.msg import Empty
# RobotState: wraps a JointState as MoveIt2's expected "starting state" type.
from moveit_msgs.msg import RobotState
# GetCartesianPath: the MoveIt2 service that turns waypoints into a joint trajectory.
from moveit_msgs.srv import GetCartesianPath
# ExecuteTrajectory: the MoveIt2 action that actually drives the arm along a trajectory.
from moveit_msgs.action import ExecuteTrajectory
# Trajectory message types used to build the "return home" trajectory by hand.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# SolveMaze: this package's own action, hosted by this node.
from maze_interfaces.action import SolveMaze

# The MoveIt2 planning group this node commands.
GROUP_NAME = 'fr3_arm'
# The link MoveIt2 should trace the Cartesian path with (the laser tip).
LINK_NAME = 'fr3_laser_link'
# Maximum Cartesian step (meters) between interpolated waypoints in the planned trajectory.
MAX_STEP_M = 0.005
# Below this fraction of the requested Cartesian path, treat the attempt as
# a real IK/planning failure and report it rather than executing a
# truncated trace anyway -- the "no IK success check" gap the reviewer
# flagged. A small amount of shortfall near the goal corner is an already-
# understood, harmless edge-of-reach effect (see project memory), hence not
# 1.0.
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

# The node that hosts the SolveMaze action and orchestrates perception -> plan -> execute -> home.
class ControlNode(Node):
    """Hosts SolveMaze and orchestrates one full solve cycle per goal.

    Attributes:
        joint_state: The arm's most recent joint state, or ``None`` until
            the first message arrives.
        path_future: A future that resolves when the next /path message
            arrives, or ``None`` between solve attempts.
        perception_start_publisher: Publisher for the "start a new
            perception cycle" trigger.
        cartesian_path_client: Client for MoveIt2's Cartesian-path-planning
            service.
        execute_trajectory_client: Client for MoveIt2's
            trajectory-execution action.
        action_server: Hosts the SolveMaze action that the referee node
            calls.
    """

    # Constructor: sets up state, subscriptions, clients, and the action server.
    def __init__(self) -> None:
        """Initializes state, subscriptions, clients, and the action server."""
        # Register this node with rclpy under the name "control_node".
        super(ControlNode, self).__init__(node_name='control_node')
        # The arm's most recent JointState; None until the first message arrives.
        self.joint_state: JointState | None = None
        # A Future that resolves when the next /path message arrives; None between solve attempts.
        self.path_future: Future | None = None
        # Subscribe to the arm's live joint states.
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        # Subscribe to the planned path from planning_node.
        self.create_subscription(Path, '/path', self.path_callback, 10)
        # Publisher for the "start a new perception cycle" trigger.
        self.perception_start_publisher = self.create_publisher(Empty, '/perception/start', 10)
        # Client for MoveIt2's Cartesian-path-planning service.
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        # Client for MoveIt2's trajectory-execution action.
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # Hosts the SolveMaze action that the referee node calls.
        self.action_server = ActionServer(self, SolveMaze, '/solve_maze', execute_callback=self.execute_callback)
        # Log that startup completed.
        self.get_logger().info('control_node started')

    # Store the latest joint state whenever one arrives.
    def joint_state_callback(self, joint_state_message: JointState) -> None:
        """Stores the latest joint state for later use.

        Args:
            joint_state_message: The arm's current joint positions/velocities.
        """
        # Just remember the message; used lazily elsewhere.
        self.joint_state = joint_state_message

    # Resolve the pending path_future when a new path arrives, if one is being awaited.
    def path_callback(self, path_message: Path) -> None:
        """Resolves the pending path future when a new path arrives.

        Args:
            path_message: The planned path just published by planning_node.
        """
        # Only resolve if a solve attempt is actually waiting on a path right now.
        if self.path_future is not None and not self.path_future.done():
            # Deliver this path to whoever is awaiting path_future.
            self.path_future.set_result(path_message)

    # Read the arm's current joint angles, in this node's fixed JOINT_NAMES order.
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
    # Returns None on success, or a failed SolveMaze.Result describing what went wrong.
    async def trace_segment(self, target_pose: Pose, index: int, total: int) -> SolveMaze.Result | None:
        """Plans and executes one current-position -> target_pose segment.

        Args:
            target_pose: The corner this segment should end at.
            index: This segment's 1-based position among the full path, used
                only for failure messages.
            total: The total number of segments in the full path, used only
                for failure messages.

        Returns:
            ``None`` on success, or a failed :class:`SolveMaze.Result`
            describing what went wrong.
        """
        # Build the Cartesian-path planning request for just this one segment.
        request = GetCartesianPath.Request()
        # Waypoints are expressed in the world frame.
        request.header.frame_id = 'world'
        # Start planning from wherever the arm actually is right now -- for every
        # segment after the first, that's its rest position at the previous corner.
        request.start_state = RobotState(joint_state=self.joint_state)
        # Plan for this node's fixed planning group.
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

        # Too little of this segment was actually reachable -- treat as a real failure.
        if response.fraction < FRACTION_THRESHOLD:
            # Return a failure result naming which corner failed and why.
            return SolveMaze.Result(
                success=False,
                message=f'only {response.fraction * 100:.0f}% of corner {index}/{total} was reachable (IK/planning failure)')

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
            # Return a failure result naming which corner failed and why.
            return SolveMaze.Result(success=False, message=f'move_group rejected the trajectory execution goal for corner {index}/{total}')
        # Execution completed but MoveIt2 reported a non-success error code.
        if trace_result.error_code.val != 1:
            # Return a failure result including the specific error code and which corner failed.
            return SolveMaze.Result(success=False, message=f'trajectory execution failed at corner {index}/{total}: error_code={trace_result.error_code.val}')
        # No failure -- this segment traced successfully.
        return None

    # The SolveMaze action's execute callback: runs one full digitize -> plan -> trace -> home cycle.
    async def execute_callback(self, goal_handle: ServerGoalHandle) -> SolveMaze.Result:
        """Runs one full digitize -> plan -> trace -> home solve cycle.

        Args:
            goal_handle: The SolveMaze action goal handle to report feedback
                and completion on.

        Returns:
            The result of this solve attempt.
        """
        # Guard the whole cycle so any failure becomes a clean action result instead of a crash.
        try:
            # Report the first stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='waiting_for_stable_frame'))
            # Create a fresh Future that path_callback will resolve once a path arrives.
            self.path_future = Future()
            # Tell perception_node to (re)start looking for a stable frame.
            self.perception_start_publisher.publish(Empty())
            # Wait until planning_node publishes a path for this cycle.
            path_message = await self.path_future

            # Report the planning stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='planning'))
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

            # Report the executing stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='executing'))
            # Trace each corner-to-corner segment in its own Cartesian-path call, so
            # the arm actually stops at every corner instead of one continuous
            # multi-corner trajectory that carries momentum straight through turns.
            for index, waypoint in enumerate(waypoints, start=1):
                # Plan and execute this one segment.
                failure = await self.trace_segment(waypoint, index, len(waypoints))
                # Stop immediately if this segment failed.
                if failure is not None:
                    # Mark the action goal as aborted.
                    goal_handle.abort()
                    # Return the failure result from this segment.
                    return failure

            # Report the homing stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='homing'))
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
            # Homing failed outright or completed with a non-success error code.
            if home_result is None or home_result.error_code.val != 1:
                # Reaching the goal and tracing the maze is what matters for
                # solving it; failing to tidily return home afterward isn't
                # a solve failure.
                # Log a warning, but don't fail the overall solve attempt.
                self.get_logger().warning('failed to return home after solving the maze')

            # Mark the action goal as succeeded.
            goal_handle.succeed()
            # Return the successful result.
            return SolveMaze.Result(success=True, message='maze solved')
        # Catch anything unexpected so it becomes a clean failure result instead of a crash.
        except Exception as exception:
            # Mark the action goal as aborted.
            goal_handle.abort()
            # Return a failure result with the exception's message.
            return SolveMaze.Result(success=False, message=str(exception))

# Standard ROS2 Python entry point.
def main(args: list[str] | None = None) -> None:
    """Initializes rclpy, spins ControlNode, and shuts down on exit.

    Args:
        args: Command-line arguments forwarded to rclpy, or ``None`` to use
            ``sys.argv``.
    """
    # Initialize the rclpy context.
    r.init(args=args)
    # Construct the node.
    node = ControlNode()
    # Block, processing callbacks, until shutdown.
    r.spin(node)
    # Clean up the node on exit.
    node.destroy_node()
    # Tear down the rclpy context.
    r.shutdown()

# Only run main() when this file is executed directly (not on import).
if __name__ == '__main__':
    # Invoke the entry point.
    main()
