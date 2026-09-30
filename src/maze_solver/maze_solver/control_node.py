"""Hosts the SolveMaze action and orchestrates the solve cycle.

Runs one digitize -> plan -> trace -> home cycle per SolveMaze goal:
triggers perception, waits for a path, and delegates all MoveIt2 motion
mechanics -- tracing the path corner by corner and returning to the ready
pose -- to :class:`~maze_solver.motion_executor.MotionExecutor`, keeping
this module focused purely on the solve-cycle sequence and its action
bookkeeping.
"""

# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# ActionServer to host SolveMaze; ServerGoalHandle for type hints.
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# Future: used to await a /path message arriving on a plain topic subscription.
from rclpy.task import Future
# Path: the planned route received from planning_node.
from nav_msgs.msg import Path
# Empty: the trigger message sent to (re)start perception.
from std_msgs.msg import Empty
# SolveMaze: this package's own action, hosted by this node.
from maze_interfaces.action import SolveMaze
# MotionExecutor: owns all MoveIt2 mechanics this node's solve cycle drives.
from maze_solver.motion_executor import MotionExecutor

# The node that hosts the SolveMaze action and orchestrates perception -> plan -> execute -> home.
class ControlNode(Node):
    """Hosts SolveMaze and orchestrates one full solve cycle per goal.

    Attributes:
        motion: Owns the MoveIt2 clients and motion-execution mechanics.
        path_future: A future that resolves when the next /path message
            arrives, or ``None`` between solve attempts.
        perception_start_publisher: Publisher for the "start a new
            perception cycle" trigger.
        action_server: Hosts the SolveMaze action that the referee node
            calls.
    """

    # Constructor: sets up the motion executor, subscriptions, and the action server.
    def __init__(self) -> None:
        """Initializes the motion executor, subscriptions, and the action server."""
        # Register this node with rclpy under the name "control_node".
        super(ControlNode, self).__init__(node_name='control_node')
        # Owns MoveIt2 clients and the actual segment/home motion mechanics.
        self.motion = MotionExecutor(self)
        # A Future that resolves when the next /path message arrives; None between solve attempts.
        self.path_future: Future | None = None
        # Subscribe to the planned path from planning_node.
        self.create_subscription(Path, '/path', self.path_callback, 10)
        # Publisher for the "start a new perception cycle" trigger.
        self.perception_start_publisher = self.create_publisher(Empty, '/perception/start', 10)
        # Hosts the SolveMaze action that the referee node calls.
        self.action_server = ActionServer(self, SolveMaze, '/solve_maze', execute_callback=self.execute_callback)
        # Log that startup completed.
        self.get_logger().info('control_node started')

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
            # Convert the planned path into oriented Cartesian waypoints.
            waypoints = self.motion.build_waypoints(path_message)

            # Report the executing stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='executing'))
            # Trace each corner-to-corner segment in turn, stopping fully at each corner.
            for index, waypoint in enumerate(waypoints, start=1):
                # Plan and execute this one segment.
                failure_message = await self.motion.trace_segment(waypoint, index, len(waypoints))
                # Stop immediately if this segment failed.
                if failure_message is not None:
                    # Mark the action goal as aborted.
                    goal_handle.abort()
                    # Return a failure result with this segment's failure message.
                    return SolveMaze.Result(success=False, message=failure_message)

            # Report the homing stage to the action client.
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='homing'))
            # Drive the arm back to the ready pose.
            if not await self.motion.go_home():
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
