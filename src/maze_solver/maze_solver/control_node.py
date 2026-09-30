"""Hosts the SolveMaze action and orchestrates the solve cycle.

Runs one digitize -> plan -> trace -> home cycle per SolveMaze goal:
triggers perception, waits for a path, and delegates all path-tracing and
homing to motion_executor (a separate node) via the TraceMaze action,
relaying its feedback into this node's own SolveMaze feedback. This keeps
control_node focused purely on the solve-cycle sequence and its action
bookkeeping, with zero direct coupling to MoveIt2 or motion mechanics.
"""

# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# ActionClient to call motion_executor's TraceMaze action; ActionServer to host SolveMaze; ServerGoalHandle for type hints.
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# Future: used to await a /path message arriving on a plain topic subscription.
from rclpy.task import Future
# Path: the planned route received from planning_node.
from nav_msgs.msg import Path
# Empty: the trigger message sent to (re)start perception.
from std_msgs.msg import Empty
# SolveMaze: this node's own action, hosted here; TraceMaze: motion_executor's action, called from here.
from maze_interfaces.action import SolveMaze, TraceMaze

# The node that hosts the SolveMaze action and orchestrates perception -> plan -> trace -> home.
class ControlNode(Node):
    """Hosts SolveMaze and orchestrates one full solve cycle per goal.

    Attributes:
        path_future: A future that resolves when the next /path message
            arrives, or ``None`` between solve attempts.
        perception_start_publisher: Publisher for the "start a new
            perception cycle" trigger.
        trace_maze_client: Client for motion_executor's TraceMaze action.
        action_server: Hosts the SolveMaze action that the referee node
            calls.
    """

    # Constructor: sets up subscriptions, the TraceMaze client, and the action server.
    def __init__(self) -> None:
        """Initializes subscriptions, the TraceMaze client, and the action server."""
        # Register this node with rclpy under the name "control_node".
        super(ControlNode, self).__init__(node_name='control_node')
        # A Future that resolves when the next /path message arrives; None between solve attempts.
        self.path_future: Future | None = None
        # Subscribe to the planned path from planning_node.
        self.create_subscription(Path, '/path', self.path_callback, 10)
        # Publisher for the "start a new perception cycle" trigger.
        self.perception_start_publisher = self.create_publisher(Empty, '/perception/start', 10)
        # Client for motion_executor's TraceMaze action.
        self.trace_maze_client = ActionClient(self, TraceMaze, '/trace_maze')
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

    # Send the given path to motion_executor's TraceMaze action and await its result.
    async def trace_path(self, path_message: Path, goal_handle: ServerGoalHandle) -> TraceMaze.Result:
        """Sends a path to motion_executor and awaits the trace result.

        Relays TraceMaze's own feedback (``executing``/``homing``) straight
        into this SolveMaze goal's feedback, so a listener sees the same
        stage sequence as before motion execution moved to its own node.

        Args:
            path_message: The path for motion_executor to trace.
            goal_handle: The SolveMaze goal handle to relay feedback onto.

        Returns:
            motion_executor's result for this trace attempt.
        """
        # Build the TraceMaze goal from the planned path.
        trace_goal = TraceMaze.Goal(path=path_message)
        # Send the goal, relaying each TraceMaze feedback message as SolveMaze feedback.
        trace_goal_handle = await self.trace_maze_client.send_goal_async(
            trace_goal,
            feedback_callback=lambda feedback: goal_handle.publish_feedback(
                SolveMaze.Feedback(stage=feedback.feedback.stage)),
        )
        # motion_executor rejected the goal outright.
        if not trace_goal_handle.accepted:
            # Report this as a failure result, same shape as any other trace failure.
            return TraceMaze.Result(success=False, message='motion_executor rejected the trace goal')
        # Wait for the trace attempt to finish.
        result = await trace_goal_handle.get_result_async()
        # Return the actual result payload.
        return result.result

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
            # Hand the path to motion_executor and await its trace result.
            trace_result = await self.trace_path(path_message, goal_handle)
            # motion_executor failed to trace the path (or to fully home, which it reports as a warning, not a failure).
            if not trace_result.success:
                # Mark the action goal as aborted.
                goal_handle.abort()
                # Return a failure result with motion_executor's own message.
                return SolveMaze.Result(success=False, message=trace_result.message)

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
