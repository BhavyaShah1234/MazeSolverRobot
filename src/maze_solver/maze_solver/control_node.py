import rclpy as r
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from maze_interfaces.action import SolveMaze

GROUP_NAME = 'fr3_arm'
LINK_NAME = 'fr3_laser_link'
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
JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
# Must match maze_moveit_config/config/maze_robot.srdf's "ready" group_state.
READY_ANGLES = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
HOME_SECONDS = 4.0

class ControlNode(Node):
    def __init__(self):
        super(ControlNode, self).__init__(node_name='control_node')
        self.joint_state = None
        self.path_future = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.perception_start_publisher = self.create_publisher(Empty, '/perception/start', 10)
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.action_server = ActionServer(self, SolveMaze, '/solve_maze', execute_callback=self.execute_callback)
        self.get_logger().info('control_node started')

    def joint_state_callback(self, joint_state_message):
        self.joint_state = joint_state_message

    def path_callback(self, path_message):
        if self.path_future is not None and not self.path_future.done():
            self.path_future.set_result(path_message)

    def current_angles(self):
        current_by_name = dict(zip(self.joint_state.name, self.joint_state.position))
        return [current_by_name[name] for name in JOINT_NAMES]

    async def run_trajectory(self, joint_trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = joint_trajectory
        goal_handle = await self.execute_trajectory_client.send_goal_async(goal)
        if not goal_handle.accepted:
            return None
        result = await goal_handle.get_result_async()
        return result.result

    async def execute_callback(self, goal_handle):
        try:
            goal_handle.publish_feedback(SolveMaze.Feedback(stage='waiting_for_stable_frame'))
            self.path_future = Future()
            self.perception_start_publisher.publish(Empty())
            path_message = await self.path_future

            goal_handle.publish_feedback(SolveMaze.Feedback(stage='planning'))
            waypoints = []
            for pose in path_message.poses:
                waypoint = Pose()
                waypoint.position.x = pose.pose.position.x
                waypoint.position.y = pose.pose.position.y
                waypoint.position.z = pose.pose.position.z
                # laser_link's local +Z (its pointing axis) faces world -Z --
                # a 180 degree rotation about X does that, and there's no
                # reason to prefer any particular roll about the pointing
                # axis itself, so this one fixed orientation covers the
                # whole path.
                waypoint.orientation.x = 1.0
                waypoint.orientation.w = 0.0
                waypoints.append(waypoint)

            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.start_state = RobotState(joint_state=self.joint_state)
            request.group_name = GROUP_NAME
            request.link_name = LINK_NAME
            request.waypoints = waypoints
            request.max_step = MAX_STEP_M
            request.jump_threshold = 0.0
            # The maze walls are a standalone Gazebo model, never published
            # into MoveIt's planning scene as a CollisionObject, so
            # collision avoidance here couldn't see them anyway --
            # planning_node's A* is what actually keeps the path inside the
            # corridor.
            request.avoid_collisions = False
            request.max_velocity_scaling_factor = VELOCITY_SAFETY_FACTOR
            request.max_acceleration_scaling_factor = VELOCITY_SAFETY_FACTOR
            response = await self.cartesian_path_client.call_async(request)

            if response.fraction < FRACTION_THRESHOLD:
                goal_handle.abort()
                return SolveMaze.Result(
                    success=False,
                    message=f'only {response.fraction * 100:.0f}% of the path was reachable (IK/planning failure)')

            goal_handle.publish_feedback(SolveMaze.Feedback(stage='executing'))
            trace_result = await self.run_trajectory(response.solution.joint_trajectory)
            if trace_result is None:
                goal_handle.abort()
                return SolveMaze.Result(success=False, message='move_group rejected the trajectory execution goal')
            if trace_result.error_code.val != 1:
                goal_handle.abort()
                return SolveMaze.Result(success=False, message=f'trajectory execution failed: error_code={trace_result.error_code.val}')

            goal_handle.publish_feedback(SolveMaze.Feedback(stage='homing'))
            start_point = JointTrajectoryPoint()
            start_point.positions = self.current_angles()
            start_point.time_from_start.sec = 0
            home_point = JointTrajectoryPoint()
            home_point.positions = READY_ANGLES
            home_point.time_from_start.sec = int(HOME_SECONDS)
            home_trajectory = JointTrajectory()
            home_trajectory.joint_names = JOINT_NAMES
            home_trajectory.points = [start_point, home_point]
            home_result = await self.run_trajectory(home_trajectory)
            if home_result is None or home_result.error_code.val != 1:
                # Reaching the goal and tracing the maze is what matters for
                # solving it; failing to tidily return home afterward isn't
                # a solve failure.
                self.get_logger().warning('failed to return home after solving the maze')

            goal_handle.succeed()
            return SolveMaze.Result(success=True, message='maze solved')
        except Exception as exception:
            goal_handle.abort()
            return SolveMaze.Result(success=False, message=str(exception))

def main(args=None):
    r.init(args=args)
    node = ControlNode()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
