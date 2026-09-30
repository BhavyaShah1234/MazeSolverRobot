#!/usr/bin/python3
"""Drives the continuous spawn -> solve -> respawn maze loop in simulation.

Loads the maze's corner/sizing configuration, generates a random maze
layout, spawns it into Gazebo as inline SDF, and repeatedly calls the
solver's ``SolveMaze`` action, respawning a fresh maze after every result.
"""

# subprocess: used to invoke the `gz service` CLI for maze removal/creation.
import subprocess
# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# Resolves this package's own installed share directory, to find scene.yaml.
from ament_index_python.packages import get_package_share_directory
# mazelib's maze data structure.
from mazelib import Maze
# mazelib's actual maze-generation algorithm.
from mazelib.generate.BacktrackingGenerator import BacktrackingGenerator
# SolveMaze: the action this node calls on the solver.
from maze_interfaces.action import SolveMaze
# ActionClient: used to send SolveMaze goals.
from rclpy.action import ActionClient
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# Future: the type returned by async action calls this node reacts to.
from rclpy.task import Future
# Path-joining for the scene.yaml file location.
import os
# YAML parsing for scene.yaml.
import yaml

# The Gazebo world this referee operates in.
WORLD_NAME = 'empty'
# The name given to the spawned maze-wall entity in Gazebo.
MAZE_ENTITY_NAME = 'calibration_maze'

# The node that spawns mazes, calls SolveMaze, and loops forever.
class RefereeNode(Node):
    """Spawns mazes, calls SolveMaze, and loops forever.

    Attributes:
        corners: The maze corner/sizing configuration loaded from
            scene.yaml.
        solve_maze_client: Client for the solver's SolveMaze action.
        startup_timer: Polls (non-blocking) until the SolveMaze server is
            ready, then sends the first goal.
    """

    # Constructor: loads scene config, sets up the action client, and spawns the first maze.
    def __init__(self) -> None:
        """Loads scene config, sets up the action client, and spawns the first maze."""
        # Register this node with rclpy under the name "referee_node".
        super(RefereeNode, self).__init__(node_name='referee_node')
        # This package's installed share directory.
        share_dir = get_package_share_directory('maze_gazebo')
        # Open and parse scene.yaml.
        with open(os.path.join(share_dir, 'config', 'scene.yaml'), 'r') as f:
            # Load the whole scene configuration.
            scene = yaml.safe_load(f)
        # Keep just the maze corner/sizing config.
        self.corners: dict = scene['maze']
        # Client for the solver's SolveMaze action.
        self.solve_maze_client = ActionClient(self, SolveMaze, '/solve_maze')
        # Polls (non-blocking) until the SolveMaze server is ready, then sends the first goal.
        self.startup_timer = self.create_timer(0.5, self.try_send_solve_goal)
        # Log that startup completed.
        self.get_logger().info('referee_node started')
        # Spawn the very first maze immediately.
        self.reset_maze()

    # Compute the maze's world-frame bounding box from its configured corner points.
    def bounds(self) -> tuple[float, float, float, float]:
        """Computes the maze's world-frame bounding box.

        Returns:
            A tuple of (x_min, x_max, y_min, y_max), in meters.
        """
        # All configured corners' X coordinates.
        xs = [c['x'] for name, c in self.corners.items() if isinstance(c, dict)]
        # All configured corners' Y coordinates.
        ys = [c['y'] for name, c in self.corners.items() if isinstance(c, dict)]
        # Return (x_min, x_max, y_min, y_max).
        return min(xs), max(xs), min(ys), max(ys)

    # Generate a random maze layout as a list of wall segments in grid coordinates.
    def generate_layout(self) -> list[list[list[int]]]:
        """Generates a random maze layout as grid-unit wall segments.

        Returns:
            A list of [[x1, y1], [x2, y2]] wall segments, in grid units.
        """
        # Number of cells per side of the square maze.
        n = self.corners['cells_per_side']
        # Construct a new mazelib Maze.
        maze = Maze()
        # Use the backtracking algorithm to generate an n x n maze.
        maze.generator = BacktrackingGenerator(n, n)
        # Run generation.
        maze.generate()
        # mazelib's internal grid representation (odd indices are cell walls).
        grid = maze.grid
        # Accumulates [[x1, y1], [x2, y2]] wall segments in grid units.
        segments = []
        # Walk every horizontal wall position (row boundaries).
        for row in range(n + 1):
            # Walk every column at this row boundary.
            for col in range(n):
                # Skip the top-left corner cell's segment -- it's the maze entrance.
                if row == 0 and col == 0:
                    # Leave this opening un-walled.
                    continue
                # Skip the bottom-right corner cell's segment -- it's the maze exit.
                if row == n and col == n - 1:
                    # Leave this opening un-walled.
                    continue
                # Outer border rows are always walled; interior rows follow mazelib's own wall grid.
                if row == 0 or row == n or grid[2 * row, 2 * col + 1]:
                    # Add this horizontal wall segment.
                    segments.append([[col, row], [col + 1, row]])
        # Walk every vertical wall position (column boundaries).
        for row in range(n):
            # Walk every row at this column boundary.
            for col in range(n + 1):
                # Outer border columns are always walled; interior columns follow mazelib's own wall grid.
                if col == 0 or col == n or grid[2 * row + 1, 2 * col]:
                    # Add this vertical wall segment.
                    segments.append([[col, row], [col, row + 1]])
        # Return the full list of wall segments.
        return segments

    # Convert grid-unit wall segments into an inline Gazebo SDF model string.
    def make_maze_sdf(self, segments: list[list[list[int]]]) -> str:
        """Converts grid-unit wall segments into an inline Gazebo SDF model.

        Args:
            segments: The wall segments to render, as returned by
                :meth:`generate_layout`.

        Returns:
            A complete SDF document string defining the maze model.
        """
        # The maze's real-world bounding box.
        x_min, x_max, y_min, y_max = self.bounds()
        # Number of cells per side.
        n = self.corners['cells_per_side']
        # Wall thickness, in meters.
        thickness = self.corners['wall_thickness_m']
        # Wall height, in meters.
        height = self.corners['wall_height_m']
        # World-space size of one grid cell along X.
        cell_size_x = (x_max - x_min) / n
        # World-space size of one grid cell along Y.
        cell_size_y = (y_max - y_min) / n

        # Convert a grid-unit (gx, gy) coordinate into world (x, y).
        def to_world(gx: int, gy: int) -> tuple[float, float]:
            """Converts a grid-unit coordinate into world (x, y).

            Args:
                gx: Grid-unit X coordinate.
                gy: Grid-unit Y coordinate.

            Returns:
                The corresponding world-frame (x, y) point.
            """
            # Scale and offset by the maze's real-world bounding box.
            return (x_min + gx * cell_size_x, y_min + gy * cell_size_y)

        # Accumulates the SDF <visual>/<collision> XML for each wall segment.
        wall_elements = []
        # Build one wall box per segment.
        for (gx1, gy1), (gx2, gy2) in segments:
            # World-space start point of this segment.
            wx1, wy1 = to_world(gx1, gy1)
            # World-space end point of this segment.
            wx2, wy2 = to_world(gx2, gy2)
            # Center point of this wall box.
            cx, cy = (wx1 + wx2) / 2.0, (wy1 + wy2) / 2.0
            # Length of this wall segment.
            length = ((wx2 - wx1) ** 2 + (wy2 - wy1) ** 2) ** 0.5
            # A horizontal segment is long in X, thin in Y.
            if abs(wy2 - wy1) < 1e-9:
                # Box dimensions for a horizontal wall.
                size_x, size_y = length, thickness
            else:
                # Box dimensions for a vertical wall.
                size_x, size_y = thickness, length
            # Append this wall's visual and collision geometry as SDF XML.
            wall_elements.append(f'''
      <visual name="wall_visual_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.7 0.1 1</ambient>
          <diffuse>0.1 0.7 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="wall_collision_{len(wall_elements)}">
        <pose>{cx} {cy} {height / 2.0} 0 0 0</pose>
        <geometry>
          <box>
            <size>{size_x} {size_y} {height}</size>
          </box>
        </geometry>
      </collision>''')

        # Assemble the full SDF model document from all the wall elements.
        return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{MAZE_ENTITY_NAME}">
    <static>true</static>
    <link name="link">{''.join(wall_elements)}
    </link>
  </model>
</sdf>'''

    # Delete any existing maze entity, generate a new layout, and spawn it.
    def reset_maze(self) -> None:
        """Deletes any existing maze entity, generates a new layout, and spawns it."""
        # Ask Gazebo to remove the previous maze entity, if any (a no-op if it doesn't exist).
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'name: "{MAZE_ENTITY_NAME}" type: MODEL',
        ], capture_output=True, text=True, timeout=3.0)

        # Build a fresh SDF model string for a newly generated maze layout.
        sdf_content = self.make_maze_sdf(self.generate_layout())
        # gz.msgs.EntityFactory has a plain `sdf` string field, so the SDF
        # goes straight into the request as text -- no temp file, and so no
        # race with Gazebo reading a file that a later step then deletes
        # (which is what a filename-based version of this used to need to
        # work around).
        # Escape the SDF string so it survives being embedded in the gz service's text-format request.
        escaped_sdf = sdf_content.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
        # Ask Gazebo to create the new maze entity from the inline SDF.
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/create',
            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'sdf: "{escaped_sdf}" name: "{MAZE_ENTITY_NAME}"',
        ], capture_output=True, text=True, timeout=3.0)
        # Log that the maze was (re)spawned.
        self.get_logger().info('maze reset')

    # Timer callback: polls until the SolveMaze server is ready, then sends the first goal.
    def try_send_solve_goal(self) -> None:
        """Polls until the SolveMaze server is ready, then sends the first goal."""
        # A blocking wait_for_server() call before this node had ever been
        # spun didn't detect the action server coming up at all, even well
        # after it existed -- server_is_ready() is a plain non-blocking
        # check instead, polled from a normal timer callback once this node
        # is actually spinning, which works reliably.
        # Keep waiting if the server isn't up yet.
        if not self.solve_maze_client.server_is_ready():
            # Nothing to do this tick.
            return
        # Server is ready -- this polling timer is no longer needed.
        self.startup_timer.cancel()
        # Kick off the actual solve/reset loop.
        self.send_solve_goal()

    # Send one SolveMaze goal to the solver.
    def send_solve_goal(self) -> None:
        """Sends one SolveMaze goal to the solver."""
        # SolveMaze's goal carries no fields.
        goal = SolveMaze.Goal()
        # Send the goal asynchronously.
        send_future = self.solve_maze_client.send_goal_async(goal)
        # React once the server accepts or rejects the goal.
        send_future.add_done_callback(self.goal_response)

    # Callback for when the solver accepts or rejects a SolveMaze goal.
    def goal_response(self, future: Future) -> None:
        """Reacts once the solver accepts or rejects a SolveMaze goal.

        Args:
            future: Resolves to the goal handle once the solver responds.
        """
        # Guard against the future itself raising (e.g. server disappearing).
        try:
            # Resolve the goal handle.
            goal_handle = future.result()
            # The solver rejected this goal outright.
            if not goal_handle.accepted:
                # Log the rejection.
                self.get_logger().warning('solve_maze goal rejected, retrying')
                # Respawn the maze and try again.
                self.reset_maze()
                # Send a fresh goal.
                self.send_solve_goal()
                # Nothing more to do for this rejected goal.
                return
            # Goal accepted -- wait for its eventual result.
            result_future = goal_handle.get_result_async()
            # React once the solve attempt finishes.
            result_future.add_done_callback(self.solve_result)
        # Any unexpected failure while handling the response.
        except Exception:
            # Respawn the maze and try again, same as an explicit rejection.
            self.reset_maze()
            # Send a fresh goal.
            self.send_solve_goal()

    # Callback for when a SolveMaze goal finishes (success or failure).
    def solve_result(self, future: Future) -> None:
        """Reacts once a SolveMaze goal finishes, win or lose.

        Args:
            future: Resolves to the action result wrapper once the solve
                attempt finishes.
        """
        # Guard against the future itself raising.
        try:
            # Unwrap the actual SolveMaze.Result payload.
            result = future.result().result
            # Log whether the maze was solved and why/why not.
            self.get_logger().info(f'solve_maze finished: success={result.success} message="{result.message}"')
        # Any unexpected failure while reading the result.
        except Exception:
            # Nothing useful to log -- fall through to the reset/retry below regardless.
            pass
        # Always respawn a new maze and immediately try to solve it again, win or lose.
        finally:
            # Respawn the maze for the next cycle.
            self.reset_maze()
            # Send the next goal.
            self.send_solve_goal()

# Standard ROS2 Python entry point.
def main(args: list[str] | None = None) -> None:
    """Initializes rclpy, spins RefereeNode, and shuts down on exit.

    Args:
        args: Command-line arguments forwarded to rclpy, or ``None`` to use
            ``sys.argv``.
    """
    # Initialize the rclpy context.
    r.init(args=args)
    # Construct the node.
    node = RefereeNode()
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
