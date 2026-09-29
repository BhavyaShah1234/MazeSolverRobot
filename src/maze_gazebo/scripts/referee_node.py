#!/usr/bin/python3
import subprocess
import rclpy as r
from ament_index_python.packages import get_package_share_directory
from mazelib import Maze
from mazelib.generate.BacktrackingGenerator import BacktrackingGenerator
from maze_interfaces.action import SolveMaze
from rclpy.action import ActionClient
from rclpy.node import Node
import os
import yaml

WORLD_NAME = 'empty'
MAZE_ENTITY_NAME = 'calibration_maze'

class RefereeNode(Node):
    def __init__(self):
        super(RefereeNode, self).__init__(node_name='referee_node')
        share_dir = get_package_share_directory('maze_gazebo')
        with open(os.path.join(share_dir, 'config', 'scene.yaml'), 'r') as f:
            scene = yaml.safe_load(f)
        self.corners = scene['maze']
        self.solve_maze_client = ActionClient(self, SolveMaze, '/solve_maze')
        self.startup_timer = self.create_timer(0.5, self.try_send_solve_goal)
        self.get_logger().info('referee_node started')
        self.reset_maze()

    def bounds(self):
        xs = [c['x'] for name, c in self.corners.items() if isinstance(c, dict)]
        ys = [c['y'] for name, c in self.corners.items() if isinstance(c, dict)]
        return min(xs), max(xs), min(ys), max(ys)

    def generate_layout(self):
        n = self.corners['cells_per_side']
        maze = Maze()
        maze.generator = BacktrackingGenerator(n, n)
        maze.generate()
        grid = maze.grid
        segments = []
        for row in range(n + 1):
            for col in range(n):
                if row == 0 and col == 0:
                    continue
                if row == n and col == n - 1:
                    continue
                if row == 0 or row == n or grid[2 * row, 2 * col + 1]:
                    segments.append([[col, row], [col + 1, row]])
        for row in range(n):
            for col in range(n + 1):
                if col == 0 or col == n or grid[2 * row + 1, 2 * col]:
                    segments.append([[col, row], [col, row + 1]])
        return segments

    def make_maze_sdf(self, segments):
        x_min, x_max, y_min, y_max = self.bounds()
        n = self.corners['cells_per_side']
        thickness = self.corners['wall_thickness_m']
        height = self.corners['wall_height_m']
        cell_size_x = (x_max - x_min) / n
        cell_size_y = (y_max - y_min) / n

        def to_world(gx, gy):
            return (x_min + gx * cell_size_x, y_min + gy * cell_size_y)

        wall_elements = []
        for (gx1, gy1), (gx2, gy2) in segments:
            wx1, wy1 = to_world(gx1, gy1)
            wx2, wy2 = to_world(gx2, gy2)
            cx, cy = (wx1 + wx2) / 2.0, (wy1 + wy2) / 2.0
            length = ((wx2 - wx1) ** 2 + (wy2 - wy1) ** 2) ** 0.5
            if abs(wy2 - wy1) < 1e-9:
                size_x, size_y = length, thickness
            else:
                size_x, size_y = thickness, length
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

        return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{MAZE_ENTITY_NAME}">
    <static>true</static>
    <link name="link">{''.join(wall_elements)}
    </link>
  </model>
</sdf>'''

    def reset_maze(self):
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'name: "{MAZE_ENTITY_NAME}" type: MODEL',
        ], capture_output=True, text=True, timeout=3.0)

        sdf_content = self.make_maze_sdf(self.generate_layout())
        # gz.msgs.EntityFactory has a plain `sdf` string field, so the SDF
        # goes straight into the request as text -- no temp file, and so no
        # race with Gazebo reading a file that a later step then deletes
        # (which is what a filename-based version of this used to need to
        # work around).
        escaped_sdf = sdf_content.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
        subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/create',
            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', f'sdf: "{escaped_sdf}" name: "{MAZE_ENTITY_NAME}"',
        ], capture_output=True, text=True, timeout=3.0)
        self.get_logger().info('maze reset')

    def try_send_solve_goal(self):
        # A blocking wait_for_server() call before this node had ever been
        # spun didn't detect the action server coming up at all, even well
        # after it existed -- server_is_ready() is a plain non-blocking
        # check instead, polled from a normal timer callback once this node
        # is actually spinning, which works reliably.
        if not self.solve_maze_client.server_is_ready():
            return
        self.startup_timer.cancel()
        self.send_solve_goal()

    def send_solve_goal(self):
        goal = SolveMaze.Goal()
        send_future = self.solve_maze_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('solve_maze goal rejected, retrying')
                self.reset_maze()
                self.send_solve_goal()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.solve_result)
        except Exception:
            self.reset_maze()
            self.send_solve_goal()

    def solve_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'solve_maze finished: success={result.success} message="{result.message}"')
        except Exception:
            pass
        finally:
            self.reset_maze()
            self.send_solve_goal()

def main(args=None):
    r.init(args=args)
    node = RefereeNode()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
