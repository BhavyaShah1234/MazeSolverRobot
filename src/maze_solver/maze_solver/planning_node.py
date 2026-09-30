# heapq: priority queue backing the A* open set.
import heapq
# OpenCV: resizing/dilating the occupancy grid into a coarser planning grid.
import cv2
# NumPy: array math for the grid and path geometry.
import numpy as np
# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# QoS classes: needed to subscribe/publish with matching transient-local durability.
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# PoseArray for the incoming start/goal poses; PoseStamped for path waypoints.
from geometry_msgs.msg import PoseArray, PoseStamped
# OccupancyGrid: the metric maze grid from perception; Path: the planned route to publish.
from nav_msgs.msg import OccupancyGrid, Path

# How many full-resolution grid cells are merged into one planning-grid cell.
DOWNSAMPLE_CELLS = 6
# How many planning cells to dilate obstacles by, as a safety margin.
INFLATE_CELLS = 1

# The node that turns a metric occupancy grid + start/goal poses into a simplified Cartesian path.
class PlanningNode(Node):
    # Constructor: sets up state, subscriptions, and the path publisher.
    def __init__(self):
        # Register this node with rclpy under the name "planning_node".
        super(PlanningNode, self).__init__(node_name='planning_node')
        # The most recently received occupancy grid; None until perception publishes one.
        self.grid = None
        # QoS matching perception's transient-local, reliable publishers.
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Subscribe to the digitized occupancy grid.
        self.create_subscription(OccupancyGrid, '/maze_occupancy_grid', self.grid_callback, path_qos)
        # Subscribe to the start/goal poses.
        self.create_subscription(PoseArray, '/goals', self.goals_callback, path_qos)
        # Publisher for the planned (and simplified) path.
        self.path_publisher = self.create_publisher(Path, '/path', path_qos)
        # Log that startup completed.
        self.get_logger().info('planning_node started')

    # Store the latest occupancy grid whenever one arrives.
    def grid_callback(self, grid_message):
        # Just remember the message; used lazily by goals_callback.
        self.grid = grid_message

    # Downsample and inflate the full-resolution grid into a coarser, safety-margined planning grid.
    def downsample(self, grid_message, scale, inflate):
        # Binarize the raw grid data (>0 means occupied) and reshape it to (height, width).
        wall = (np.asarray(grid_message.data, dtype=np.int8) > 0).astype(np.uint8).reshape(grid_message.info.height, grid_message.info.width)
        # Compute the downsampled grid's width and height.
        width, height = wall.shape[1] // scale, wall.shape[0] // scale
        # Shrink the binary wall image to planning-grid resolution.
        small = cv2.resize(wall * 255, (width, height), interpolation=cv2.INTER_AREA)
        # Build a circular structuring element sized by the inflate radius.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflate + 1, 2 * inflate + 1))
        # Dilate the walls by that kernel to add a safety margin around them.
        blocked = cv2.dilate((small > 0).astype(np.uint8), kernel) > 0
        # the free floor outside the maze connects the two boundary openings, so a plan
        # over the raw grid can shortcut around the maze instead of through it; fencing
        # everything outside the walls' own bounding box in as blocked forces it to stay inside
        # Find the row/column indices of every currently-blocked cell.
        rows, columns = np.nonzero(blocked)
        # Block everything above the walls' bounding box.
        blocked[:rows.min(), :] = True
        # Block everything below the walls' bounding box.
        blocked[rows.max() + 1:, :] = True
        # Block everything left of the walls' bounding box.
        blocked[:, :columns.min()] = True
        # Block everything right of the walls' bounding box.
        blocked[:, columns.max() + 1:] = True
        # Return the final blocked/free planning grid.
        return blocked

    # Manhattan distance heuristic for A*, admissible on a 4-connected grid.
    def heuristic(self, a, b):
        # Sum of absolute coordinate differences.
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Return the in-bounds 4-connected neighbors of a grid cell.
    def neighbors(self, cell, width, height):
        # Unpack the cell's coordinates.
        x, y = cell
        # The four axis-aligned neighbor candidates.
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        # Filter out any candidate that falls outside the grid.
        return [candidate for candidate in candidates if 0 <= candidate[0] < width and 0 <= candidate[1] < height]

    # Walk the came_from chain from a goal cell back to the start, then reverse it.
    def reconstruct_path(self, came_from, current):
        # Start the path at the goal cell.
        path = [current]
        # Follow predecessors back to the start.
        while current in came_from:
            # Step to this cell's predecessor.
            current = came_from[current]
            # Add it to the path.
            path.append(current)
        # The path was built goal-to-start; reverse it to start-to-goal.
        path.reverse()
        # Return the ordered path of cells.
        return path

    # Standard grid A* search from start to goal over the blocked/free grid.
    def astar(self, blocked, start, goal):
        # Grid dimensions.
        height, width = blocked.shape
        # Priority queue of (estimated_total_cost, cost_so_far, cell), seeded with the start cell.
        open_heap = [(self.heuristic(start, goal), 0, start)]
        # Best known cost to reach each visited cell.
        cost_so_far = {start: 0}
        # Predecessor of each visited cell, for path reconstruction.
        came_from = {}
        # Process cells until the open set is exhausted or the goal is reached.
        while open_heap:
            # Pop the cell with the lowest estimated total cost.
            _, cost, current = heapq.heappop(open_heap)
            # Goal reached -- reconstruct and return the path.
            if current == goal:
                # Build the ordered path of cells from start to goal.
                return self.reconstruct_path(came_from, current)
            # Skip this entry if a cheaper path to `current` was already found since it was queued.
            if cost > cost_so_far[current]:
                # Stale heap entry -- ignore it.
                continue
            # Examine each neighboring cell.
            for neighbor in self.neighbors(current, width, height):
                # Skip neighbors that are inside a wall (or its safety margin).
                if blocked[neighbor[1], neighbor[0]]:
                    # Not traversable.
                    continue
                # Cost to reach this neighbor via `current` (every step costs 1).
                new_cost = cost + 1
                # Only update if this is a cheaper route than any found so far.
                if new_cost < cost_so_far.get(neighbor, float('inf')):
                    # Record the new best cost to this neighbor.
                    cost_so_far[neighbor] = new_cost
                    # Record `current` as this neighbor's predecessor.
                    came_from[neighbor] = current
                    # Push the neighbor onto the open set, prioritized by estimated total cost.
                    heapq.heappush(open_heap, (new_cost + self.heuristic(neighbor, goal), new_cost, neighbor))
        # Open set exhausted without reaching the goal -- no path exists.
        return None

    # Reduce a cell-by-cell path down to just its endpoints and direction-change corners.
    def simplify(self, cells):
        # Keep only the endpoints and the cells where direction changes
        # (corners): every A* cell becoming its own waypoint made the
        # executor interpolate needlessly finely through long straight
        # corridors. The control node interpolates the Cartesian segments
        # between these corners itself.
        # A path of 2 or fewer cells is already as simple as it can be.
        if len(cells) <= 2:
            # Nothing to simplify.
            return cells
        # Always keep the very first cell.
        simplified = [cells[0]]
        # Tracks the direction of the previous step, to detect direction changes.
        previous_direction = None
        # Walk the interior cells (excluding the first and last).
        for i in range(1, len(cells) - 1):
            # Direction of the step arriving at this cell.
            direction = (cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1])
            # A direction change means this cell is a corner worth keeping.
            if direction != previous_direction:
                # Keep this corner cell.
                simplified.append(cells[i])
            # Remember this step's direction for the next iteration.
            previous_direction = direction
        # Always keep the very last cell.
        simplified.append(cells[-1])
        # Return the simplified list of cells.
        return simplified

    # Convert a planning-grid (downsampled) coordinate into a world-frame (x, y) point.
    def grid_xy_to_world(self, grid_message, gx, gy):
        # Full-resolution grid's meters-per-cell.
        resolution = grid_message.info.resolution
        # The grid's origin pose (position + orientation) in the world frame.
        origin = grid_message.info.origin
        # The origin's orientation quaternion.
        q = origin.orientation
        # Grid orientation is a pure yaw rotation (see perception_node), so a
        # full quaternion-to-matrix isn't needed here: cos/sin of the half
        # angle are enough to reconstruct the 2D rotation directly.
        # Recover the yaw angle from the quaternion's z/w components.
        yaw = 2.0 * np.arctan2(q.z, q.w)
        # Cosine and sine of the yaw, for the 2D rotation.
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        # Convert the full-resolution grid-cell coordinate into local (pre-rotation) meters.
        local_x, local_y = gx * resolution, gy * resolution
        # Rotate and translate into world X.
        world_x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        # Rotate and translate into world Y.
        world_y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        # Return the world-frame (x, y) point.
        return world_x, world_y

    # Runs whenever a new start/goal PoseArray arrives: plans and publishes a path.
    def goals_callback(self, goals_message):
        # Guard everything so a transient bad input doesn't crash the node.
        try:
            # Can't plan without a grid yet.
            if self.grid is None:
                # Nothing to do until perception publishes one.
                return
            # Build the coarse, safety-inflated planning grid.
            blocked = self.downsample(self.grid, DOWNSAMPLE_CELLS, INFLATE_CELLS)
            # Unpack the start and goal poses.
            start_pose, goal_pose = goals_message.poses

            # Convert a world-frame pose into a planning-grid (downsampled) cell.
            def to_grid_cell(pose):
                # Invert grid_xy_to_world for the full-resolution grid, then
                # scale down to the downsampled A* grid.
                # The grid's orientation quaternion.
                q = self.grid.info.origin.orientation
                # Recover the yaw angle from the quaternion's z/w components.
                yaw = 2.0 * np.arctan2(q.z, q.w)
                # Cosine and sine of the yaw, for the inverse 2D rotation.
                cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
                # Pose position relative to the grid's origin, in world X.
                dx = pose.position.x - self.grid.info.origin.position.x
                # Pose position relative to the grid's origin, in world Y.
                dy = pose.position.y - self.grid.info.origin.position.y
                # Inverse-rotate into the grid's local X, then convert to full-resolution cell units.
                local_x = (cos_yaw * dx + sin_yaw * dy) / self.grid.info.resolution
                # Inverse-rotate into the grid's local Y, then convert to full-resolution cell units.
                local_y = (-sin_yaw * dx + cos_yaw * dy) / self.grid.info.resolution
                # Scale down from full-resolution cells to planning-grid cells.
                return int(local_x) // DOWNSAMPLE_CELLS, int(local_y) // DOWNSAMPLE_CELLS

            # Convert the start pose to a planning-grid cell.
            start = to_grid_cell(start_pose)
            # Convert the goal pose to a planning-grid cell.
            goal = to_grid_cell(goal_pose)
            # Run A* over the planning grid.
            cells = self.astar(blocked, start, goal)
            # No path exists between start and goal.
            if cells is None:
                # Log a throttled warning so this doesn't spam every frame.
                self.get_logger().warning('no path found through the maze', throttle_duration_sec=5.0)
                # Nothing to publish this cycle.
                return
            # Remember the raw cell count for the log message below.
            cell_count = len(cells)
            # Reduce the path to just its corners.
            cells = self.simplify(cells)
            # Start building the Path message.
            path_message = Path()
            # Reuse the goals message's header (frame/timestamp).
            path_message.header = goals_message.header
            # Convert each simplified planning-grid cell into a world-frame waypoint.
            for x, y in cells:
                # Convert this cell (re-scaled to full-resolution, centered) into world (x, y).
                world_x, world_y = self.grid_xy_to_world(self.grid, (x + 0.5) * DOWNSAMPLE_CELLS, (y + 0.5) * DOWNSAMPLE_CELLS)
                # Build the PoseStamped for this waypoint.
                pose = PoseStamped()
                # Reuse the goals message's header for this waypoint too.
                pose.header = goals_message.header
                # World X coordinate of the waypoint.
                pose.pose.position.x = world_x
                # World Y coordinate of the waypoint.
                pose.pose.position.y = world_y
                # Keep the same plane height as the start/goal poses.
                pose.pose.position.z = goals_message.poses[0].position.z
                # Identity orientation -- only position matters for a path waypoint.
                pose.pose.orientation.w = 1.0
                # Append this waypoint to the path.
                path_message.poses.append(pose)
            # Publish the finished path.
            self.path_publisher.publish(path_message)
            # Log a throttled summary of how much the path was simplified.
            self.get_logger().info(f'published path: {len(cells)} waypoints (simplified from {cell_count} cells)', throttle_duration_sec=2.0)
        # Swallow any unexpected error so one bad input doesn't take the node down.
        except Exception:
            # Silently skip this cycle.
            pass

# Standard ROS2 Python entry point.
def main(args=None):
    # Initialize the rclpy context.
    r.init(args=args)
    # Construct the node.
    node = PlanningNode()
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
