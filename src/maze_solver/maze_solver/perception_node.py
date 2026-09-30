"""Digitizes the overhead camera feed into a metric occupancy grid.

Waits for a stable, unobstructed camera frame, thresholds it to find the
maze's walls and entrance/exit openings, and publishes the result as a
metric :class:`nav_msgs.msg.OccupancyGrid` plus a
:class:`geometry_msgs.msg.PoseArray` of start/goal poses, using the
camera's live TF pose rather than any simulation-specific assumption.
"""

# OpenCV: HSV color masking and connected-component analysis on the camera image.
import cv2
# NumPy: vector/matrix math for camera-pose and pixel<->world conversions.
import numpy as np
# rclpy aliased to r, matching this project's ROS2 node convention.
import rclpy as r
# tf2_ros: look up the camera's live pose in the TF tree instead of hardcoding it.
import tf2_ros
# groupby: used to find runs of "open" pixels along the maze's border columns (the entrance/exit gaps).
from itertools import groupby
# CvBridge: convert ROS sensor_msgs/Image messages to OpenCV arrays.
from cv_bridge import CvBridge
# Base class for all ROS2 nodes in rclpy.
from rclpy.node import Node
# Time: used to request "latest available" TF transforms.
from rclpy.time import Time
# QoS classes: needed to publish the grid/goals as durable (transient-local) topics.
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# Camera image and intrinsics message types.
from sensor_msgs.msg import Image, CameraInfo
# Pose/PoseArray for the start+goal points; Quaternion for the grid's orientation.
from geometry_msgs.msg import Pose, PoseArray, Quaternion
# OccupancyGrid: the metric, ROS-standard message this node publishes the digitized maze as.
from nav_msgs.msg import OccupancyGrid
# Empty: the trigger message that (re)starts perception for a new maze; Header: a message's own timestamp/frame.
from std_msgs.msg import Empty, Header

# Number of consecutive identical camera frames required before trusting a digitization.
STABLE_FRAMES = 5
# Z of the plane the color mask is actually detecting (the top of the green
# walls, roughly the floor): used only to scale pixel distances to real-world
# distances via similar triangles (see pixel_to_world_xy below), not tied to
# any particular maze's configured wall height.
MAZE_PLANE_Z_M = 0.015

# The reference camera mount (roll=pi, pitch=pi/2, yaw=0 -- straight down)
# maps its LOCAL -Y axis to "image column (u) increasing" and its LOCAL -Z
# axis to "image row (v) increasing", in world XY, at yaw=0. Rotating these
# same local axes by the camera's ACTUAL orientation (from TF, whatever its
# current yaw is) gives the real column/row directions in world XY for any
# yaw -- this is what makes the occupancy grid's orientation track a
# reconfigured camera yaw instead of silently mirroring it. Derived once
# analytically (see project memory), not re-derived at runtime.
# The camera's local axis that corresponds to "image column increasing".
LOCAL_COLUMN_AXIS = np.array([0.0, -1.0, 0.0])
# The camera's local axis that corresponds to "image row increasing".
LOCAL_ROW_AXIS = np.array([0.0, 0.0, -1.0])

# Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix.
def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Converts a quaternion into a 3x3 rotation matrix.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        The equivalent 3x3 rotation matrix.
    """
    # Standard quaternion-to-rotation-matrix formula, returned as a 3x3 NumPy array.
    return np.array([
        # First row of the rotation matrix.
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        # Second row of the rotation matrix.
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        # Third row of the rotation matrix.
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

# The node that turns overhead-camera frames into a metric occupancy grid and start/goal poses.
class PerceptionNode(Node):
    """Digitizes overhead-camera frames into a metric grid and goal poses.

    Attributes:
        bridge: Converts ROS Image messages to OpenCV arrays.
        camera_info: The latest camera intrinsics, or ``None`` until the
            first message arrives.
        triggered: Whether this perception run has already locked onto a
            stable, trusted frame.
        previous_frame: The most recent mask, kept to compare against the
            next frame for stability.
        stable_count: How many consecutive frames have matched
            ``previous_frame`` so far.
        frozen_grid: The grid message computed once ``triggered`` is
            ``True``, republished every frame after that.
        frozen_goals: The goals message computed once ``triggered`` is
            ``True``, republished every frame after that.
        tf_buffer: Buffers incoming TF transforms for synchronous lookup.
        tf_listener: Subscribes to /tf and /tf_static, filling ``tf_buffer``.
        grid_publisher: Publisher for the digitized occupancy grid.
        goals_publisher: Publisher for the start/goal poses found in the
            maze.
    """

    # Constructor: sets up state, subscriptions, publishers, and TF listening.
    def __init__(self) -> None:
        """Initializes state, subscriptions, publishers, and TF listening."""
        # Register this node with rclpy under the name "perception_node".
        super(PerceptionNode, self).__init__(node_name='perception_node')
        # CvBridge instance reused across every incoming frame.
        self.bridge = CvBridge()
        # Latest CameraInfo (intrinsics); None until the first message arrives.
        self.camera_info: CameraInfo | None = None
        # Whether this perception run has already locked onto a stable, trusted frame.
        self.triggered: bool = False
        # The most recent mask, kept to compare against the next frame for stability.
        self.previous_frame: list | None = None
        # How many consecutive frames have matched previous_frame so far.
        self.stable_count: int = 0
        # The grid message computed once triggered==True, republished every frame after that.
        self.frozen_grid: OccupancyGrid | None = None
        # The goals message computed once triggered==True, republished every frame after that.
        self.frozen_goals: PoseArray | None = None
        # Buffers incoming TF transforms so lookup_transform can be called synchronously.
        self.tf_buffer = tf2_ros.Buffer()
        # Subscribes to /tf and /tf_static on this node's behalf, filling tf_buffer.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscribe to the raw overhead camera image.
        self.create_subscription(Image, '/overhead_camera/image', self.image_callback, 10)
        # Subscribe to the camera's intrinsics (needed for the pixel<->world math).
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.camera_info_callback, 10)
        # Subscribe to the "start a new perception cycle" trigger from control_node.
        self.create_subscription(Empty, '/perception/start', self.start_callback, 10)
        # QoS for the grid/goals topics: reliable and transient-local so late subscribers still get the last value.
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Publisher for the digitized occupancy grid.
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/maze_occupancy_grid', grid_qos)
        # Publisher for the start/goal poses found in the maze.
        self.goals_publisher = self.create_publisher(PoseArray, '/goals', grid_qos)
        # Log that startup completed.
        self.get_logger().info('perception_node started')

    # Store the latest camera intrinsics whenever a new CameraInfo message arrives.
    def camera_info_callback(self, camera_info_message: CameraInfo) -> None:
        """Stores the latest camera intrinsics for later use.

        Args:
            camera_info_message: The camera's intrinsics, published
                alongside its image stream.
        """
        # Just remember the message; used lazily by build_grid_and_goals.
        self.camera_info = camera_info_message

    # Reset all per-cycle state so perception starts looking for a fresh stable frame.
    def start_callback(self, empty_message: Empty) -> None:
        """Resets all per-cycle state to begin a new perception cycle.

        Args:
            empty_message: The trigger message; carries no data.
        """
        # Clear the "already locked on" flag.
        self.triggered = False
        # Forget the previous candidate frame data.
        self.previous_frame = None
        # Reset the stability counter.
        self.stable_count = 0
        # Drop any previously frozen grid.
        self.frozen_grid = None
        # Drop any previously frozen goals.
        self.frozen_goals = None
        # Log that a new perception cycle has begun.
        self.get_logger().info('perception restarted, looking for a new stable frame')

    # Decode a ROS Image message into a BGR OpenCV array, regardless of its original encoding.
    def decode_image(self, image_message: Image) -> np.ndarray:
        """Decodes a ROS Image message into a BGR OpenCV array.

        Args:
            image_message: The camera image to decode.

        Returns:
            The image as a BGR OpenCV array, regardless of its original
            encoding.
        """
        # Let CvBridge do the raw decode first, in whatever encoding the message declares.
        image = self.bridge.imgmsg_to_cv2(image_message)
        # RGB images need a channel swap to become BGR (OpenCV's native order).
        if image_message.encoding == 'rgb8':
            # Convert RGB to BGR.
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # RGBA images also need a channel swap (and drop of alpha) to become BGR.
        if image_message.encoding == 'rgba8':
            # Convert RGBA to BGR.
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        # BGRA images just need the alpha channel dropped.
        if image_message.encoding == 'bgra8':
            # Convert BGRA to BGR.
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        # Already BGR (or some other encoding CvBridge already normalized) -- use as-is.
        return image

    # Threshold the image to a binary mask of "green wall" pixels.
    def digitize_mask(self, image: np.ndarray) -> np.ndarray:
        """Thresholds an image to a binary mask of "green wall" pixels.

        Args:
            image: A BGR image, as returned by :meth:`decode_image`.

        Returns:
            A single-channel mask where wall pixels are 255 and everything
            else is 0.
        """
        # Convert to HSV, since color thresholding is far more robust in HSV than BGR.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Keep only pixels within the green wall's HSV range.
        return cv2.inRange(hsv_image, (35, 40, 40), (85, 255, 255))

    # Find the pixel coordinates of the maze's entrance and exit openings along its outer border.
    def compute_goals_pixels(self, mask: np.ndarray) -> list[tuple[int, int]] | None:
        """Finds the maze's entrance/exit opening pixels along its border.

        Args:
            mask: The binary wall mask, as returned by :meth:`digitize_mask`.

        Returns:
            A two-element list of (column, row) pixel coordinates, entrance
            first then exit, or ``None`` if either opening isn't
            unambiguously found.
        """
        # Binarize the mask to 0/1 for connected-component analysis.
        wall = (mask > 0).astype(np.uint8)
        # Label connected wall blobs so small noise specks can be filtered out.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(wall)
        # Keep only components that are at least 5% of the largest component's area (drops noise).
        keep = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 0.05 * stats[1:, cv2.CC_STAT_AREA].sum()]
        # Rebuild the wall mask using only the kept (real) components.
        wall = np.isin(labels, keep)
        # Bounding box of the whole maze structure in pixel space.
        x, y, w, h = cv2.boundingRect(wall.astype(np.uint8))
        # Width (in pixels) of the border strip searched for an opening on each side.
        band = max(4, w // 50)
        # Accumulates the (column, row) pixel of each opening found.
        openings = []
        # start is the opening in the left border (world y_min, the maze's
        # entrance), goal the one in the right border
        # Loop over the left-border and right-border strips in turn.
        for u, columns in ((x + band // 2, slice(x, x + band)), (x + w - 1 - band // 2, slice(x + w - band, x + w))):
            # Collects the row(s) where a gap run was found in this strip; row tracks scan position.
            rows, row = [], 0
            # Walk runs of solid/non-solid pixels down this vertical strip.
            for solid, group in groupby(wall[y:y + h, columns].any(axis=1)):
                # Length of this run of same-value pixels.
                length = len(list(group))
                # A non-solid run of plausible opening size is a candidate gap.
                if not solid and h // 20 <= length <= h // 8:
                    # Record the row at the middle of this gap.
                    rows.append(y + row + length // 2)
                # Advance the scan position past this run.
                row += length
            # An opening is only trusted if the strip has exactly one candidate gap.
            if len(rows) != 1:
                # Ambiguous or missing opening -- bail out, this frame isn't usable yet.
                return None
            # Record this side's (column, row) opening pixel.
            openings.append((u, rows[0]))
        # Return [(u, v) for entrance, (u, v) for exit].
        return openings

    # Look up the camera's current pose and derive its column/row world-direction vectors.
    def camera_pose(self, frame_id: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Looks up the camera's pose and its column/row world directions.

        Args:
            frame_id: The TF frame the camera's image data is published in.

        Returns:
            A tuple of (camera world position, column world direction,
            row world direction); the two direction vectors are 2D (x, y).
        """
        # Get the camera's transform relative to "world" at the latest available time.
        transform = self.tf_buffer.lookup_transform('world', frame_id, Time())
        # Camera's translation component.
        t = transform.transform.translation
        # Camera's rotation component, as a quaternion.
        q = transform.transform.rotation
        # Convert that quaternion into a rotation matrix.
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        # Rotate the reference "column" axis into world space at the camera's actual orientation.
        column_dir = rotation @ LOCAL_COLUMN_AXIS
        # Rotate the reference "row" axis into world space at the camera's actual orientation.
        row_dir = rotation @ LOCAL_ROW_AXIS
        # Return camera position plus the 2D (x, y) column/row world directions.
        return np.array([t.x, t.y, t.z]), column_dir[:2], row_dir[:2]

    # Build the metric OccupancyGrid and the start/goal PoseArray from one digitized mask.
    def build_grid_and_goals(self, mask: np.ndarray, header: Header) -> tuple[OccupancyGrid, PoseArray | None]:
        """Builds the metric occupancy grid and start/goal poses from a mask.

        Args:
            mask: The binary wall mask, as returned by :meth:`digitize_mask`.
            header: The source image's header (timestamp and TF frame).

        Returns:
            A tuple of (occupancy grid, goals). ``goals`` is ``None`` if the
            entrance/exit openings weren't unambiguously found in ``mask``.
        """
        # Camera position and its column/row world-direction vectors.
        camera_xyz, column_dir, row_dir = self.camera_pose(header.frame_id)
        # Focal length (pixels), assuming square pixels (fx == fy).
        focal_length = self.camera_info.k[0]
        # Principal point (pixels).
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        # Meters-per-pixel at the maze plane, via similar triangles (camera height above the plane / focal length).
        scale = (camera_xyz[2] - MAZE_PLANE_Z_M) / focal_length

        # Start building the OccupancyGrid message.
        grid_message = OccupancyGrid()
        # Copy the incoming image's header (timestamp) as a starting point.
        grid_message.header = header
        # The grid is expressed in the world frame, not the camera's own frame.
        grid_message.header.frame_id = 'world'
        # Store the computed meters-per-pixel as the grid's resolution.
        grid_message.info.resolution = float(scale)
        # image column u <-> world y, image row v <-> world x is a
        # *reflection* (right-handed image axes onto this camera's
        # particular mount), not a rotation -- det([column_dir|row_dir]) is
        # -1, so it can't be expressed as an OccupancyGrid origin
        # orientation (a quaternion is a pure rotation, det +1) if the data
        # is stored in the image's own (u,v) layout. Storing it transposed
        # instead (grid cell (gx=v, gy=u)) turns it into a pure rotation:
        # both the grid's local axes and (column_dir, row_dir) then line up
        # via one rotation, whatever the camera's current yaw is -- see
        # project memory for the full derivation.
        # Grid width becomes the mask's pixel height (transposed storage, see comment above).
        grid_message.info.width = mask.shape[0]
        # Grid height becomes the mask's pixel width (transposed storage, see comment above).
        grid_message.info.height = mask.shape[1]
        # The grid's yaw, derived from the row direction vector, since it's now a pure rotation.
        yaw = np.arctan2(row_dir[1], row_dir[0])
        # Half-angle, needed for the quaternion's z/w components.
        half = yaw / 2.0
        # Encode the yaw as a quaternion (only z and w are nonzero for a pure yaw rotation).
        grid_message.info.origin.orientation = Quaternion(z=float(np.sin(half)), w=float(np.cos(half)))
        # World-frame position of the grid's local (0, 0) cell.
        origin_xy = camera_xyz[:2] - cx * scale * column_dir - cy * scale * row_dir
        # Store the origin's X coordinate.
        grid_message.info.origin.position.x = float(origin_xy[0])
        # Store the origin's Y coordinate.
        grid_message.info.origin.position.y = float(origin_xy[1])
        # The grid lives at the maze's floor/wall-top plane height.
        grid_message.info.origin.position.z = MAZE_PLANE_Z_M
        # Convert the 0/255 mask to 0/100 occupancy values, transpose it, and flatten it row-major for the message.
        grid_message.data = (mask.T // 255 * 100).astype('int8').flatten().tolist()

        # Find the entrance/exit opening pixels in this mask.
        openings = self.compute_goals_pixels(mask)
        # If no unambiguous openings were found, only the grid is usable this frame -- no goals yet.
        if openings is None:
            # Signal "no goals yet" to the caller.
            return grid_message, None
        # Start building the start/goal PoseArray message.
        goals_message = PoseArray()
        # Same header (frame/timestamp) as the grid.
        goals_message.header = grid_message.header
        # Convert each opening's pixel coordinate into a world-frame pose.
        for u, v in openings:
            # Project this pixel to world XY using the camera's column/row directions and scale.
            world_xy = camera_xyz[:2] + (u - cx) * scale * column_dir + (v - cy) * scale * row_dir
            # Build the Pose for this opening.
            pose = Pose()
            # World X coordinate.
            pose.position.x = float(world_xy[0])
            # World Y coordinate.
            pose.position.y = float(world_xy[1])
            # Same plane height as the grid.
            pose.position.z = MAZE_PLANE_Z_M
            # Identity orientation -- only position matters for a goal point.
            pose.orientation.w = 1.0
            # Append this opening's pose to the array.
            goals_message.poses.append(pose)
        # Return both the grid and the goals found in it.
        return grid_message, goals_message

    # Build a grid of the same size/shape as `like`, but fully occupied -- used as a "not ready yet" placeholder.
    def occupied_grid(self, like: OccupancyGrid) -> OccupancyGrid:
        """Builds a fully-occupied placeholder grid matching another one's shape.

        Args:
            like: The grid to copy header/size/origin metadata from.

        Returns:
            A new occupancy grid with the same metadata as ``like``, but
            with every cell marked occupied.
        """
        # Start a new OccupancyGrid message.
        grid_message = OccupancyGrid()
        # Reuse the reference message's header.
        grid_message.header = like.header
        # Reuse the reference message's metadata (resolution, size, origin).
        grid_message.info = like.info
        # Mark every cell as fully occupied (100).
        grid_message.data = [100] * (like.info.width * like.info.height)
        # Return the placeholder grid.
        return grid_message

    # Runs on every incoming camera frame: digitizes it and publishes a grid (frozen or placeholder).
    def image_callback(self, image_message: Image) -> None:
        """Digitizes each incoming frame and publishes a grid.

        Args:
            image_message: The latest overhead camera image.
        """
        # Guard everything so a transient bad frame doesn't crash the node.
        try:
            # Can't do the pixel<->world math without intrinsics yet.
            if self.camera_info is None:
                # Nothing to do until the first CameraInfo arrives.
                return
            # If already locked onto a stable frame, just keep republishing the frozen result.
            if self.triggered:
                # Republish the frozen grid so late subscribers (e.g. RViz) still see it.
                self.grid_publisher.publish(self.frozen_grid)
                # Republish the frozen goals as well.
                self.goals_publisher.publish(self.frozen_goals)
                # Nothing else to do this frame.
                return
            # Decode this frame to a BGR OpenCV image.
            image = self.decode_image(image_message)
            # Threshold it to the green-wall binary mask.
            mask = self.digitize_mask(image)
            # Convert to a plain Python list so it can be compared/stored cheaply.
            mask_data = mask.tolist()
            # The arm crossing the camera's view mid-motion changes the mask
            # every frame (it isn't green, so it just looks like missing
            # wall), so only trust a frame once several in a row agree -- the
            # arm sitting still that long only happens before motion starts
            # or once it's safely back at rest between mazes.
            # If this mask matches the previous candidate, extend the stability streak.
            if mask_data == self.previous_frame:
                # One more frame agreeing with the candidate.
                self.stable_count += 1
            else:
                # A different mask -- restart the stability streak with this one as the new candidate.
                self.previous_frame = mask_data
                # Reset the streak to 1 (this frame itself).
                self.stable_count = 1
            # Enough consecutive matching frames -- trust this mask and lock on.
            if self.stable_count >= STABLE_FRAMES:
                # Build the grid and goals from this now-trusted mask.
                grid_message, goals_message = self.build_grid_and_goals(mask, image_message.header)
                # Only lock on if goals (entrance/exit) were actually found.
                if goals_message is not None:
                    # Freeze this grid for the rest of the run.
                    self.frozen_grid = grid_message
                    # Freeze these goals for the rest of the run.
                    self.frozen_goals = goals_message
                    # Mark this perception cycle as locked on.
                    self.triggered = True
                    # Publish the frozen grid immediately.
                    self.grid_publisher.publish(self.frozen_grid)
                    # Publish the frozen goals immediately.
                    self.goals_publisher.publish(self.frozen_goals)
                    # Unpack the two goal poses for the log message.
                    start, goal = goals_message.poses
                    # Log the locked-on start/goal coordinates.
                    self.get_logger().info(
                        f'triggered on a stable frame: start ({start.position.x:.3f}, {start.position.y:.3f}) '
                        f'goal ({goal.position.x:.3f}, {goal.position.y:.3f})')
                    # Done for this frame.
                    return
            # Not locked on yet -- publish a fully-occupied placeholder grid instead of nothing.
            placeholder = self.occupied_grid(self.build_grid_and_goals(mask, image_message.header)[0])
            # Publish the placeholder grid.
            self.grid_publisher.publish(placeholder)
        # Swallow any unexpected error so one bad frame doesn't take the node down.
        except Exception:
            # Silently skip this frame.
            pass

# Standard ROS2 Python entry point.
def main(args: list[str] | None = None) -> None:
    """Initializes rclpy, spins PerceptionNode, and shuts down on exit.

    Args:
        args: Command-line arguments forwarded to rclpy, or ``None`` to use
            ``sys.argv``.
    """
    # Initialize the rclpy context.
    r.init(args=args)
    # Construct the node.
    node = PerceptionNode()
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
