import cv2
import numpy as np
import rclpy as r
import tf2_ros
from itertools import groupby
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty

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
LOCAL_COLUMN_AXIS = np.array([0.0, -1.0, 0.0])
LOCAL_ROW_AXIS = np.array([0.0, 0.0, -1.0])

def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

class PerceptionNode(Node):
    def __init__(self):
        super(PerceptionNode, self).__init__(node_name='perception_node')
        self.bridge = CvBridge()
        self.camera_info = None
        self.triggered = False
        self.candidate_data = None
        self.stable_count = 0
        self.frozen_grid = None
        self.frozen_goals = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, '/overhead_camera/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Empty, '/perception/start', self.start_callback, 10)
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/maze_occupancy_grid', grid_qos)
        self.goals_publisher = self.create_publisher(PoseArray, '/goals', grid_qos)
        self.get_logger().info('perception_node started')

    def camera_info_callback(self, camera_info_message):
        self.camera_info = camera_info_message

    def start_callback(self, empty_message):
        self.triggered = False
        self.candidate_data = None
        self.stable_count = 0
        self.frozen_grid = None
        self.frozen_goals = None
        self.get_logger().info('perception restarted, looking for a new stable frame')

    def decode_image(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message)
        if image_message.encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if image_message.encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if image_message.encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def digitize_mask(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, (35, 40, 40), (85, 255, 255))

    def compute_goals_pixels(self, mask):
        wall = (mask > 0).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(wall)
        keep = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 0.05 * stats[1:, cv2.CC_STAT_AREA].sum()]
        wall = np.isin(labels, keep)
        x, y, w, h = cv2.boundingRect(wall.astype(np.uint8))
        band = max(4, w // 50)
        openings = []
        # start is the opening in the left border (world y_min, the maze's
        # entrance), goal the one in the right border
        for u, columns in ((x + band // 2, slice(x, x + band)), (x + w - 1 - band // 2, slice(x + w - band, x + w))):
            rows, row = [], 0
            for solid, group in groupby(wall[y:y + h, columns].any(axis=1)):
                length = len(list(group))
                if not solid and h // 20 <= length <= h // 8:
                    rows.append(y + row + length // 2)
                row += length
            if len(rows) != 1:
                return None
            openings.append((u, rows[0]))
        return openings

    def camera_pose(self, frame_id):
        transform = self.tf_buffer.lookup_transform('world', frame_id, Time())
        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        column_dir = rotation @ LOCAL_COLUMN_AXIS
        row_dir = rotation @ LOCAL_ROW_AXIS
        return np.array([t.x, t.y, t.z]), column_dir[:2], row_dir[:2]

    def build_grid_and_goals(self, mask, header):
        camera_xyz, column_dir, row_dir = self.camera_pose(header.frame_id)
        focal_length = self.camera_info.k[0]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        scale = (camera_xyz[2] - MAZE_PLANE_Z_M) / focal_length

        grid_message = OccupancyGrid()
        grid_message.header = header
        grid_message.header.frame_id = 'world'
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
        grid_message.info.width = mask.shape[0]
        grid_message.info.height = mask.shape[1]
        yaw = np.arctan2(row_dir[1], row_dir[0])
        half = yaw / 2.0
        grid_message.info.origin.orientation = Quaternion(z=float(np.sin(half)), w=float(np.cos(half)))
        origin_xy = camera_xyz[:2] - cx * scale * column_dir - cy * scale * row_dir
        grid_message.info.origin.position.x = float(origin_xy[0])
        grid_message.info.origin.position.y = float(origin_xy[1])
        grid_message.info.origin.position.z = MAZE_PLANE_Z_M
        grid_message.data = (mask.T // 255 * 100).astype('int8').flatten().tolist()

        openings = self.compute_goals_pixels(mask)
        if openings is None:
            return grid_message, None
        goals_message = PoseArray()
        goals_message.header = grid_message.header
        for u, v in openings:
            world_xy = camera_xyz[:2] + (u - cx) * scale * column_dir + (v - cy) * scale * row_dir
            pose = Pose()
            pose.position.x = float(world_xy[0])
            pose.position.y = float(world_xy[1])
            pose.position.z = MAZE_PLANE_Z_M
            pose.orientation.w = 1.0
            goals_message.poses.append(pose)
        return grid_message, goals_message

    def occupied_grid(self, like):
        grid_message = OccupancyGrid()
        grid_message.header = like.header
        grid_message.info = like.info
        grid_message.data = [100] * (like.info.width * like.info.height)
        return grid_message

    def image_callback(self, image_message):
        try:
            if self.camera_info is None:
                return
            if self.triggered:
                self.grid_publisher.publish(self.frozen_grid)
                self.goals_publisher.publish(self.frozen_goals)
                return
            image = self.decode_image(image_message)
            mask = self.digitize_mask(image)
            mask_data = mask.tolist()
            # The arm crossing the camera's view mid-motion changes the mask
            # every frame (it isn't green, so it just looks like missing
            # wall), so only trust a frame once several in a row agree -- the
            # arm sitting still that long only happens before motion starts
            # or once it's safely back at rest between mazes.
            if mask_data == self.candidate_data:
                self.stable_count += 1
            else:
                self.candidate_data = mask_data
                self.stable_count = 1
            if self.stable_count >= STABLE_FRAMES:
                grid_message, goals_message = self.build_grid_and_goals(mask, image_message.header)
                if goals_message is not None:
                    self.frozen_grid = grid_message
                    self.frozen_goals = goals_message
                    self.triggered = True
                    self.grid_publisher.publish(self.frozen_grid)
                    self.goals_publisher.publish(self.frozen_goals)
                    start, goal = goals_message.poses
                    self.get_logger().info(
                        f'triggered on a stable frame: start ({start.position.x:.3f}, {start.position.y:.3f}) '
                        f'goal ({goal.position.x:.3f}, {goal.position.y:.3f})')
                    return
            placeholder = self.occupied_grid(self.build_grid_and_goals(mask, image_message.header)[0])
            self.grid_publisher.publish(placeholder)
        except Exception:
            pass

def main(args=None):
    r.init(args=args)
    node = PerceptionNode()
    r.spin(node)
    node.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
