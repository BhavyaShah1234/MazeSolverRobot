import numpy as np

class Converter:
    def __init__(self, depth, img_h, img_w, fov_h, fov_w, tx, ty, tz):
        self.depth = depth
        self.cm_per_pixel_w = fov_w / img_w
        self.cm_per_pixel_h = fov_h / img_h
        self.camera_to_robot = np.array([[0, -1, 0, tx], [-1, 0, 0, ty], [0, 0, -1, tz], [0, 0, 0, 1]])

    def pixel_to_robot(self, x_pixel, y_pixel):
        x_camera, y_camera, z_camera = x_pixel * self.cm_per_pixel_w, y_pixel * self.cm_per_pixel_h, self.depth
        x_robot, y_robot, z_robot = np.reshape(np.matmul(self.camera_to_robot, np.expand_dims(np.array([x_camera, y_camera, z_camera, 1]), axis=1))[:-1], newshape=[-1]).tolist()
        return x_robot, y_robot, z_robot
