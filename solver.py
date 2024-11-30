import ikpy
import math
import numpy as np

class InverseKinematicsSolver:
    def __init__(self, urdf_file):
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_file)

    def get_angles(self, x, y, z):
        angles = self.chain.inverse_kinematics(np.array([x, y, z]), target_orientation=np.array([0, 0, math.pi / 2]), orientation_mode='Z')
        angles = np.rad2deg(angles).tolist()
        return angles
