import bstar as b
import mapper as m
import frame_converter as f
import inverse_kinematics as k

if __name__ == '__main__':
    GRID_H, GRID_W = 80, 80
    OBSTACLE_PENALTY = 100
    DEPTH = 100
    FOV_H, FOV_W = 20, 20
    TX, TY, TZ = 0, 0, 0
    URDF_FILE = 'myCobot_Pro_600_2.urdf'
    mapper = m.GridMapper('maze3.png')
    grid = mapper.get_grid(grid_h=GRID_H, grid_w=GRID_W)
    planner = b.Bstar(grid, OBSTACLE_PENALTY)
    path = planner.find_path()
    # grid.plot_grid(path, pause_time=10, text=False)
    path = mapper.interpolate_path(path, GRID_H, GRID_W)
    mapper.plot_path_on_image(path)
    converter = f.Converter(DEPTH, mapper.img_h, mapper.img_w, FOV_H, FOV_W, TX, TY, TZ)
    solver = k.InverseKinematicsSolver(URDF_FILE)
    for time in path:
        x_robot, y_robot, z_robot = converter.pixel_to_robot(path[time]['x'], path[time]['y'])
        angles = solver.get_angles(x_robot, y_robot, z_robot)
