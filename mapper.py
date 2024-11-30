import cv2 as cv
import grid as g
import numpy as np

class GridMapper:
    def __init__(self, img_path):
        self.img_path = img_path
        self.img_h, self.img_w, _ = cv.imread(self.img_path).shape
        self.start = None
        self.end = None

    def put_start_and_end(self, event, x, y, flag, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.start = (x / param.shape[1], y / param.shape[0])
        elif event == cv.EVENT_RBUTTONDOWN:
            self.end = (x / param.shape[1], y / param.shape[0])

    def get_grid(self, grid_h=50, grid_w=50, show=False):
        while True:
            if self.start is None or self.end is None:
                frame = cv.imread(self.img_path)
                frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                frame = cv.dilate(frame, kernel=np.ones(shape=(3, 3)), iterations=3)
                frame = cv.erode(frame, kernel=(3, 3), iterations=2)
                cv.imshow('Frame', frame)
                cv.setMouseCallback('Frame', self.put_start_and_end, frame)
                if cv.waitKey(1) == 27:
                    break
            else:
                break
        frame = cv.resize(frame, (grid_w, grid_h))
        self.start = (int(self.start[0] * frame.shape[1]), int(self.start[1] * frame.shape[0]))
        self.end = (int(self.end[0] * frame.shape[1]), int(self.end[1] * frame.shape[0]))
        grid = g.Grid(frame.shape[0], frame.shape[1])
        grid.put_start(self.start[0], self.start[1])
        grid.put_end(self.end[0], self.end[1])
        obstacles_y, obstacles_x = np.where(frame < 255)
        obstacles_x = np.expand_dims(obstacles_x, axis=-1)
        obstacles_y = np.expand_dims(obstacles_y, axis=-1)
        obstacles = np.concatenate([obstacles_x, obstacles_y], axis=-1).tolist()
        for i, (x, y) in enumerate(obstacles):
            grid.put_obstacle(i, x, y, 0, 0, (-2, 2), (-2, 2))
        if show:
            grid.plot_grid(pause_time=10)
        return grid

    def interpolate_path(self, path, grid_h, grid_w):
        new_path = {}
        for time in path:
            x = path[time]['x'] * self.img_w / grid_w
            y = path[time]['y'] * self.img_h / grid_h
            new_path[time] = {'x': x, 'y': y}
        return new_path

    def plot_path_on_image(self, path):
        img = cv.imread(self.img_path)
        for time in path:
            img = cv.circle(img, (int(path[time]['x']), int(path[time]['y'])), 2, (0, 0, 255), -1)
        cv.imshow('Result', img)
        if cv.waitKey(0) == 27:
            cv.destroyAllWindows()
