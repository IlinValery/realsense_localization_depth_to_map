# TODO: Valery 3D points visualisation
import numpy as np
import pptk
from helpers.base_observer import BaseObserver, BaseSubject


class PointCloudVisualizer(BaseObserver):
    def __init__(self, update_each_frames=15):
        self.show_each_frames_number = update_each_frames
        self.current_frame_number = 0

        self.viewer = None
        self.points = np.random.rand(100, 3)
        self.camera_pose = None
        self.is_changed = True

    def __del__(self):
        if self.viewer is not None:
            self.viewer.close()

    def set_points(self, new_points=None):
        self.is_changed = True
        if new_points is not None:
            self.points = new_points
        else:
            self.points = np.random.rand(100000, 3)

    def set_camera_pose(self, camera_matrix):
        # TODO: setup camera pose
        pass

    def update_visualisation(self):
        self.current_frame_number -= 1
        if self.current_frame_number < 0:
            self.current_frame_number = self.show_each_frames_number
            if self.viewer is None:
                self.viewer = pptk.viewer(self.points)
            else:
                self.viewer.load(self.points, )

            if self.camera_pose is not None:
                pass

    def on_parent_update(self, subject: BaseSubject) -> None:
        self.update_visualisation()
