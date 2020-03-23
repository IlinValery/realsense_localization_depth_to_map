import time

import numpy as np

from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseObserver, BaseSubject
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R


class T265Sensor(BaseSensor, BaseObserver):
    def __init__(self, is_device, source_name):
        # sensor cofig
        super(T265Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.pose)

        self.frameset = None
        self.pose = None
        self.sync_pose = None

    def parent_update(self, subject: BaseSubject) -> None:
        self.sync_pose = self.pose

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        self.process_frameset()

    def process_frameset(self):
        # TODO process necessary data
        self.pose = self.frameset.get_pose_frame()

    def get_coordinates(self):
        data = self.sync_pose.get_pose_data()
        data_rot = [float(i.strip('xyzw: ')) for i in str(data.rotation).split(', ')]
        r = R.from_quat(data_rot)
        rotation = np.array(r.as_matrix())
        translation = np.array([float(i.strip('xyzw: ')) for i in str(data.translation).split(', ')])[np.newaxis].T
        T = np.hstack((rotation, translation))
        T = np.vstack((T, np.array([0, 0, 0, 1])))
        return T
