import numpy as np
from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseObserver, BaseSubject
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R


class T265Sensor(BaseSensor, BaseObserver, BaseSubject):
    def __init__(self, is_device, source_name):

        # Initialization of D435 sensor
        super(T265Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.pose)

        # initialize observers
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # TODO: insert initial conditions here:
        self.frameset = None
        self.pose = None
        self.sync_pose = None
        # self.increment = 0
        # self.frameset_list = []

    def on_parent_update(self, subject: BaseSubject) -> None:
        self.sync_pose = self.pose

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        self.process_frameset()
        self.notify()

    def process_frameset(self):
        # TODO process necessary data
        self.pose = self.frameset.get_pose_frame()
        # self.increment+=1
        # self.frameset_list.append(self.frameset.copy())

    def get_transformation(self):
        if self.sync_pose is not None:
            data = self.sync_pose.get_pose_data()
            data_rot = [float(i.strip('xyzw: ')) for i in str(data.rotation).split(', ')]
            r = R.from_quat(data_rot)
            rotation = np.array(r.as_matrix())
            translation = np.array([float(i.strip('xyzw: ')) for i in str(data.translation).split(', ')])[np.newaxis].T
            T = np.hstack((rotation, translation))
            T = np.vstack((T, np.array([0, 0, 0, 1])))
            return T

    def get_sync_pose(self):
        return self.sync_pose

    def get_pose(self):
        return self.pose

    def get_increment(self):
        return self.increment


    # Методы ИЗДАТЕЛя
    # Набор методов для управлениями подпискичами.
    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.on_parent_update(self)