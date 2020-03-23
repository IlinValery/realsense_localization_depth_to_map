import time

import numpy as np

from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseSubject, BaseObserver
from typing import List
import pyrealsense2 as rs


class D435Sensor(BaseSensor, BaseSubject):
    def __init__(self, is_device, source_name):
        # initialize of sensor
        super(D435Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)


        # initialize observers
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # TODO: insert initial conditions here:
        self.frameset = None
        self.color_image = None

    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.parent_update(self)

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        self.process_frameset()
        self.notify()

    def process_frameset(self):
        # TODO extract necessary data here:
        color_frame = self.frameset.get_color_frame()
        self.color_image = np.asanyarray(color_frame.get_data())

    # TODO: get functions here:
    def get_image(self):
        return self.color_image



