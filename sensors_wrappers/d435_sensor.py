import time

import numpy as np

from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseSubject, BaseObserver
from typing import List
import pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt

class D435Sensor(BaseSensor, BaseSubject):
    def __init__(self, is_device, source_name):
        # initialize of sensor
        super(D435Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        self.pipe.start(self.cfg)

        # initialize observers
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # initial conditions
        self.frameset = None
        self.color_image = None


        self.number = 0
        self.numbers = []

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        color_frame = self.frameset.get_color_frame()
        # depth_frame = self.frameset.get_depth_frame()
        self.color_image = np.asanyarray(color_frame.get_data())

        # cv2.imshow('new_window', color)
        # plt.plot()
        # plt.imshow(self.color_image)
        # plt.show()
        # update initial conditions (this function was wrapped in while)
        self.number += 1

        if (self.number % 30 == 0):
            # print("sensor d435 Updated", self.number)
            # publish on other sensor notify
            self.numbers.append(self.number)
            self.notify()

        time.sleep(1/30) # as 30Hz

    def get_image(self):
        return self.color_image

    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.parent_update(self)

