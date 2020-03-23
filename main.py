import pyrealsense2 as rs
import numpy as np
# import matplotlib.pyplot as plt
import cv2
import time

from sensors.d435_sensor import D435Sensor
from sensors.t265_sensor import T265Sensor

if __name__ == "__main__":

    D435 = D435Sensor()
    T265 = T265Sensor()
    D435.attach(T265)

    D435.start_sensor()
    T265.start_sensor()

    try:
        while True:
            # actions here
            pass
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()

