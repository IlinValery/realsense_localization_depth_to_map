import pyrealsense2 as rs
import numpy as np
# import matplotlib.pyplot as plt
import cv2
import time

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor

if __name__ == "__main__":

    # D435 = D435Sensor(is_device=True, source_name='845112070910')
    D435 = D435Sensor(is_device=False, source_name='data/435.bag')
    T265 = T265Sensor(is_device=False, source_name='data/265.bag')  # change
    D435.attach(T265)

    D435.start_sensor()
    T265.start_sensor()

    try:
        while True:
            image = D435.get_image()

            if image is not None:
                print(image.shape)
                cv2.imshow('new', image)

            time.sleep(1)
            # actions here
            pass
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()

    print(D435.numbers)
    print(T265.numbers)
