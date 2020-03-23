import pyrealsense2 as rs
import numpy as np
# import matplotlib.pyplot as plt
import cv2
import time

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor


if __name__ == "__main__":

    # D435 = D435Sensor(is_device=True, source_name='845112070910')0000943222110531
    # T265 = T265Sensor(is_device=True, source_name='943222110531')  # change
    D435 = D435Sensor(is_device=False, source_name='data/435.bag')
    T265 = T265Sensor(is_device=False, source_name='data/265.bag')
    D435.attach(T265)

    # # TODO: allow writing to files here
    # D435.allow_writing_to_file("D435.bag")
    # T265.allow_writing_to_file("T265.bag")

    D435.start_sensor()
    T265.start_sensor()

    try:
        while True:
            image = D435.get_image()


            if image is not None:
                # print(image.shape)
                pose = T265.get_coordinates()
                print(pose)
                cv2.imshow('D435 RGB Frame', image)
                cv2.waitKey(1000)

            # time.sleep(1)
            # actions here
            pass
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()


