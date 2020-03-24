import pyrealsense2 as rs
import numpy as np
# import matplotlib.pyplot as plt
import cv2
import time

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor


if __name__ == "__main__":

    D435 = D435Sensor(is_device=True, source_name='845112070910')  # 
    T265 = T265Sensor(is_device=True, source_name='whatever')  # change 0000943222110531
    # D435 = D435Sensor(is_device=False, source_name='data/435.bag')
    # T265 = T265Sensor(is_device=False, source_name='data/265.bag')
    D435.attach(T265)

    # TODO: allow writing to files here
    # D435.allow_writing_to_file("D435.bag")
    # T265.allow_writing_to_file("T265.bag")

    T265.start_sensor()
    D435.start_sensor()


    try:
        while True:

            color_frame, depth_frame = D435.get_frames()
            pose265 = T265.get_pose()
            if (color_frame is not None) and (pose265 is not None):
                print('\nframeset435 gtab time', color_frame.get_timestamp())
                print('pose265 grab time    ', pose265.get_timestamp())

                transformation_matrix = T265.get_transformation()
                # print('transformation_matrix',transformation_matrix)

                color_image, depth_image = D435.get_images()
                cv2.imshow('D435 RGB Frame', color_image)
                depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                cv2.imshow('D435 Depth Frame', depth_image)
                cv2.waitKey(100)

            # actions here
            pass
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()


